#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h" // IWYU pragma: keep
#include "esp_err.h"
#include "esp_log.h"

/*
 * ESP32-S3 Lifting Assistive Device Template
 *
 * Inputs:
 * - Pressure Pad UP (ADC): command upward movement
 * - Pressure Pad DOWN (ADC): command downward movement
 * - Position Potentiometer (ADC): arm position safety feedback
 *
 * Output:
 * - VNH7070 PWM + direction pins
 *
 * Behavior:
 * - Pressure value maps to PWM duty command
 * - Duty is clamped to 25% max
 * - Duty ramps up/down smoothly
 * - Movement is blocked when position limits are exceeded
 */

static const char *TAG = "LAD_CTRL";

/* ---------- Pin/Channel Configuration ----------
 * Mapped from provided schematic:
 * - PRESSURE_PAD_1 -> GPIO1 (IO1)
 * - PRESSURE_PAD_2 -> GPIO2 (IO2)
 * - MOTOR_PWM      -> GPIO4 (IO4)
 * - MOTOR_CW       -> GPIO5 (IO5)
 * - MOTOR_CCW      -> GPIO6 (IO6)
 * - MOTOR_SEL      -> GPIO7 (IO7)
 * - MOTOR_CS       -> GPIO12 (IO12)
 * - AXIS_POT       -> GPIO11 (IO11)
 */
#define MOTOR_PWM_GPIO           GPIO_NUM_4
#define MOTOR_INA_GPIO           GPIO_NUM_5
#define MOTOR_INB_GPIO           GPIO_NUM_6
#define MOTOR_SEL_GPIO           GPIO_NUM_7

/* ESP32-S3 ADC mapping used by the schematic pins above. */
#define PRESSURE_UP_ADC_UNIT     ADC_UNIT_1
#define PRESSURE_UP_ADC_CH       ADC_CHANNEL_0 /* GPIO1 */

#define PRESSURE_DOWN_ADC_UNIT   ADC_UNIT_1
#define PRESSURE_DOWN_ADC_CH     ADC_CHANNEL_1 /* GPIO2 */

#define POSITION_ADC_UNIT        ADC_UNIT_2
#define POSITION_ADC_CH          ADC_CHANNEL_0 /* GPIO11 */

#define MOTOR_CS_ADC_UNIT        ADC_UNIT_2
#define MOTOR_CS_ADC_CH          ADC_CHANNEL_1 /* GPIO12 */

/* ---------- Control Constants ---------- */
#define CONTROL_PERIOD_MS        20

/* ADC is configured to 12-bit width -> raw range 0..4095 */
#define ADC_RAW_MAX              4095

/* Ignore slight resting pressure/noise from pads */
#define PRESSURE_DEADZONE_RAW    200

/* Max duty cap */
#define MAX_DUTY_PERCENT         50.0f

/* Ramp speed: max duty-percent change per second */
#define RAMP_PERCENT_PER_SEC     100.0f

/* Minimum command needed to consider motion request valid */
#define MIN_CMD_DUTY_PERCENT     0.5f

/* Require opposite pad to be near idle before allowing movement */
#define OTHER_PAD_IDLE_MAX_RAW   100

/* ---------- MCPWM (motor-control PWM) Configuration ---------- */
#define MOTOR_PWM_FREQ_HZ                20000
#define MCPWM_TIMER_RESOLUTION_HZ        10000000
#define MOTOR_SEL_ACTIVE_LEVEL           1

/* Current-sense safety threshold (ADC raw 0..4095). Tune to your hardware. */
#define MOTOR_CS_OVERCURRENT_RAW         805 /* ~5A or .65V adc read */

typedef struct {
	/* Potentiometer thresholds (ADC raw) */
	int lower_limit_raw;
	int upper_limit_raw;
} travel_limits_t;

static travel_limits_t g_limits = {
	.lower_limit_raw = 600,
	.upper_limit_raw = 3500,
};

static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_oneshot_unit_handle_t s_adc2 = NULL;

static mcpwm_timer_handle_t s_mcpwm_timer = NULL;
static mcpwm_oper_handle_t s_mcpwm_oper = NULL;
static mcpwm_cmpr_handle_t s_mcpwm_comparator = NULL;
static mcpwm_gen_handle_t s_mcpwm_generator = NULL;

static float s_current_duty_percent = 0.0f;

/*
 * clampf
 * Purpose: Constrain a float value to a safe inclusive range.
 * How: Returns the low bound when underflowing, high bound when overflowing, else input.
 * Why: Keeps control calculations bounded and avoids duplicated clamp logic.
 */
static inline float clampf(float x, float lo, float hi)
{
	/* Enforce lower and upper bounds before returning the value. */
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

/*
 * clampi
 * Purpose: Constrain an integer value to a safe inclusive range.
 * How: Uses branch checks against low and high limits.
 * Why: Prevents invalid ADC or configuration values from propagating.
 */
static inline int clampi(int x, int lo, int hi)
{
	/* Use the same saturation pattern as clampf for integer paths. */
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

/*
 * set_travel_limits_raw
 * Purpose: Update software travel limits used by position safety checks.
 * How: Clamps requested thresholds, validates ordering, and commits to global state.
 * Why: Allows calibration changes at runtime without recompiling.
 */
static void set_travel_limits_raw(int lower_raw, int upper_raw)
{
	/* Clamp requested values to the physical ADC range. */
	lower_raw = clampi(lower_raw, 0, ADC_RAW_MAX);
	upper_raw = clampi(upper_raw, 0, ADC_RAW_MAX);

	/* Reject invalid ranges so motion limits stay meaningful. */
	if (lower_raw >= upper_raw) {
		ESP_LOGW(TAG, "Invalid limits requested: lower=%d upper=%d", lower_raw, upper_raw);
		return;
	}

	/* Persist validated limits used by the control loop. */
	g_limits.lower_limit_raw = lower_raw;
	g_limits.upper_limit_raw = upper_raw;
	ESP_LOGI(TAG, "Updated travel limits: lower=%d upper=%d", lower_raw, upper_raw);
}

/*
 * motor_pwm_set_percent
 * Purpose: Set motor command duty cycle in percent.
 * How: Clamps duty, converts percent to timer ticks, and updates MCPWM compare value.
 * Why: Encapsulates PWM math so all callers use the same safe scaling.
 */
static esp_err_t motor_pwm_set_percent(float duty_percent)
{
	/* Bound user command to configured max duty for safety. */
	duty_percent = clampf(duty_percent, 0.0f, MAX_DUTY_PERCENT);
	/* Compute timer period ticks from resolution and frequency. */
	const uint32_t period_ticks = MCPWM_TIMER_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ;
	/* Convert duty percent into compare ticks for hardware PWM. */
	const uint32_t compare_ticks = (uint32_t)((duty_percent / 100.0f) * (float)period_ticks);
	return mcpwm_comparator_set_compare_value(s_mcpwm_comparator, compare_ticks);
}

/*
 * motor_set_direction_up
 * Purpose: Drive the H-bridge for upward motion.
 * How: Sets INA high and INB low on motor direction GPIOs.
 * Why: Keeps direction polarity mapping centralized and explicit.
 */
static esp_err_t motor_set_direction_up(void)
{
	/* Apply the pin polarity that corresponds to upward travel. */
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INA_GPIO, 1));
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INB_GPIO, 0));
	return ESP_OK;
}

/*
 * motor_set_direction_down
 * Purpose: Drive the H-bridge for downward motion.
 * How: Sets INA low and INB high on motor direction GPIOs.
 * Why: Mirrors upward mapping and avoids duplicated pin logic.
 */
static esp_err_t motor_set_direction_down(void)
{
	/* Apply the pin polarity that corresponds to downward travel. */
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INA_GPIO, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INB_GPIO, 1));
	return ESP_OK;
}

/*
 * motor_stop
 * Purpose: Put motor outputs into a safe neutral stop.
 * How: Clears both direction pins, keeps motor select active, and forces 0% PWM.
 * Why: Provides one consistent stop path for neutral state and fault handling.
 */
static esp_err_t motor_stop(void)
{
	/* Neutral drive state removes commanded motor direction. */
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INA_GPIO, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INB_GPIO, 0));
	/* Keep selector pin in active mode expected by the driver. */
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_SEL_GPIO, MOTOR_SEL_ACTIVE_LEVEL));
	return motor_pwm_set_percent(0.0f);
}

/*
 * pressure_raw_to_percent
 * Purpose: Convert pressure pad ADC input into a duty command.
 * How: Removes deadzone, normalizes over remaining span, then scales to max duty.
 * Why: Gives proportional control while ignoring light resting pressure noise.
 */
static float pressure_raw_to_percent(int pressure_raw)
{
	/* Remove low-level noise floor so idle pressure reads as zero. */
	int adjusted = pressure_raw - PRESSURE_DEADZONE_RAW;
	if (adjusted <= 0) {
		return 0.0f;
	}

	/* Normalize remaining pressure span into the 0..1 range. */
	int span = ADC_RAW_MAX - PRESSURE_DEADZONE_RAW;
	float normalized = (float)adjusted / (float)span;
	normalized = clampf(normalized, 0.0f, 1.0f);

	/* Scale normalized pressure to the configured maximum duty limit. */
	return normalized * MAX_DUTY_PERCENT;
}

/*
 * ramp_toward
 * Purpose: Limit how fast a command can change between loop iterations.
 * How: Steps toward target by at most max_step, otherwise snaps to target.
 * Why: Reduces abrupt motor torque changes and improves smoothness.
 */
static float ramp_toward(float current, float target, float max_step)
{
	/* Slew-limit duty updates to avoid sudden command jumps. */
	if (target > current + max_step) return current + max_step;
	if (target < current - max_step) return current - max_step;
	return target;
}

/*
 * read_adc_raw
 * Purpose: Read one ADC channel and return a bounded raw value.
 * How: Selects the correct ADC unit handle, reads oneshot sample, clamps result.
 * Why: Centralizes ADC error handling and consistent range enforcement.
 */
static int read_adc_raw(adc_unit_t unit, adc_channel_t channel)
{
	int raw = 0;
	/* Route request to ADC1 or ADC2 based on the provided unit. */
	adc_oneshot_unit_handle_t handle = (unit == ADC_UNIT_1) ? s_adc1 : s_adc2;
	esp_err_t err = adc_oneshot_read(handle, channel, &raw);
	/* Return a safe default on read failure and emit a diagnostic log. */
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "adc_oneshot_read failed on unit %d channel %d: %s", unit, channel, esp_err_to_name(err));
		return 0;
	}
	/* Clamp to legal 12-bit range before exposing the value. */
	return clampi(raw, 0, ADC_RAW_MAX);
}

/*
 * init_gpio
 * Purpose: Configure motor control pins and set safe startup states.
 * How: Initializes INA/INB/SEL as outputs and drives them to neutral defaults.
 * Why: Prevents undefined pin states from causing unintended movement.
 */
static esp_err_t init_gpio(void)
{
	/* Configure all motor-control GPIO lines in one transaction. */
	const gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << MOTOR_INA_GPIO) | (1ULL << MOTOR_INB_GPIO) | (1ULL << MOTOR_SEL_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	/* Apply pin configuration before setting deterministic output levels. */
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INA_GPIO, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_INB_GPIO, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTOR_SEL_GPIO, MOTOR_SEL_ACTIVE_LEVEL));
	return ESP_OK;
}

/*
 * init_pwm
 * Purpose: Build and start MCPWM resources used for motor speed control.
 * How: Creates timer/operator/comparator/generator, binds actions, then enables timer.
 * Why: Hardware PWM provides stable frequency and low-jitter duty updates.
 */
static esp_err_t init_pwm(void)
{
	/* Define base timer used for PWM period and resolution. */
	const mcpwm_timer_config_t timer_config = {
		.group_id = 0,
		.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
		.resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
		.period_ticks = MCPWM_TIMER_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ,
		.count_mode = MCPWM_TIMER_COUNT_MODE_UP,
	};
	/* Create timer object and store handle for later control. */
	ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &s_mcpwm_timer));

	/* Create operator that owns comparator and generator resources. */
	const mcpwm_operator_config_t oper_config = {
		.group_id = 0,
	};
	ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &s_mcpwm_oper));
	/* Link the operator to the configured timer. */
	ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_mcpwm_oper, s_mcpwm_timer));

	/* Comparator updates pulse width by defining duty transition point. */
	const mcpwm_comparator_config_t comparator_config = {
		.flags.update_cmp_on_tez = true,
	};
	ESP_ERROR_CHECK(mcpwm_new_comparator(s_mcpwm_oper, &comparator_config, &s_mcpwm_comparator));

	/* Generator routes operator output to the motor PWM GPIO pin. */
	const mcpwm_generator_config_t generator_config = {
		.gen_gpio_num = MOTOR_PWM_GPIO,
	};
	ESP_ERROR_CHECK(mcpwm_new_generator(s_mcpwm_oper, &generator_config, &s_mcpwm_generator));

	/* Set PWM high at cycle start and low at compare for duty control. */
	ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
		s_mcpwm_generator,
		MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
	ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
		s_mcpwm_generator,
		MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_mcpwm_comparator, MCPWM_GEN_ACTION_LOW)));

	/* Start from 0% duty and begin continuous timer operation. */
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_mcpwm_comparator, 0));
	ESP_ERROR_CHECK(mcpwm_timer_enable(s_mcpwm_timer));
	ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_mcpwm_timer, MCPWM_TIMER_START_NO_STOP));
	return ESP_OK;
}

/*
 * init_adc
 * Purpose: Initialize ADC oneshot units and configure all active channels.
 * How: Creates ADC1 and ADC2 units, then applies shared 12-bit channel settings.
 * Why: Keeps all analog inputs configured consistently for control logic.
 */
static esp_err_t init_adc(void)
{
	/* Create ADC1 handle used for both pressure pad channels. */
	const adc_oneshot_unit_init_cfg_t unit1_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit1_cfg, &s_adc1));

	/* Create ADC2 handle used for position and current-sense channels. */
	const adc_oneshot_unit_init_cfg_t unit2_cfg = {
		.unit_id = ADC_UNIT_2,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit2_cfg, &s_adc2));

	/* Use one channel configuration so all inputs share scaling behavior. */
	const adc_oneshot_chan_cfg_t chan_cfg = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_12,
	};

	/* Configure each logical sensor channel with the common settings. */
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, PRESSURE_UP_ADC_CH, &chan_cfg));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, PRESSURE_DOWN_ADC_CH, &chan_cfg));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc2, POSITION_ADC_CH, &chan_cfg));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc2, MOTOR_CS_ADC_CH, &chan_cfg));
	return ESP_OK;
}

/*
 * app_main
 * Purpose: Execute the periodic motor control loop for the lifting device.
 * How: Initializes hardware, reads sensors, arbitrates command direction, applies safety,
 *      ramps duty, and writes motor outputs every control period.
 * Why: Combines user intent and protective checks into a deterministic control task.
 */
void app_main(void)
{
	/* Initialize peripherals first so later control calls are valid. */
	ESP_ERROR_CHECK(init_gpio());
	ESP_ERROR_CHECK(init_pwm());
	ESP_ERROR_CHECK(init_adc());
	ESP_ERROR_CHECK(motor_stop());

	/* Push default limits through validation path at startup. */
	set_travel_limits_raw(g_limits.lower_limit_raw, g_limits.upper_limit_raw);

	/* Precompute loop timing values used by slew-rate limiting. */
	const float dt_sec = (float)CONTROL_PERIOD_MS / 1000.0f;
	const float max_ramp_step = RAMP_PERCENT_PER_SEC * dt_sec;

	while (true) {
		/* Sample all command and safety sensors once per control tick. */
		const int up_raw = read_adc_raw(PRESSURE_UP_ADC_UNIT, PRESSURE_UP_ADC_CH);
		const int down_raw = read_adc_raw(PRESSURE_DOWN_ADC_UNIT, PRESSURE_DOWN_ADC_CH);
		const int pos_raw = read_adc_raw(POSITION_ADC_UNIT, POSITION_ADC_CH);
		const int motor_cs_raw = read_adc_raw(MOTOR_CS_ADC_UNIT, MOTOR_CS_ADC_CH);

		/* Convert raw pressure values into comparable duty commands. */
		const float up_cmd = pressure_raw_to_percent(up_raw);
		const float down_cmd = pressure_raw_to_percent(down_raw);

		/* Begin with neutral command and enable direction only after arbitration. */
		bool move_up = false;
		bool move_down = false;
		float target_duty = 0.0f;

		/* Allow only one direction when opposite pad is near idle. */
		if (up_cmd > down_cmd &&
			up_cmd > MIN_CMD_DUTY_PERCENT &&
			down_raw <= OTHER_PAD_IDLE_MAX_RAW) {
			move_up = true;
			target_duty = up_cmd;
		} else if (down_cmd > up_cmd &&
				   down_cmd > MIN_CMD_DUTY_PERCENT &&
				   up_raw <= OTHER_PAD_IDLE_MAX_RAW) {
			move_down = true;
			target_duty = down_cmd;
		} else {
			/* Stay neutral when commands conflict or no valid request exists. */
			target_duty = 0.0f;
		}

		/* Block upward motion when the upper position threshold is reached. */
		if (move_up && pos_raw >= g_limits.upper_limit_raw) {
			move_up = false;
			target_duty = 0.0f;
			ESP_LOGW(TAG, "Upper limit reached, stopping. pos=%d limit=%d", pos_raw, g_limits.upper_limit_raw);
		}
		/* Block downward motion when the lower position threshold is reached. */
		if (move_down && pos_raw <= g_limits.lower_limit_raw) {
			move_down = false;
			target_duty = 0.0f;
			ESP_LOGW(TAG, "Lower limit reached, stopping. pos=%d limit=%d", pos_raw, g_limits.lower_limit_raw);
		}

		/* Immediately force neutral command when overcurrent is detected. */
		if (motor_cs_raw >= MOTOR_CS_OVERCURRENT_RAW) {
			move_up = false;
			move_down = false;
			target_duty = 0.0f;
			ESP_LOGE(TAG, "Overcurrent detected, stopping. cs=%d threshold=%d", motor_cs_raw, MOTOR_CS_OVERCURRENT_RAW);
		}

		/* Ramp duty toward target for smooth acceleration and deceleration. */
		s_current_duty_percent = ramp_toward(s_current_duty_percent, target_duty, max_ramp_step);
		s_current_duty_percent = clampf(s_current_duty_percent, 0.0f, MAX_DUTY_PERCENT);

		/* Apply direction and duty command, or force a full stop when neutral. */
		if (move_up) {
			ESP_ERROR_CHECK(motor_set_direction_up());
			ESP_ERROR_CHECK(motor_pwm_set_percent(s_current_duty_percent));
		} else if (move_down) {
			ESP_ERROR_CHECK(motor_set_direction_down());
			ESP_ERROR_CHECK(motor_pwm_set_percent(s_current_duty_percent));
		} else {
			ESP_ERROR_CHECK(motor_stop());
			s_current_duty_percent = 0.0f;
		}

		/* Emit one status line per cycle for tuning and troubleshooting. */
		ESP_LOGI(TAG,
				 "up=%4d down=%4d pos=%4d cs=%4d duty=%.1f%% lim=[%d,%d]",
				 up_raw,
				 down_raw,
				 pos_raw,
				 motor_cs_raw,
				 s_current_duty_percent,
				 g_limits.lower_limit_raw,
				 g_limits.upper_limit_raw);

		/* Maintain fixed loop cadence for predictable control behavior. */
		usleep(CONTROL_PERIOD_MS * 1000U);
	}
}
