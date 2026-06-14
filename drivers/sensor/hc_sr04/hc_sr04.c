/*
 * Copyright (c) 2024 Adrien Leravat
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hc_sr04

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(HC_SR04, CONFIG_SENSOR_LOG_LEVEL);

#define SOUND_SPEED_MM_PER_MS (343)
#define PULSE_WIDTH_US (PWM_USEC(10))

struct hcsr04_data {
	const struct device *dev;
	atomic_t echo_high_cycles;
};

struct hcsr04_config {
	const struct pwm_dt_spec pwm_trigger;
	const struct pwm_dt_spec pwm_echo;
};

static void capture_callback(const struct device *dev, uint32_t channel,
				  uint32_t period_cycles, uint32_t pulse_cycles, int status,
				  void *user_data)
{
	const struct device *hcsr04_dev = user_data;
	struct hcsr04_data *data = hcsr04_dev->data;

	atomic_set(&data->echo_high_cycles, pulse_cycles);
}

static int hcsr04_init(const struct device *dev)
{
	const struct hcsr04_config *cfg = dev->config;
	int ret;

	if (!pwm_is_ready_dt(&cfg->pwm_echo)) {
		return -ENODEV;
	}
	if (!pwm_is_ready_dt(&cfg->pwm_trigger)) {
		return -ENODEV;
	}
  
	ret = pwm_set(cfg->pwm_trigger.dev, 
								cfg->pwm_trigger.channel, 
								cfg->pwm_trigger.period, 
								PULSE_WIDTH_US, 
								cfg->pwm_trigger.flags);
	if (ret < 0) {
		return ret;
	}

	ret = pwm_configure_capture(cfg->pwm_echo.dev, cfg->pwm_echo.channel, 
			cfg->pwm_echo.flags | PWM_CAPTURE_TYPE_PULSE |
			PWM_CAPTURE_MODE_CONTINUOUS,
			capture_callback, (void *)dev);
	if (ret != 0) {
		return ret;
	}

	ret = pwm_enable_capture(cfg->pwm_echo.dev, cfg->pwm_echo.channel);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int hcsr04_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int hcsr04_channel_get(const struct device *dev, enum sensor_channel chan,
	struct sensor_value *val)
{
	const struct hcsr04_data *data = dev->data;
	const struct hcsr04_config *config = dev->config;
	uint32_t distance_mm;
	uint64_t usec;

	if (chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}

	pwm_cycles_to_usec(config->pwm_echo.dev, config->pwm_echo.channel, data->echo_high_cycles, &usec);
	distance_mm = usec * SOUND_SPEED_MM_PER_MS / (2 * USEC_PER_MSEC);
	return sensor_value_from_milli(val, distance_mm);
}

static DEVICE_API(sensor, hcsr04_driver_api) = {
	.sample_fetch = hcsr04_sample_fetch,
	.channel_get = hcsr04_channel_get
};


#define HC_SR04_INIT(index)                                                           \
	static struct hcsr04_data hcsr04_data_##index = {                             \
		.dev = DEVICE_DT_INST_GET(index),                                     \
		.echo_high_cycles = ATOMIC_INIT(0),                                   \
	};                                                                            \
	static struct hcsr04_config hcsr04_config_##index = {                         \
		.pwm_echo = PWM_DT_SPEC_INST_GET_BY_IDX(index, 0),                                                    \
		.pwm_trigger = PWM_DT_SPEC_INST_GET_BY_IDX(index, 1),                                                    \
	};                                                                            \
                                                                                      \
	SENSOR_DEVICE_DT_INST_DEFINE(index, &hcsr04_init, NULL, &hcsr04_data_##index, \
				&hcsr04_config_##index, POST_KERNEL,                  \
				CONFIG_SENSOR_INIT_PRIORITY, &hcsr04_driver_api);     \

DT_INST_FOREACH_STATUS_OKAY(HC_SR04_INIT)
