/*
 * Copyright (c) 2021 Matija Tudan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <kernel.h>
#include <drivers/i2c.h>
#include <drivers/adc.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(adc_ads1015, 4);//CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADS101X_RESOLUTION 12U

/* Register addresses */
enum {
	CONVERSION	= 0x00,
	CONFIG		= 0x01,
};

/* Masks */
enum {
	FSTAT_DNR        = 0x01,
};

struct ads101x_config {
	const char *i2c_bus;
	uint16_t i2c_addr;
	uint8_t channels;
};

struct ads101x_data {
	const struct device *dev;
	const struct device *i2c;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	uint8_t differential;
	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack,
			CONFIG_ADC_ADS101X_ACQUISITION_THREAD_STACK_SIZE);
};

static int ads101x_reg_read(const struct device *dev, uint8_t reg,
			      uint16_t *val)
{
	struct ads101x_data *data = dev->data;
	const struct ads101x_config *cfg = dev->config;

	if (i2c_burst_read(data->i2c, cfg->i2c_addr,
			   reg, (uint8_t *) val, 2) < 0) {
		LOG_ERR("I2C read failed");
		return -EIO;
	}

	*val = sys_be16_to_cpu(*val);

	return 0;
}

static int ads101x_reg_write(const struct device *dev, uint8_t reg,
			       uint16_t val)
{
	struct ads101x_data *data = dev->data;
	const struct ads101x_config *cfg = dev->config;
	uint8_t buf[3] = {reg, val >> 8, val & 0xFF};

	return i2c_write(data->i2c, buf, sizeof(buf), cfg->i2c_addr);
}

static int ads101x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct ads101x_config *config = dev->config;
	struct ads101x_data *data = dev->data;

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("unsupported channel reference '%d'",
			channel_cfg->reference);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'",
			channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	WRITE_BIT(data->differential, channel_cfg->channel_id,
		  channel_cfg->differential);

	return 0;
}

static int ads101x_validate_buffer_size(const struct device *dev,
					const struct adc_sequence *sequence)
{
	const struct ads101x_config *config = dev->config;
	uint8_t channels = 0;
	size_t needed;
	uint32_t mask;

	for (mask = BIT(config->channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(uint16_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ads101x_start_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct ads101x_config *config = dev->config;
	struct ads101x_data *data = dev->data;
	int err;

	if (sequence->resolution != ADS101X_RESOLUTION) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x",
			sequence->channels);
		return -ENOTSUP;
	}

	err = ads101x_validate_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int ads101x_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct ads101x_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = ads101x_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int ads101x_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	return ads101x_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads101x_data *data = CONTAINER_OF(ctx, struct ads101x_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct ads101x_data *data = CONTAINER_OF(ctx, struct ads101x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int ads101x_read_channel(const struct device *dev, uint8_t channel,
				uint16_t result)
{
	// const struct ads101x_config *config = dev->config;
	// struct ads101x_data *data = dev->data;
	uint16_t config, config_channel;
	int err;

	//TODO: read I2C reg
	err = ads101x_reg_read(dev, CONFIG, &config);
	if (err) {
		LOG_ERR("Unable to read CONFIG reg");
		return -EIO;
	}
	LOG_DBG("---------> config reg = %#4x", config);

	/* | MUX[2:0] | Input multiplexer configuration | */
	/* |----------|---------------------------------| */
	/* |   100    |   AINP = AIN0 and AINN = GND    | */
	/* |   101    |   AINP = AIN1 and AINN = GND    | */
	/* |   110    |   AINP = AIN2 and AINN = GND    | */
	/* |   111    |   AINP = AIN3 and AINN = GND    | */
	config_channel = channel + 4;

	/* | 15 | 14 13 12 | 11 10 9  |  8   |  7 6 5  |     4     |    3     |    2     |      1 0      | */
	/* |----|----------|----------|------|---------|-----------|----------|----------|---------------| */
	/* | OS | MUX[2:0] | PGA[2:0] | MODE | DR[2:0] | COMP_MODE | COMP_POL | COMP_LAT | COMP_QUE[1:0] | */
	config_channel <<= 12;

	return 0;
}

static void ads101x_acquisition_thread(struct ads101x_data *data)
{
	uint16_t result = 0;
	uint8_t channel;
	int err;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		while (data->channels) {
			channel = find_lsb_set(data->channels) - 1;

			LOG_DBG("reading channel %d", channel);

			err = ads101x_read_channel(data->dev, channel, result);
			if (err) {
				LOG_ERR("failed to read channel %d (err %d)",
					channel, err);
				adc_context_complete(&data->ctx, err);
				break;
			}

			LOG_DBG("read channel %d, result = %d", channel,
				result);

			*data->buffer++ = result;
			WRITE_BIT(data->channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}

static int ads101x_init(const struct device *dev)
{
	const struct ads101x_config *config = dev->config;
	struct ads101x_data *data = dev->data;

	data->dev = dev;
	k_sem_init(&data->sem, 0, 1);

	data->i2c = device_get_binding(config->i2c_bus);
	if (!data->i2c) {
		LOG_ERR("I2C device '%s' not found", config->i2c_bus);
		return -EINVAL;
	}

	k_thread_create(&data->thread, data->stack,
			CONFIG_ADC_ADS101X_ACQUISITION_THREAD_STACK_SIZE,
			(k_thread_entry_t)ads101x_acquisition_thread,
			data, NULL, NULL,
			CONFIG_ADC_ADS101X_ACQUISITION_THREAD_PRIO,
			0, K_NO_WAIT);

	adc_context_unlock_unconditionally(&data->ctx);

	LOG_DBG("Init complete");

	return 0;
}

static const struct adc_driver_api ads101x_adc_api = {
	.channel_setup = ads101x_channel_setup,
	.read = ads101x_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads101x_read_async,
#endif
	.ref_internal = 2048,
};

#define INST_DT_ADS101X(inst, t) DT_INST(inst, ti_ads##t)

#define ADS101X_DEVICE(t, n, ch) \
	static struct ads101x_data ads##t##_data_##n = { \
		ADC_CONTEXT_INIT_TIMER(ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_LOCK(ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_SYNC(ads##t##_data_##n, ctx), \
	}; \
	static const struct ads101x_config ads##t##_config_##n = { \
		.i2c_bus = DT_BUS_LABEL(INST_DT_ADS101X(n, t)), \
		.i2c_addr = DT_REG_ADDR(INST_DT_ADS101X(n, t)), \
		.channels = ch, \
	}; \
	DEVICE_DT_DEFINE(INST_DT_ADS101X(n, t), \
			 &ads101x_init, device_pm_control_nop, \
			 &ads##t##_data_##n, \
			 &ads##t##_config_##n, POST_KERNEL, \
			 CONFIG_ADC_ADS101X_INIT_PRIORITY, \
			 &ads101x_adc_api)

/*
 * ADS1014: 1 channel
 */
#define ADS1014_DEVICE(n) ADS101X_DEVICE(1014, n, 1)

/*
 * ADS1015: 4 channels
 */
#define ADS1015_DEVICE(n) ADS101X_DEVICE(1015, n, 4)

#define CALL_WITH_ARG(arg, expr) expr(arg)

#define INST_DT_ADS101X_FOREACH(t, inst_expr) \
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(ti_ads##t), \
		     CALL_WITH_ARG, inst_expr)

INST_DT_ADS101X_FOREACH(1014, ADS1014_DEVICE);
INST_DT_ADS101X_FOREACH(1015, ADS1015_DEVICE);
