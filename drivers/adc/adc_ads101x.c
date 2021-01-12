/*
 * Copyright (c) 2021 Matija Tudan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ADC driver for the ADS1013/ADS1014/ADS1015 ADCs.
 */

#include <zephyr.h>
#include <kernel.h>
#include <drivers/adc.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util.h>

LOG_MODULE_REGISTER(adc_ads101x, 4);//CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADS101X_RESOLUTION 12U

struct ads101x_config {
	const char *bus_name;
	uint16_t addr;
	uint8_t channels;
};

struct ads101x_data {
	struct adc_context ctx;
	const struct device *dev;
	const struct device *i2c;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	uint8_t differential;
	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack,
			CONFIG_ADC_ADS101X_ACQUISITION_THREAD_STACK_SIZE);
};

static int ads101x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct ads101x_config *config = dev->config;
	struct ads101x_data *data = dev->data;

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_EXTERNAL0) {
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
				uint16_t *result)
{
	const struct ads101x_config *config = dev->config;
	struct ads101x_data *data = dev->data;
	// uint8_t tx_bytes[2];
	// uint8_t rx_bytes[2];
	// int err;
	// const struct spi_buf tx_buf[2] = {
	// 	{
	// 		.buf = tx_bytes,
	// 		.len = sizeof(tx_bytes)
	// 	},
	// 	{
	// 		.buf = NULL,
	// 		.len = 1
	// 	}
	// };
	// const struct spi_buf rx_buf[2] = {
	// 	{
	// 		.buf = NULL,
	// 		.len = 1
	// 	},
	// 	{
	// 		.buf = rx_bytes,
	// 		.len = sizeof(rx_bytes)
	// 	}
	// };
	// const struct spi_buf_set tx = {
	// 	.buffers = tx_buf,
	// 	.count = ARRAY_SIZE(tx_buf)
	// };
	// const struct spi_buf_set rx = {
	// 	.buffers = rx_buf,
	// 	.count = ARRAY_SIZE(rx_buf)
	// };

	// /*
	//  * Configuration bits consists of: 5 dummy bits + start bit +
	//  * SGL/#DIFF bit + D2 + D1 + D0 + 6 dummy bits
	//  */
	// tx_bytes[0] = BIT(2) | channel >> 2;
	// tx_bytes[1] = channel << 6;

	// if ((data->differential & BIT(channel)) == 0) {
	// 	tx_bytes[0] |= BIT(1);
	// }

	// // err = spi_transceive(data->spi_dev, &config->spi_cfg, &tx, &rx);
	// if (err) {
	// 	return err;
	// }

	// *result = sys_get_be16(rx_bytes);
	// *result &= BIT_MASK(ADS101X_RESOLUTION);

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

			err = ads101x_read_channel(data->dev, channel, &result);
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

	printk("printk: ads101x_init\n");
	LOG_INF("LOG_INF: ads101x_init");

	k_sem_init(&data->sem, 0, 1);
	
	data->i2c = device_get_binding(config->bus_name);
	if (!data->i2c) {
		LOG_ERR("Could not find I2C device");
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
};

#define INST_DT_ADS101X(inst, t) DT_INST(inst, ti_ads##t)

#define ADS101X_DEVICE(t, n, ch) \
	static struct ads101x_data ads##t##_data_##n = { \
		ADC_CONTEXT_INIT_TIMER(ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_LOCK(ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_SYNC(ads##t##_data_##n, ctx), \
	}; \
	static const struct ads101x_config ads##t##_config_##n = { \
		.bus_name = DT_BUS_LABEL(INST_DT_ADS101X(n, t)), \
		.addr = DT_REG_ADDR(INST_DT_ADS101X(n, t)), \
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

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_ADS101X_FOREACH(t, inst_expr) \
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(ti_ads##t), \
		     CALL_WITH_ARG, inst_expr)

INST_DT_ADS101X_FOREACH(1014, ADS1014_DEVICE);
INST_DT_ADS101X_FOREACH(1015, ADS1015_DEVICE);
