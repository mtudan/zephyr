/*
 * Copyright (c) 2021 Matija Tudan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_isl12057

#include <kernel.h>
#include <device.h>
#include <drivers/rtc/renesas_isl12057.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <sys/timeutil.h>
#include <sys/util.h>

LOG_MODULE_REGISTER(ISL12057, CONFIG_COUNTER_LOG_LEVEL);

static int isl12057_init(const struct device *dev)
{
	const struct isl12057_config *cfg = dev->config;
	struct isl12057_data *data = dev->data;
	int ret;

	data->i2c = device_get_binding(cfg->bus_name);
	if (!data->i2c) {
		LOG_ERR("Could not get pointer to %s device", cfg->bus_name);
		return -EINVAL;
	}

	// uint8_t tmp;
	// i2c_reg_read_byte(data->i2c, cfg->addr, 0x00, &tmp);
	// printk("isl12057_init sec = %u\n", tmp);
	// i2c_reg_read_byte(data->i2c, cfg->addr, 0x01, &tmp);
	// printk("isl12057_init min = %u\n", tmp);
	// i2c_reg_read_byte(data->i2c, cfg->addr, 0x02, &tmp);
	// printk("isl12057_init hour = %u\n", tmp);
	// i2c_reg_read_byte(data->i2c, cfg->addr, 0x04, &tmp);
	// printk("isl12057_init date = %u\n", tmp);
	// i2c_reg_read_byte(data->i2c, cfg->addr, 0x05, &tmp);
	// printk("isl12057_init month = %u\n", tmp);
	// i2c_reg_read_byte(data->i2c, cfg->addr, 0x06, &tmp);
	// printk("isl12057_init year = %u\n", tmp);

	LOG_DBG("Init complete");

	return 0;
}

static int isl12057_start(const struct device *dev)
{
	return -EALREADY;
}

static int isl12057_stop(const struct device *dev)
{
	return -EALREADY;
}

uint8_t decode_bcd(struct register_map *rm)
{
	rm->sec = rm->sec - 6 * (rm->sec >> 4);
	rm->min = rm->min - 6 * (rm->min >> 4);
	rm->hour = rm->hour - 6 * (rm->hour >> 4);
	rm->dow = rm->dow - 6 * (rm->dow >> 4);
	rm->dom = rm->dom - 6 * (rm->dom >> 4);
	rm->moncen = rm->moncen - 6 * (rm->moncen >> 4);
	rm->year = rm->year - 6 * (rm->year >> 4);
	printk("rm->sec = %u\n", rm->sec);
	printk("rm->min = %u\n", rm->min);
	printk("rm->hour = %u\n", rm->hour);
	printk("rm->dow = %u\n", rm->dow);
	printk("rm->dom = %u\n", rm->dom);
	printk("rm->moncen = %u\n", rm->moncen);
	printk("rm->year = %u\n", rm->year);
}

static int isl12057_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct isl12057_config *cfg = dev->config;
	struct isl12057_data *data = dev->data;
	uint8_t addr = 0;
	time_t time = 0;

	// k_sem_take(&data->lock, K_FOREVER);

	int rc = i2c_write_read(data->i2c, cfg->addr,
					       &addr, sizeof(addr),
					       &data->registers, 7);

	printk("sec = %u\n", data->registers.sec);
	printk("min = %u\n", data->registers.min);
	printk("hour = %u\n", data->registers.hour);
	printk("day of week = %u\n", data->registers.dow);
	printk("day of month = %u\n", data->registers.dom);
	printk("month = %u\n", data->registers.moncen);
	printk("year = %u\n", data->registers.year);

	// TODO: remove, comperes if decode_bcd() will do a good job
	printk("bcd_to_dec sec = %u\n", data->registers.sec - 6 * (data->registers.sec >> 4));
	printk("bcd_to_dec min = %u\n", data->registers.min - 6 * (data->registers.min >> 4));
	printk("bcd_to_dec hour = %u\n", data->registers.hour - 6 * (data->registers.hour >> 4));
	printk("bcd_to_dec day of week = %u\n", data->registers.dow - 6 * (data->registers.dow >> 4));
	printk("bcd_to_dec day of month = %u\n", data->registers.dom - 6 * (data->registers.dom >> 4));
	printk("bcd_to_dec month = %u\n", data->registers.moncen - 6 * (data->registers.moncen >> 4));
	printk("bcd_to_dec year = %u\n", data->registers.year - 6 * (data->registers.year >> 4));

	printk("-------------------------------------\n");
	decode_bcd(&data->registers);
	printk("sec = %u\n", data->registers.sec);
	printk("min = %u\n", data->registers.min);
	printk("hour = %u\n", data->registers.hour);
	printk("day of week = %u\n", data->registers.dow);
	printk("day of month = %u\n", data->registers.dom);
	printk("month = %u\n", data->registers.moncen);
	printk("year = %u\n", data->registers.year);
	printk("-------------------------------------\n");

	if (rc >= 0) {
		// *time = decode_rtc(data);
	}

	// k_sem_give(&data->lock);

	if (rc >= 0) {
		// *ticks = time;
	}

	return rc;
}

static int isl12057_set_alarm(const struct device *dev, uint8_t chan_id,
						      const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan_id);

	uint8_t ticks = (uint8_t)alarm_cfg->ticks;

	// Get the data pointer
	struct isl12057_data *data = dev->data;

	// // Ret val for error checking
	// int ret;

	// // Clear any flags in CTRL2
	// uint8_t reg = 0;
	// uint8_t mask = ISL12057_CTRL2_TF;

	// ret = i2c_reg_update_byte(data->i2c, DT_REG_ADDR(DT_DRV_INST(0)),
	// 													ISL12057_CTRL2, mask, reg);
	// if (ret)
	// {
	// 	LOG_ERR("Unable to set RTC alarm. (err %i)", ret);
	// 	return ret;
	// }

	// // Write the tick count. Ticks are 1 sec
	// ret = i2c_reg_write_byte(data->i2c, DT_REG_ADDR(DT_DRV_INST(0)),
	// 												 ISL12057_TIMER_VALUE, ticks);
	// if (ret)
	// {
	// 	LOG_ERR("Unable to set RTC timer value. (err %i)", ret);
	// 	return ret;
	// }

	// // Set to 1 second mode
	// reg = (ISL12057_TIMER_MODE_FREQ_1 << ISL12057_TIMER_MODE_FREQ_SHIFT) | ISL12057_TIMER_MODE_EN | ISL12057_TIMER_MODE_INT_EN;
	// mask = ISL12057_TIMER_MODE_FREQ_MASK | ISL12057_TIMER_MODE_EN | ISL12057_TIMER_MODE_INT_EN;

	// LOG_INF("mode 0x%x", reg);

	// // Write back the updated register value
	// ret = i2c_reg_update_byte(data->i2c, DT_REG_ADDR(DT_DRV_INST(0)),
	// 													ISL12057_TIMER_MODE, mask, reg);
	// if (ret)
	// {
	// 	LOG_ERR("Unable to set RTC alarm. (err %i)", ret);
	// 	return ret;
	// }

	return 0;
}

static int isl12057_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	struct isl12057_data *data = dev->data;

	// // Ret val for error checking
	// int ret;

	// // Clear any flags in CTRL2
	// uint8_t reg = 0;
	// uint8_t mask = ISL12057_CTRL2_TF;

	// ret = i2c_reg_update_byte(data->i2c, DT_REG_ADDR(DT_DRV_INST(0)),
	// 													ISL12057_CTRL2, mask, reg);
	// if (ret)
	// {
	// 	LOG_ERR("Unable to set RTC alarm. (err %i)", ret);
	// 	return ret;
	// }

	// // Turn off all itnerrupts/timer mode
	// reg = 0;
	// mask = ISL12057_TIMER_MODE_EN | ISL12057_TIMER_MODE_INT_EN | ISL12057_TIMER_MODE_INT_TI_TP;

	// LOG_INF("mode 0x%x", reg);

	// // Write back the updated register value
	// ret = i2c_reg_update_byte(data->i2c, DT_REG_ADDR(DT_DRV_INST(0)),
	// 													ISL12057_TIMER_MODE, mask, reg);
	// if (ret)
	// {
	// 	LOG_ERR("Unable to cancel RTC alarm. (err %i)", ret);
	// 	return ret;
	// }

	return 0;
}

static int isl12057_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	return 0;
}

static uint32_t isl12057_get_pending_int(const struct device *dev)
{
	struct isl12057_data *data = dev->data;

	// // Start with 0
	// uint8_t reg = 0;

	// // Write back the updated register value
	// int ret = i2c_reg_read_byte(data->i2c, DT_REG_ADDR(DT_DRV_INST(0)),
	// 														ISL12057_CTRL2, &reg);
	// if (ret)
	// {
	// 	LOG_ERR("Unable to get RTC CTRL2 reg. (err %i)", ret);
	// 	return ret;
	// }

	// // Return 1 if interrupt. 0 if no flag.
	// return (reg & ISL12057_CTRL2_TF) ? 1U : 0U;

	return 0;
}

static uint32_t isl12057_get_top_value(const struct device *dev)
{
	return 0;
}

static uint32_t isl12057_get_max_relative_alarm(const struct device *dev)
{
	return 0;
}

static const struct counter_driver_api isl12057_driver_api = {
	.start = isl12057_start,
	.stop = isl12057_stop,
	.get_value = isl12057_get_value,
	.set_alarm = isl12057_set_alarm,
	.cancel_alarm = isl12057_cancel_alarm,
	.set_top_value = isl12057_set_top_value,
	.get_pending_int = isl12057_get_pending_int,
	.get_top_value = isl12057_get_top_value,
	.get_max_relative_alarm = isl12057_get_max_relative_alarm,
};

static const struct isl12057_config isl12057_driver_config = {
	.generic = {
		.max_top_value = UINT8_MAX,
		.freq = 1,
		.flags = COUNTER_CONFIG_INFO_COUNT_UP,
		.channels = 2,
	},
	.bus_name = DT_INST_BUS_LABEL(0),
	.addr = DT_INST_REG_ADDR(0),
};

static struct isl12057_data isl12057_driver_data;

#if CONFIG_COUNTER_RENESAS_ISL12057_INIT_PRIORITY <= CONFIG_I2C_INIT_PRIORITY
#error COUNTER_RENESAS_ISL12057_INIT_PRIORITY must be greater than I2C_INIT_PRIORITY
#endif

DEVICE_DT_INST_DEFINE(0, isl12057_init, device_pm_control_nop,
		    &isl12057_driver_data, &isl12057_driver_config,
		    POST_KERNEL, CONFIG_COUNTER_RENESAS_ISL12057_INIT_PRIORITY,
		    &isl12057_driver_api);
