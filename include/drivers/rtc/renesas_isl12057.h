/*
 * Copyright (c) 2021 Matija Tudan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTC_ISL12057_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTC_ISL12057_H_

#include <time.h>

#include <drivers/counter.h>
#include <kernel.h>
#include <zephyr/types.h>
#include <sys/notify.h>

#define ISL12057_CTRL1 0x00
#define ISL12057_CTRL1_EXT_TEST BIT(7)
#define ISL12057_CTRL1_STOP BIT(5)
#define ISL12057_CTRL1_SR BIT(4)
#define ISL12057_CTRL1_CIE BIT(2)
#define ISL12057_CTRL1_12_24 BIT(1)
#define ISL12057_CTRL1_CAP_SEL BIT(0)

#define ISL12057_CTRL2 0x01
#define ISL12057_CTRL2_AIE BIT(7)
#define ISL12057_CTRL2_AF BIT(6)
#define ISL12057_CTRL2_MI BIT(5)
#define ISL12057_CTRL2_HMI BIT(4)
#define ISL12057_CTRL2_TF BIT(3)

/* CLKOUT frequency selection */
#define ISL12057_CTRL2_COF_32K 0x0
#define ISL12057_CTRL2_COF_16K 0x1
#define ISL12057_CTRL2_COF_8K 0x2
#define ISL12057_CTRL2_COF_4K 0x3
#define ISL12057_CTRL2_COF_2K 0x4
#define ISL12057_CTRL2_COF_1K 0x5
#define ISL12057_CTRL2_COF_1 0x6
#define ISL12057_CTRL2_COF_LOW 0x7

#define ISL12057_OFFSET 0x02
#define ISL12057_RAM_BYTE 0x03
#define ISL12057_TENS_SHIFT 4

/* Time related */
#define ISL12057_SECONDS 0x04
#define ISL12057_SECONDS_OS BIT(7)

#define ISL12057_MINUTES 0x05

#define ISL12057_HOURS 0x06
#define ISL12057_HOURS_AM_PM BIT(5)

#define ISL12057_DAYS 0x07

#define ISL12057_WEEKDAYS 0x08
#define ISL12057_WEEKDAYS_SUN 0x0
#define ISL12057_WEEKDAYS_MON 0x1
#define ISL12057_WEEKDAYS_TUE 0x2
#define ISL12057_WEEKDAYS_WED 0x3
#define ISL12057_WEEKDAYS_THU 0x4
#define ISL12057_WEEKDAYS_FRI 0x5
#define ISL12057_WEEKDAYS_SAT 0x6

#define ISL12057_MONTHS 0x09
#define ISL12057_MONTHS_JAN 0x1
#define ISL12057_MONTHS_FEB 0x2
#define ISL12057_MONTHS_MAR 0x3
#define ISL12057_MONTHS_APR 0x4
#define ISL12057_MONTHS_MAY 0x5
#define ISL12057_MONTHS_JUN 0x6
#define ISL12057_MONTHS_JUL 0x7
#define ISL12057_MONTHS_AUG 0x8
#define ISL12057_MONTHS_SEP 0x9
#define ISL12057_MONTHS_OCT 0x10
#define ISL12057_MONTHS_NOV 0x11
#define ISL12057_MONTHS_DEC 0x12

#define ISL12057_YEARS 0x0a

/* Alarm related */
#define ISL12057_SECOND_ALARM 0x0b
#define ISL12057_SECOND_ALARM_EN BIT(7)

#define ISL12057_MINUTE_ALARM 0x0c
#define ISL12057_MINUTE_ALARM_EN BIT(7)

#define ISL12057_HOUR_ALARM 0x0d
#define ISL12057_HOUR_ALARM_EN BIT(7)
#define ISL12057_HOUR_ALARM_AM_PM BIT(5)

#define ISL12057_DAY_ALARM 0x0e
#define ISL12057_DAY_ALARM_EN BIT_MASK(7)

#define ISL12057_WEEKDAY_ALARM 0x0f
#define ISL12057_WEEKDAY_ALARM_EN BIT(7)

/* Timer registers */
#define ISL12057_TIMER_VALUE 0x10
#define ISL12057_TIMER_MODE 0x11
#define ISL12057_TIMER_MODE_FREQ_MASK BIT(4) | BIT(3)
#define ISL12057_TIMER_MODE_FREQ_SHIFT 3
#define ISL12057_TIMER_MODE_FREQ_4K 0x0
#define ISL12057_TIMER_MODE_FREQ_64 0x1
#define ISL12057_TIMER_MODE_FREQ_1 0x2
#define ISL12057_TIMER_MODE_FREQ_1_60 0x3
#define ISL12057_TIMER_MODE_EN BIT(2)
#define ISL12057_TIMER_MODE_INT_EN BIT(1)
#define ISL12057_TIMER_MODE_INT_TI_TP BIT(0)

struct register_map {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dow;
	uint8_t dom;
	uint8_t moncen;
	uint8_t year;
};

struct isl12057_time {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	bool am_pm;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
};

struct isl12057_config {
	/* Common structure first because generic API expects this here */
	struct counter_config_info generic;
	const char *bus_name;
	uint16_t addr;
};

struct isl12057_data {
	const struct device *i2c;
	struct register_map registers;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTC_ISL12057_H_ */
