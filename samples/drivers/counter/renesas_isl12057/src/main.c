/*
 * Copyright (c) 2019-2020 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/counter.h>
#include <sys/printk.h>
// #include <drivers/rtc/maxim_ds3231.h>

// /* Format times as: YYYY-MM-DD HH:MM:SS DOW DOY */
// static const char *format_time(time_t time,
// 			       long nsec)
// {
// 	static char buf[64];
// 	char *bp = buf;
// 	char *const bpe = bp + sizeof(buf);
// 	struct tm tv;
// 	struct tm *tp = gmtime_r(&time, &tv);

// 	bp += strftime(bp, bpe - bp, "%Y-%m-%d %H:%M:%S", tp);
// 	if (nsec >= 0) {
// 		bp += snprintf(bp, bpe - bp, ".%09lu", nsec);
// 	}
// 	bp += strftime(bp, bpe - bp, " %a %j", tp);
// 	return buf;
// }

void main(void)
{
	const struct device *isl12057;
	const char *const dev_id = DT_LABEL(DT_INST(0, renesas_isl12057));

	isl12057 = device_get_binding(dev_id);
	if (!isl12057) {
		printk("No device %s available\n", dev_id);
		return;
	}

	uint32_t time = 0;
	counter_get_value(isl12057, &time);

	k_sleep(K_FOREVER);
}
