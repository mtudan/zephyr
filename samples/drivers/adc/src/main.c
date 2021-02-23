/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <devicetree.h>
#include <drivers/adc.h>

#define ADS1015 DT_INST(0, ti_ads1015)
#if DT_NODE_HAS_STATUS(ADS1015, okay)
#define ADS1015_LABEL DT_LABEL(ADS1015)
#else
#error Your devicetree has no enabled nodes with compatible "ti,ads1015"
#define ADS1015_LABEL "<none>"
#endif

#define ADC_NUM_CHANNELS	1

/* Common settings supported by most ADCs */
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT

/* Get the numbers of up to two channels */
static uint8_t channel_ids[ADC_NUM_CHANNELS];
// static uint8_t channel_ids[ADC_NUM_CHANNELS] = {
// 	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 0),
// #if ADC_NUM_CHANNELS == 2
// 	DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), 1)
// #endif
// };

static int16_t sample_buffer[ADC_NUM_CHANNELS];

struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	/* channel ID will be overwritten below */
	.channel_id = 0,
	.differential = 0
};

struct adc_sequence sequence = {
	/* individual channels will be added below */
	.channels    = 0,
	.buffer      = sample_buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(sample_buffer),
	.resolution  = ADC_RESOLUTION,
};

void main(void)
{
	int err;
	const struct device *dev_adc = device_get_binding(ADS1015_LABEL);
	if (dev_adc == NULL) {
		printk("No ADC device %s found...\n", dev_adc->name);
		return;
	}
	printk("Found ADC device %s\n", dev_adc->name);

	// const struct device *dev_adc = DEVICE_DT_GET(ADC_NODE);

	if (!device_is_ready(dev_adc)) {
		printk("ADC device not found\n");
		return;
	}

	/*
	 * Configure channels individually prior to sampling
	 */
	for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
		channel_cfg.channel_id = channel_ids[i];

		adc_channel_setup(dev_adc, &channel_cfg);

		sequence.channels |= BIT(channel_ids[i]);
	}

	int32_t adc_vref = adc_ref_internal(dev_adc);

	while (1) {
		printk("ADC reading ...\n");
		/*
		 * Read sequence of channels (fails if not supported by MCU)
		 */
		err = adc_read(dev_adc, &sequence);
		if (err != 0) {
			printk("ADC reading failed with error %d.\n", err);
			return;
		}

		printk("ADC reading:");
		for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
			int32_t raw_value = sample_buffer[i];

			printk(" %d", raw_value);
			if (adc_vref > 0) {
				/*
				 * Convert raw reading to millivolts if driver
				 * supports reading of ADC reference voltage
				 */
				int32_t mv_value = raw_value;

				adc_raw_to_millivolts(adc_vref, ADC_GAIN,
					ADC_RESOLUTION, &mv_value);
				printk(" = %d mV  ", mv_value);
			}
		}
		printk("\n");

		k_sleep(K_MSEC(1000));
		return; // TODO: remove!
	}
}
