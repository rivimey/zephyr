/*
 * Copyright (c) 2020 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ADC driver for the ADS130E08 ADCs.
 */

#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>

LOG_MODULE_REGISTER(adc_ads130e08, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

struct ads13xxxx_config {
	struct spi_dt_spec bus;
	uint8_t channels;
	uint8_t resolution;
	enum { GAIN_1, GAIN_2, GAIN_8 } gain;
};

struct ads13xxxx_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	uint8_t differential;
	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_ADS13XXXX_ACQ_THREAD_STACK_SIZE);
};

enum adc13xxx_spi_commands {
    WAKEUP = 0x2,   // wake from standby
    STANDBY = 0x4,  // enter standby
    RESET = 0x6,    // reset device
    START = 0x8,    // start or restart conversions (see also pin 'START')
    STOP = 0xA,     // stop conversions

    RDATAC = 0x10,   // read continuous, default at startup, and RREG ignored.
    SDATAC = 0x11,   // stop read continuous.
    RDATA = 0x12,    // read by command.
    RREG_BASE = 0x20, // low 5 bits = reg no & second byte = num regs to read - 1
    WREG_BASE = 0x40, // low 5 bits = reg no & second byte
};

enum adc13xxx_reg_offsets {
    ID = 0x0,

    CONFIG1 = 0x1,
    CONFIG2 = 0x2,
    CONFIG3 = 0x3,
    FAULT = 0x4,

    CH1SET = 0x5,
    CH2SET = 0x6,
    CH3SET = 0x7,
    CH4SET = 0x8,
    CH5SET = 0x9,
    CH6SET = 0xa,
    CH7SET = 0xb,
    CH8SET = 0xc,

    // addresses 0xd to 0x11 must be written as 0.

    FAULT_STATP = 0x12,
    FAULT_STATN = 0x13,
    GPIO = 0x14,
};

// SPI read returns 24 status bits of form:
// (1100 + FAULT_STATP[8] + FAULT_STATN[8] + GPIO[7:4]),
// followed by 16 bit channel data for each enabled channel.
#define STATUS_LEN   		(24U)
#define STATUS_MASK  		(0xffffffU)
#define STATUS_MAGIC(w)		(((w) >> 20) & 0xfU)
#define STATUS_F_STATP(w)	(((w) >> 12) & 0xffU)
#define STATUS_F_STATN(w)	(((w) >> 4) & 0xffU)
#define STATUS_GPIO(w)		((w) & 0xfU)

#pragma pack(push,1)
struct ads13xxxx_read_packet {
    unsigned magic:4;
    unsigned fault_statp:8;
    unsigned fault_statn:8;
    unsigned gpio:4;
    uint16_t samples[8];
};

#if sizeof(struct ads13xxxx_read_packet) != 19
#warning ads13xxxx read structure must be Packed.
#endif
#pragma pack(pop)

/**
 * Read a register of the ADS13xxxx device into a buffer.
 *
 * The n_reg value is one less than the number of bytes returned, as it
 * excludes the named register.
 *
 * @param dev struct ads13xxxx device control block
 * @param reg register number in the device to read (from...)
 * @param n_reg number of additional registers to read, after first
 * @param data where to put the data read in
 * @return errno
 */
static int ads13xxxx_spi_read_reg(
    	const struct device *dev,
	const enum adc13xxx_reg_offsets reg,
	uint32_t n_reg,
	uint8_t *data)
{
    const struct ads13xxxx_config *cfg = dev->config;

    _ASSERT(reg >= 0 && reg < 0x14);
    _ASSERT(n_reg <= 0x14);

    uint8_t cmd[3] = {
	RREG_BASE + reg,
	n_reg
    };
    const struct spi_buf tx_buf = {
	.buf = cmd,
	.len = ARRAY_SIZE(cmd),
    };
    const struct spi_buf_set tx = {
	.buffers = &tx_buf,
	.count = 1,
    };
    const struct spi_buf rx_buf = {
	.buf = data,
	.len = n_reg+1,
    };
    const struct spi_buf_set rx = {
	.buffers = &rx_buf,
	.count = 1,
    };

    return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int ads13xxxx_spi_write(const struct device *dev, uint32_t addr,
			   uint8_t *data, uint32_t len)
{
    const struct w5500_config *cfg = dev->config;
    int ret;
    uint8_t cmd[3] = {
	addr >> 8,
	addr,
	W5500_SPI_WRITE_CONTROL(addr),
    };
    const struct spi_buf tx_buf[2] = {
	{
	    .buf = cmd,
	    .len = ARRAY_SIZE(cmd),
	},
	{
	    .buf = data,
	    .len = len,
	},
    };
    const struct spi_buf_set tx = {
	.buffers = tx_buf,
	.count = ARRAY_SIZE(tx_buf),
    };

    ret = spi_write_dt(&cfg->spi, &tx);

    return ret;
}


static int ads13xxxx_init(const struct device *dev)
{
    const struct ads13xxxx_config *config = dev->config;
    struct ads13xxxx_data *data = dev->data;

    data->dev = dev;

    k_sem_init(&data->sem, 0, 1);

    if (!spi_is_ready(&config->bus)) {
	LOG_ERR("SPI bus is not ready");
	return -ENODEV;
    }

    // Read
    // Write CONFIG1 bit with CLK_EN=0 to disable int clock output.
    spi_read(dev,  )
    //

    k_thread_create(&data->thread,
		    data->stack,
		    CONFIG_ADC_ADS13XXXX_ACQ_THREAD_STACK_SIZE,
		    (k_thread_entry_t)ads13xxxx_acquisition_thread,
		    data,
		    NULL,
		    NULL,
		    CONFIG_ADC_ADS13XXXX_ACQUISITION_THREAD_PRIO,
		    0,
		    K_NO_WAIT);

    adc_context_unlock_unconditionally(&data->ctx);
    return 0;
}

static int ads13xxxx_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct ads13xxxx_config *config = dev->config;
	struct ads13xxxx_data *data = dev->data;

	//if (channel_cfg->gain != ADC_GAIN_1) {
	//	LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
	//	return -ENOTSUP;
	//}

	//if (channel_cfg->reference != ADC_REF_EXTERNAL0) {
	//	LOG_ERR("unsupported channel reference '%d'", channel_cfg->reference);
	//	return -ENOTSUP;
	//}

	//if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
	//	LOG_ERR("unsupported acquisition_time '%d'", channel_cfg->acquisition_time);
	//	return -ENOTSUP;
	//}

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	//WRITE_BIT(data->differential, channel_cfg->channel_id, channel_cfg->differential);

	// Set ISR for DRDY signal
	// set START = 1

	return 0;
}

static int ads13xxxx_validate_buffer_size(const struct device *dev,
					const struct adc_sequence *sequence)
{
	const struct ads13xxxx_config *config = dev->config;
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

static int ads13xxxx_start_read(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct ads13xxxx_config *config = dev->config;
	struct ads13xxxx_data *data = dev->data;
	int err;

	if (sequence->resolution != config->resolution) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x", sequence->channels);
		return -ENOTSUP;
	}

	err = ads13xxxx_validate_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int ads13xxxx_read_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct ads13xxxx_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = ads13xxxx_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int ads13xxxx_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	return ads13xxxx_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads13xxxx_data *data = CONTAINER_OF(ctx, struct ads13xxxx_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct ads13xxxx_data *data = CONTAINER_OF(ctx, struct ads13xxxx_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int ads13xxxx_read_channel(const struct device *dev, uint8_t channel,
				uint16_t *result)
{
	const struct ads13xxxx_config *config = dev->config;
	struct ads13xxxx_data *data = dev->data;
	uint8_t tx_bytes[2];
	uint8_t rx_bytes[2];
	int err;
	const struct spi_buf tx_buf[2] = {
		{
			.buf = tx_bytes,
			.len = sizeof(tx_bytes)
		},
		{
			.buf = NULL,
			.len = 1
		}
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1
		},
		{
			.buf = rx_bytes,
			.len = sizeof(rx_bytes)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf)
	};

	/*
	 * Configuration bits consists of: 5 dummy bits + start bit +
	 * SGL/#DIFF bit + D2 + D1 + D0 + 6 dummy bits
	 */
	tx_bytes[0] = BIT(2) | channel >> 2;
	tx_bytes[1] = channel << 6;

	if ((data->differential & BIT(channel)) == 0) {
		tx_bytes[0] |= BIT(1);
	}

	err = spi_transceive_dt(&config->bus, &tx, &rx);
	if (err) {
		return err;
	}

	*result = sys_get_be16(rx_bytes);
	*result &= BIT_MASK(config->resolution);

	return 0;
}

static void ads13xxxx_acquisition_thread(struct ads13xxxx_data *data)
{
	uint16_t result = 0;
	uint8_t channel;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		while (data->channels) {
			channel = find_lsb_set(data->channels) - 1;

			LOG_DBG("reading channel %d", channel);

			err = ads13xxxx_read_channel(data->dev, channel, &result);
			if (err) {
				LOG_ERR("failed to read channel %d (err %d)", channel, err);
				adc_context_complete(&data->ctx, err);
				break;
			}

			LOG_DBG("read channel %d, result = %d", channel, result);

			*data->buffer++ = result;
			WRITE_BIT(data->channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}

static const struct adc_driver_api ads13xxxx_adc_api = {
	.channel_setup = ads13xxxx_channel_setup,
	.read = ads13xxxx_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads13xxxx_read_async,
#endif
};

#define INST_DT_ADS13xxxx(inst, t)  DT_INST(inst, ti_ads##t)

#define ADS13xxxx_DEVICE(t, n, ch, res) \
	static struct ads13xxxx_data ti_ads##t##_data_##n = { \
		ADC_CONTEXT_INIT_TIMER(ti_ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_LOCK(ti_ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_SYNC(ti_ads##t##_data_##n, ctx), \
	}; \
    \
	static const struct ads13xxxx_config ti_ads##t##_config_##n = { \
		.bus = SPI_DT_SPEC_GET(INST_DT_ADS130E08(n, t), \
					 SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
					 SPI_WORD_SET(8), 0), \
		.channels = ch, \
		.resolution = res, \
	}; \
    \
	DEVICE_DT_DEFINE(INST_DT_ADS13xxxx(n, t), \
			 &ads13xxxx_init, NULL, \
			 &ti_ads##t##_data_##n, \
			 &ti_ads##t##_config_##n, \
             POST_KERNEL, \
			 CONFIG_ADC_INIT_PRIORITY, \
			 &ads13xxxx_adc_api)

/* ADS130E08: 8 channels, 16bit */
#define ADS130E08_DEVICE(n) ADS13xxxx_DEVICE(130e08, n, 8, 16)
#define CALL_WITH_ARG(arg, expr) expr(arg);
#define INST_DT_ADS130E08_FOREACH(t, inst_expr)	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(ti_ads##t), CALL_WITH_ARG, inst_expr)

INST_DT_ADS130E08_FOREACH(130e08, ADS130E08_DEVICE);


#if 0

// Flag to tell us if we are reading ADC data for the first time
bool firstRead = true;
// Location where DMA will store ADC data in memory, length defined elsewhere
signed long adcData;

// Interrupt the MCU each time DRDY asserts when collecting data
void DRDYinterupt(void)
{
    // Clear the ADC's 2-deep FIFO on the first read.
    if (firstRead) {
        for (i = 0; i < numFrameWords; i++) {
            SPI.write(spiDummyWord + i);
        }
        for (i = 0; i < numFrameWords; i++) {
            SPI.read();
        }
        // Clear the flag
        firstRead = false;
        // Let the DMA start sending ADC data to memory
        DMA.enable();
    }

    // Send the dummy data to the ADC to get the ADC data.
    for (i = 0; i < numFrameWords; i++) {
        SPI.write(spiDummyWord + i);
    }
}

/*
 * adcRegisterWrite
 * @short
 *     function that writes one ADC register at a time. Blocks return until SPI
 *     is idle. Returns false if the word length is wrong.
 *
 * @param
 *     addrMask: 16-bit register address mask
 *     data: data to write
 *     adcWordLength: word length which ADC expects. Either 16, 24 or 32.
 *
 * @return
 *     true if word length was valid false if not
*/
bool adcRegisterWrite(unsigned short addrMask, unsigned short data,
                      unsigned char adcWordLength)
{
    unsigned char shiftValue;       // Stores the amount of bit shift based on
                                    // ADC word length.
    if (adcWordLength == 16) {
        shiftValue = 0;             // If length is 16, no shift.
    }
    else if (adcWordLength == 24) {
        shiftValue = 8;             // If length is 24, shift left by 8.
    }
    else if (adcWordLength == 32) {
        shiftValue = 16;            // If length is 32, shift left by 16.
    }
    else {
        return false;               // If not, invalid length.
    }

    // Write address and opcode Shift to accommodate ADC word length.
    SPI.write((WREG_OPCODE | addrMask) << shiftValue);
    // Write register data.
    SPI.write(data << shiftValue);
    // Wait for data to complete sending.
    while (SPI.isBusy());

    return true;
}

/* main routine */
main(){
    enableSupplies();
    GPIO.inputEnable('input');              // Enable GPIO connected to DRDY
    clkout.enable(8192000);                 // Enable 8.192 MHz clock to CLKIN

    SPI.enable();                           // Enable SPI port
    SPI.wordLengthSet(24);                  // ADC default word length is 24 bits
    SPI.configCS(STAY_ASSERTED);            // Configure CS to remain asserted until frame is complete
    while(!GPIO.read()){}                   // Wait for DRDY to go high indicating it is ok to talk to ADC

    adcRegisterWrite(CLOCK_ADDR,            // Write CLOCK register
                     ALL_CH_DISABLE_MASK |  // Turn off all channels so short frames can be written during config
                     OSR_1024_MASK |
                     PWR_HR_MASK, 24);      // Re-write defaults for other bits in CLOCK register
    adcRegisterWrite(MODE_ADDR,             // Write MODE register
                     RESET_MASK |           // Clear the RESET flag
                     DRDY_FMT_PULSE_MASK |  // Make DRDY active low pulse
                     WLENGTH_24_MASK |      // Re-write defaults for other bits in MODE register
                     SPI_TIMEOUT_MASK, 24;
    adcRegisterWrite(GAIN1_ADDR,            // Write GAIN1 register
                     PGAGAIN3_32_MASK |     // Set channels 1 and 3 PGA gain to 32 in this example
                     PGAGAIN1_32_MASK, 24); // Leave channels 0 and 2 at default gain of 1
    adcRegisterWrite(THRSHLD_LSB_ADDR,      // Write THRSHLD_LSB register
                     0x09, 24);             // Set DCBLOCK filter to have a corner frequency of 622 mHz

    DMA.triggerSet(SPI);                    // Configure DMA to trigger when data comes in on the MCU SPI port
    DMA.txAddrSet(SPI.rxAddr());            // Set the DMA to take from the incoming SPI port
    DMA.rxAddrSet(&adcData);                // Set the DMA to send ADC data to a predefined memory location

    adcRegisterWrite(MODE_ADDR,             // Write MODE register
                     WLENGTH_32_SIGN_EXTEND_MASK | // Make ADC word size 32 bits to accommodate DMA
                     DRDY_FMT_PULSE_MASK |  // Re-write other set bits in MODE register
                     SPI_TIMEOUT_MASK, 24);

    SPI.wordLengthSet(32);                  // Set SPI word size to 32 bits to accomodate DMA
    adcRegisterWrite(CLOCK_ADDR,            // Write CLOCK register
                     ALL_CH_ENABLE_MASK |   // Turn on all ADC channels
                     OSR_1024_MASK |
                     PWR_HR_MASK, 32);      // Re-write defaults for other bits in CLOCK register
    GPIO.interuptEnable();                  // Enable DRDY interrupt and begin streaming data
}

#endif
