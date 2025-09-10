#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>

#define ADXL357_REG_DEVID_AD 0x00
#define ADXL357_REG_DEVID_MST 0x01
#define ADXL357_REG_PARTID 0x02
#define ADXL357_REG_REVID 0x03
#define ADXL357_REG_STATUS 0x04
#define ADXL357_REG_FIFO_ENTRIES 0x05
#define ADXL357_REG_TEMP2 0x06
#define ADXL357_REG_TEMP1 0x07
#define ADXL357_REG_XDATA3 0x08
#define ADXL357_REG_XDATA2 0x09
#define ADXL357_REG_XDATA1 0x0A
#define ADXL357_REG_YDATA3 0x0B
#define ADXL357_REG_YDATA2 0x0C
#define ADXL357_REG_YDATA1 0x0D
#define ADXL357_REG_ZDATA3 0x0E
#define ADXL357_REG_ZDATA2 0x0F
#define ADXL357_REG_ZDATA1 0x10
#define ADXL357_REG_FIFO_DATA 0x11
#define ADXL357_REG_ACT_EN 0x24
#define ADXL357_REG_ACT_THRESH_H 0x25
#define ADXL357_REG_ACT_THRESH_L 0x26
#define ADXL357_REG_ACT_COUNT 0x27
#define ADXL357_REG_POWER_CTL 0x2D
#define ADXL357_REG_RANGE 0x2C

#define ADXL357_READ_CMD 0x01
#define ADXL357_WRITE_CMD 0x00

static const struct device *spi_dev;
static struct spi_config spi_cfg;

static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi2), cs_gpios, 0);

float temp_c = 0.0f;

uint8_t adxl357_read_reg(uint8_t reg)
{
	uint8_t tx_buf[2] = {(reg << 1) | ADXL357_READ_CMD, 0x00};
	uint8_t rx_buf[2] = {0};

	struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 2};
	struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = 2};

	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_spi_buf_set, &rx_spi_buf_set);
	if (ret < 0)
	{
		printk("SPI transceive failed: %d\n", ret);
		return 0;
	}
	/*
	// Add debug print
	printk("Read reg 0x%02X: tx[0]=0x%02X, tx[1]=0x%02X, rx[0]=0x%02X, rx[1]=0x%02X\n",
				 reg, tx_buf[0], tx_buf[1], rx_buf[0], rx_buf[1]);
*/
	return rx_buf[1];
}

void adxl357_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t tx_buf[2] = {(reg << 1) | ADXL357_WRITE_CMD, value};
	uint8_t rx_buf[2] = {0}; // Add receive buffer

	struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 2};
	struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = 2}; // Add this

	struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
	struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1}; // Add this

	// Use spi_transceive instead of spi_write
	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_spi_buf_set, &rx_spi_buf_set);
	if (ret < 0)
	{
		printk("SPI write failed: %d\n", ret);
	}

	// Add debug print
	printk("Write reg 0x%02X: tx[0]=0x%02X, tx[1]=0x%02X, value=0x%02X, ret=%d\n",
				 reg, tx_buf[0], tx_buf[1], value, ret);
}

int32_t adxl357_read_axis(uint8_t reg_high)
{
	uint8_t msb = adxl357_read_reg(reg_high);			// MSB
	uint8_t mid = adxl357_read_reg(reg_high + 1); // MID
	uint8_t lsb = adxl357_read_reg(reg_high + 2); // LSB

	// Combine 20-bit data (sign extend)
	int32_t result = ((msb << 16) | (mid << 8) | lsb) >> 4; // Discard lower 4 bits

	// Sign extend from 20-bit to 32-bit
	if (result & 0x80000)
	{
		result |= 0xFFF00000;
	}

	return result;
}

uint16_t adxl357_read_temp_raw(void)
{
    uint8_t temp2 = adxl357_read_reg(ADXL357_REG_TEMP2) & 0x0F; // 4 MSB
    uint8_t temp1 = adxl357_read_reg(ADXL357_REG_TEMP1);        // 8 LSB
    return ((temp2 << 8) | temp1); // 12-bit value
}

float adxl357_temp_celsius(void)
{
    uint16_t temp_raw = adxl357_read_temp_raw();
    // Nominal intercept: 1885 LSB at 25°C, slope: -9.05 LSB/°C
    return (1885.0f - (float)temp_raw) / 9.05f + 25.0f;
}


int main(void)
{
	printk("Starting ADXL357 SPI application\n");

	// Get SPI device
	spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
	if (!device_is_ready(spi_dev))
	{
		printk("SPI device not ready\n");
		return -1;
	}

	// Initialize CS GPIO
	if (!gpio_is_ready_dt(&cs_gpio))
	{
		printk("CS GPIO not ready\n");
		return -1;
	}

	int ret = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0)
	{
		printk("Failed to configure CS GPIO: %d\n", ret);
		return -1;
	}

	// Configure SPI - Use Mode 0 and lower frequency
	spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	spi_cfg.frequency = 1000000;
	spi_cfg.slave = 0;
	spi_cfg.cs = (struct spi_cs_control){
			.gpio = cs_gpio,
			.delay = 10};

	k_sleep(K_MSEC(100)); // Wait for sensor to boot

	// Read device ID to verify communication
	uint8_t dev_id = adxl357_read_reg(ADXL357_REG_DEVID_AD);
	// printk("Device ID: 0x%02X (expected: 0xAD)\n", dev_id);

	uint8_t part_id = adxl357_read_reg(ADXL357_REG_PARTID);
	// printk("Part ID: 0x%02X (expected: 0xED for ADXL357)\n", part_id);
	if (dev_id != 0xAD || part_id != 0xED)
	{
		printk("Device ID mismatch! Check wiring and SPI configuration.\n");
		return -1;
	}

	// Try standby first, then measurement
	adxl357_write_reg(ADXL357_REG_POWER_CTL, 0x01); // Standby
	k_sleep(K_MSEC(50));														// Longer delay

	adxl357_write_reg(ADXL357_REG_POWER_CTL, 0x00); // Measurement
	k_sleep(K_MSEC(50));														// Longer delay
	k_sleep(K_MSEC(100));

	// Set range to ±10g
	adxl357_write_reg(ADXL357_REG_RANGE, 0x01);

	uint8_t stat = adxl357_read_reg(ADXL357_REG_STATUS); // STATUS regularly 0x07 means that FIFO_OVR and FIFO_FULL and DATA_RDY.
	printk("STATUS: 0x%02X\n", stat);

	// Convert to g using 51200 LSB/g
	while (1)
	{
		if (stat & 0x01) // Data ready bit
		{
			temp_c = adxl357_temp_celsius();
			int32_t x = adxl357_read_axis(ADXL357_REG_XDATA3);
			int32_t y = adxl357_read_axis(ADXL357_REG_YDATA3);
			int32_t z = adxl357_read_axis(ADXL357_REG_ZDATA3);

			// Convert to g using 51200 LSB/g for ±10g range
			float x_g = (float)x / 51200.0f;
			float y_g = (float)y / 51200.0f;
			float z_g = (float)z / 51200.0f;

			//printk is not supporting float or double format so I enabled CONFIG_STDOUT_CONSOLE for use printf.
			printf("X: %.4f, Y: %.4f, Z: %.4f, Temp: %.2f °C\n", (double)x_g, (double)y_g, (double)z_g, (double)temp_c);
			
		}
		else
		{
			printk("Data not ready, Waiting.\n");
		}

		// printk("Raw Data - X: %d, Y: %d, Z: %d\n", x, y, z);
		// printk("X: %.4f g, Y: %.4f g, Z: %.4f g\n", (double)x_g, (double)y_g, (double)z_g);
		// printk("X: %d (%.4f g), Y: %d (%.4f g), Z: %d (%.4f g)\n", x, (double)x_g, y, (double)y_g, z, (double)z_g);

		k_sleep(K_MSEC(100));
	}
	return 0;
}