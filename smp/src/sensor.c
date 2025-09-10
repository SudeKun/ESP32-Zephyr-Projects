#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>
#include <zephyr/version.h>
#include <errno.h>
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

K_MUTEX_DEFINE(sensor_mutex);
static bool adxl_inited = false;

static const struct device *spi_dev;
static struct spi_config spi_cfg;
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi2), cs_gpios, 0);
static struct spi_cs_control spi_cs = {
    .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi2), cs_gpios, 0),
    .delay = 10,
};
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
    // printk("SPI write failed: %d\n", ret);
  }

  // Add debug print
  // printk("Write reg 0x%02X: tx[0]=0x%02X, tx[1]=0x%02X, value=0x%02X, ret=%d\n",reg, tx_buf[0], tx_buf[1], value, ret);
}

int32_t adxl357_read_axis(uint8_t reg_high)
{
  uint8_t msb = adxl357_read_reg(reg_high);     // MSB
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
  return ((temp2 << 8) | temp1);                              // 12-bit value
}

float adxl357_temp_celsius(void)
{
  uint16_t temp_raw = adxl357_read_temp_raw();
  // Nominal intercept: 1885 LSB at 25°C, slope: -9.05 LSB/°C
  return (1885.0f - (float)temp_raw) / 9.05f + 25.0f;
}

/* Forward declaration of thread entry */
static void sensor_thread(void *p1, void *p2, void *p3);

static struct k_thread sensor_thread_data;
K_THREAD_STACK_DEFINE(sensor_thread_stack, 4096);
static atomic_t sensor_thread_started = ATOMIC_INIT(0);
static atomic_t sensor_thread_stop = ATOMIC_INIT(0);

/* Start function called from shell command */
void start_smp_sensor(const struct shell *shell)
{
  if (atomic_cas(&sensor_thread_started, 0, 1))
  {
    atomic_set(&sensor_thread_stop, 0);
    /* First time: spawn */
    k_thread_create(&sensor_thread_data, sensor_thread_stack,
                    K_THREAD_STACK_SIZEOF(sensor_thread_stack),
                    sensor_thread, (void *)shell, NULL, NULL,
                    K_PRIO_PREEMPT(12), 0, K_NO_WAIT);
    k_thread_name_set(&sensor_thread_data, "adxl357");
    printk("sensor thread created (prio %d, stack %u)\n",
           k_thread_priority_get(k_current_get()),
           (unsigned)K_THREAD_STACK_SIZEOF(sensor_thread_stack));
  }
  else
  {
    printk("sensor thread already running\n");
  }
}

void stop_smp_sensor(void)
{
  if (!atomic_get(&sensor_thread_started))
  {
    printk("Sensor thread not running\n");
    return;
  }
  atomic_set(&sensor_thread_stop, 1);

  /* Wait (up to 500 ms) for graceful exit, then abort */
  for (int i = 0; i < 50; i++)
  {
    if (!atomic_get(&sensor_thread_started))
    {
      printk("Sensor thread stopped\n");
      return;
    }
    k_sleep(K_MSEC(10));
  }
  printk("Sensor thread aborting\n");
  printk("\033[2J\033[H");
  k_thread_abort(&sensor_thread_data);
  atomic_set(&sensor_thread_started, 0);
}

// Check
static int adxl357_init_once(void)
{
  if (adxl_inited)
  {
    return 0;
  }

  spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
  if (!device_is_ready(spi_dev))
  {
    printk("SPI device not ready\n");
    return -ENODEV;
  }
  if (!gpio_is_ready_dt(&cs_gpio))
  {
    printk("CS GPIO not ready\n");
    return -ENODEV;
  }
  int ret = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret < 0)
  {
    printk("Failed CS GPIO cfg: %d\n", ret);
    return ret;
  }

  spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
  spi_cfg.frequency = 1000000;
  spi_cfg.slave = 0;
  spi_cfg.cs = spi_cs;

  k_sleep(K_MSEC(100));

  uint8_t dev_id = adxl357_read_reg(ADXL357_REG_DEVID_AD);
  uint8_t part_id = adxl357_read_reg(ADXL357_REG_PARTID);
  if (dev_id != 0xAD || part_id != 0xED)
  {
    printk("Device ID mismatch (got 0x%02X 0x%02X)\n", dev_id, part_id);
    return -EIO;
  }

  adxl357_write_reg(ADXL357_REG_POWER_CTL, 0x01);
  k_sleep(K_MSEC(50));
  adxl357_write_reg(ADXL357_REG_POWER_CTL, 0x00);
  k_sleep(K_MSEC(50));
  adxl357_write_reg(ADXL357_REG_RANGE, 0x01);

  adxl_inited = true;
  return 0;
}

// 20-bit two's complement spans ±2^19 counts.
static inline float adxl357_counts_per_g(void)
{
  uint8_t r = adxl357_read_reg(ADXL357_REG_RANGE) & 0x03;
  int fs_g; // full-scale in g
  switch (r)
  {
  case 0x01:
    fs_g = 10;
    break; // ±10 g
  case 0x02:
    fs_g = 20;
    break; // ±20 g
  case 0x03:
    fs_g = 40;
    break; // ±40 g
  default:
    fs_g = 10;
    break; // fallback
  }
  return (float)(1 << 19) / (float)fs_g; // 524288 / FS[g]
}

static inline float adxl357_to_g(int32_t raw20)
{
  return (float)raw20 / adxl357_counts_per_g();
}

static inline float g_to_ms2(float g)
{
  return g * 9.80665f;
}

// Read regularly (UART)
static void sensor_thread(void *p1, void *p2, void *p3)
{
  const struct shell *shell = (const struct shell *)p1;
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);
  /* Fallback if no shell pointer */
  if (!shell)
  {
    shell = shell_backend_uart_get_ptr();
    printk("Sensor shell not set, using UART shell\n");
  }

  for (int i = 0; i < 3; i++)
  {
    printk("sensor init phase %d\n", i);
    k_sleep(K_MSEC(50));
  }

  printk("Starting ADXL357 SPI application\n");

  spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
  if (!device_is_ready(spi_dev))
  {
    printk("SPI device not ready\n");
    goto done;
  }
  if (!gpio_is_ready_dt(&cs_gpio))
  {
    printk("CS GPIO not ready\n");
    goto done;
  }
  int ret = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret < 0)
  {
    printk("Failed CS GPIO cfg: %d\n", ret);
    goto done;
  }

  spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
  spi_cfg.frequency = 1000000;
  spi_cfg.slave = 0;
  spi_cfg.cs = spi_cs;

  k_sleep(K_MSEC(100));
  ret = adxl357_init_once();
  if (ret != 0)
  {
    printk("Failed init: %d\n", ret);
    goto done;
  }

  while (!atomic_get(&sensor_thread_stop))
  {
    uint8_t stat = adxl357_read_reg(ADXL357_REG_STATUS);
    if (stat & 0x01)
    {
      float t = adxl357_temp_celsius();
      int32_t x = adxl357_read_axis(ADXL357_REG_XDATA3);
      int32_t y = adxl357_read_axis(ADXL357_REG_YDATA3);
      int32_t z = adxl357_read_axis(ADXL357_REG_ZDATA3);

      float gx = adxl357_to_g(x);
      float gy = adxl357_to_g(y);
      float gz = adxl357_to_g(z);
      t = adxl357_temp_celsius();

      printk("X:%.3f m/s^2 "
             "Y:%.3f m/s^2 "
             "Z:%.3f m/s^2 T:%.2f C\n",
             (double)g_to_ms2(gx),
             (double)g_to_ms2(gy),
             (double)g_to_ms2(gz), (double)t);
    }
    else
    {
      printk("STATUS 0x%02X\n", stat);
    }
    k_sleep(K_MSEC(200));
  }
done:
  atomic_set(&sensor_thread_started, 0);
  printk("Sensor thread exiting\n");
  shell = NULL;
}

// Public one-shot read (to be called from shell)
int sensor_read_once(char *buf, size_t len)
{
  if (!buf || len == 0)
    return -EINVAL;

  int rc = k_mutex_lock(&sensor_mutex, K_MSEC(200));
  if (rc != 0)
    return -EBUSY;

  rc = adxl357_init_once();

  if (rc != 0)
  {
    k_mutex_unlock(&sensor_mutex);
    snprintf(buf, len, "init failed (%d)", rc);
    return rc;
  }

  uint8_t stat = adxl357_read_reg(ADXL357_REG_STATUS);

  if (stat & 0x01)
  {
    float t = adxl357_temp_celsius();
    int32_t x = adxl357_read_axis(ADXL357_REG_XDATA3);
    int32_t y = adxl357_read_axis(ADXL357_REG_YDATA3);
    int32_t z = adxl357_read_axis(ADXL357_REG_ZDATA3);
    float gx = adxl357_to_g(x), gy = adxl357_to_g(y), gz = adxl357_to_g(z);

    snprintf(buf, len, "X:%.3f m/s^2 "
                       "Y:%.3f m/s^2 "
                       "Z:%.3f m/s^2 T:%.2f C\n",
                       (double)g_to_ms2(gx),
                       (double)g_to_ms2(gy),
                       (double)g_to_ms2(gz), (double)t);
    rc = 0;
  }
  else
  {
    snprintf(buf, len, "STATUS 0x%02X", stat);
    rc = -EAGAIN;
  }

  k_mutex_unlock(&sensor_mutex);
  return rc;
}

// --- IGNORE ---
/*
printk("X:%ld (%.3fg, %.1fmg, %.3fm/s^2) "
          "Y:%ld (%.3fg, %.1fmg, %.3fm/s^2) "
          "Z:%ld (%.3fg, %.1fmg, %.3fm/s^2) T:%.2fC\n",
          (long)x, (double)gx, (double)(gx * 1000.0f), (double)g_to_ms2(gx),
          (long)y, (double)gy, (double)(gy * 1000.0f), (double)g_to_ms2(gy),
          (long)z, (double)gz, (double)(gz * 1000.0f), (double)g_to_ms2(gz), (double)t);
*/