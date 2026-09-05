#include "bmi088_port.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"

#define BMI088_SPI_HOST   SPI2_HOST
#define BMI088_PIN_SCK    14
#define BMI088_PIN_MOSI   13
#define BMI088_PIN_MISO   27
#define BMI088_PIN_CS_ACC 15
#define BMI088_PIN_CS_GYR 4
#define BMI088_SPI_HZ     (5 * 1000 * 1000)

static spi_device_handle_t s_spi;
static const int s_cs_acc = BMI088_PIN_CS_ACC;
static const int s_cs_gyr = BMI088_PIN_CS_GYR;

static void cs_idle(void)
{
    gpio_set_level(BMI088_PIN_CS_ACC, 1);
    gpio_set_level(BMI088_PIN_CS_GYR, 1);
}

static void cs_select(int pin)
{
    cs_idle();
    esp_rom_delay_us(2);
    gpio_set_level(pin, 0);
    esp_rom_delay_us(2);
}

static BMI08_INTF_RET_TYPE spi_xfer(uint8_t reg_addr, const uint8_t *tx_data, uint8_t *rx_data, uint32_t len,
                                    void *intf_ptr)
{
    int cs = *(const int *)intf_ptr;
    uint8_t tx[1 + BMI08_MAX_LEN];
    uint8_t rx[1 + BMI08_MAX_LEN];

    if (intf_ptr == NULL || len == 0 || len > BMI08_MAX_LEN) {
        return -1;
    }

    tx[0] = reg_addr;
    if (tx_data != NULL) {
        memcpy(tx + 1, tx_data, len);
    } else {
        memset(tx + 1, 0, len);
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * (1 + len);
    t.tx_buffer = tx;
    t.rx_buffer = (rx_data != NULL) ? rx : NULL;

    cs_select(cs);
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    cs_idle();
    if (err != ESP_OK) {
        return -1;
    }
    if (rx_data != NULL) {
        memcpy(rx_data, rx + 1, len);
    }
    return BMI08_INTF_RET_SUCCESS;
}

static BMI08_INTF_RET_TYPE spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return spi_xfer(reg_addr, NULL, reg_data, len, intf_ptr);
}

static BMI08_INTF_RET_TYPE spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    return spi_xfer(reg_addr, reg_data, NULL, len, intf_ptr);
}

static void delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

esp_err_t bmi088_port_init(struct bmi08_dev *dev)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BMI088_PIN_CS_ACC) | (1ULL << BMI088_PIN_CS_GYR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    cs_idle();
    esp_rom_delay_us(100);
    gpio_set_level(BMI088_PIN_CS_ACC, 0);
    esp_rom_delay_us(100);
    gpio_set_level(BMI088_PIN_CS_ACC, 1);
    esp_rom_delay_us(200);

    spi_bus_config_t buscfg = {
        .mosi_io_num = BMI088_PIN_MOSI,
        .miso_io_num = BMI088_PIN_MISO,
        .sclk_io_num = BMI088_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    esp_err_t err = spi_bus_initialize(BMI088_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .mode = 3,
        .clock_speed_hz = BMI088_SPI_HZ,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    err = spi_bus_add_device(BMI088_SPI_HOST, &devcfg, &s_spi);
    if (err != ESP_OK) {
        return err;
    }

    memset(dev, 0, sizeof(*dev));
    dev->intf = BMI08_SPI_INTF;
    dev->variant = BMI088_VARIANT;
    dev->read_write_len = 32;
    dev->read = spi_read;
    dev->write = spi_write;
    dev->delay_us = delay_us;
    dev->intf_ptr_accel = (void *)&s_cs_acc;
    dev->intf_ptr_gyro = (void *)&s_cs_gyr;
    return ESP_OK;
}
