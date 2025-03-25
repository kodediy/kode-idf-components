# BQ25896 Battery Charger Driver

This is a component for ESP-IDF that provides support for the Texas Instruments BQ25896 battery charger IC.

## Features

- Complete I2C driver for BQ25896 power management IC
- Configurable charging parameters (voltage, current, etc.)
- Battery status monitoring
- Input power source detection
- OTG (On-The-Go) mode support for power output
- Advanced power path management

## Hardware Connection

The BQ25896 IC communicates with the ESP32 using I2C. Connect the following pins:

- Connect SCL pin of the ESP32 to the SCL pin of BQ25896
- Connect SDA pin of the ESP32 to the SDA pin of BQ25896
- Make sure to add appropriate pull-up resistors to both SCL and SDA lines

## Example Usage

```c
#include "kode_bq25896.h"

// Example of using the BQ25896 driver
void app_main(void)
{
    // Initialize I2C bus
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CONFIG_I2C_SCL_GPIO,
        .sda_io_num = CONFIG_I2C_SDA_GPIO,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

    // Initialize BQ25896
    bq25896_handle_t bq_handle = NULL;
    ESP_ERROR_CHECK(bq25896_init(i2c_bus, BQ25896_I2C_ADDR, &bq_handle));

    // Configure with default settings
    bq25896_config_t config = BQ25896_DEFAULT_CONFIG();
    
    // Customize configuration if needed
    config.charge_voltage_mv = 4200;      // Set charge voltage to 4.2V
    config.charge_current_ma = 1000;      // Set charge current to 1A
    config.input_current_limit = BQ25896_ILIM_1500MA; // Set input current limit to 1.5A
    
    // Apply configuration
    ESP_ERROR_CHECK(bq25896_configure(bq_handle, &config));

    // Enable charging
    ESP_ERROR_CHECK(bq25896_enable_charging(bq_handle, true));

    // Read status periodically
    while (1) {
        bq25896_status_t status;
        bq25896_readings_t readings;
        
        ESP_ERROR_CHECK(bq25896_get_status(bq_handle, &status));
        ESP_ERROR_CHECK(bq25896_get_readings(bq_handle, &readings));
        
        printf("Battery: %d mV, Charging: %s, Current: %d mA\n", 
               readings.vbat_mv,
               status.chg_status == BQ25896_CHG_STATUS_FAST_CHARGE ? "Yes" : "No",
               readings.ichg_ma);
               
        vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
    }
}
```

## API Reference

### Initialization and Deinitialization

```c
esp_err_t bq25896_init(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr, bq25896_handle_t *handle);
esp_err_t bq25896_delete(bq25896_handle_t handle);
```

### Configuration

```c
esp_err_t bq25896_configure(bq25896_handle_t handle, const bq25896_config_t *config);
```

### Status and Reading Functions

```c
esp_err_t bq25896_get_status(bq25896_handle_t handle, bq25896_status_t *status);
esp_err_t bq25896_get_readings(bq25896_handle_t handle, bq25896_readings_t *readings);
```

### Control Functions

```c
esp_err_t bq25896_enable_charging(bq25896_handle_t handle, bool enable);
esp_err_t bq25896_enable_otg(bq25896_handle_t handle, bool enable);
esp_err_t bq25896_reset(bq25896_handle_t handle);
esp_err_t bq25896_enable_hiz(bq25896_handle_t handle, bool enable);
```

### Parameter Setting Functions

```c
esp_err_t bq25896_set_charge_current(bq25896_handle_t handle, uint16_t current_ma);
esp_err_t bq25896_set_charge_voltage(bq25896_handle_t handle, uint16_t voltage_mv);
esp_err_t bq25896_set_input_current_limit(bq25896_handle_t handle, bq25896_ilim_t ilim);
esp_err_t bq25896_set_input_voltage_limit(bq25896_handle_t handle, uint16_t voltage_mv);
```

## References

- [BQ25896 Datasheet](https://www.ti.com/lit/ds/symlink/bq25896.pdf) 