#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_check.h"
#include "kode_bq25896.h"
#include "kode_bq25896_priv.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

static const char *TAG = "kode_bq25896";

// Define our own sentinel value for temperature
#define BQ25896_TEMP_INVALID (-32768)

/**
 * @brief Read a register from the BQ25896
 * 
 * @param handle Device handle
 * @param reg Register address
 * @param data Pointer to store the read value
 * @return esp_err_t ESP_OK on success, error otherwise
 */
static esp_err_t bq25896_read_reg(bq25896_handle_t handle, uint8_t reg, uint8_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Create device handle if not already created
    static i2c_master_dev_handle_t dev_handle = NULL;
    if (dev_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = handle->dev_addr,
            .scl_speed_hz = 400000,  // 400 kHz
        };
        
        esp_err_t ret = i2c_master_bus_add_device(handle->i2c_bus, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device: %d", ret);
            return ret;
        }
    }
    
    // Write register address
    uint8_t write_buf = reg;
    esp_err_t ret = i2c_master_transmit(dev_handle, &write_buf, 1, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address: %d", ret);
        return ret;
    }
    
    // Read register data
    ret = i2c_master_receive(dev_handle, data, 1, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register data: %d", ret);
    }
    
    return ret;
}

/**
 * @brief Write a register to the BQ25896
 * 
 * @param handle Device handle
 * @param reg Register address
 * @param data Value to write
 * @return esp_err_t ESP_OK on success, error otherwise
 */
static esp_err_t bq25896_write_reg(bq25896_handle_t handle, uint8_t reg, uint8_t data)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Create device handle if not already created
    static i2c_master_dev_handle_t dev_handle = NULL;
    if (dev_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = handle->dev_addr,
            .scl_speed_hz = 400000,  // 400 kHz
        };
        
        esp_err_t ret = i2c_master_bus_add_device(handle->i2c_bus, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device: %d", ret);
            return ret;
        }
    }
    
    // Prepare buffer with register address and data
    uint8_t write_buffer[2] = {reg, data};
    
    // Send register address and data
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buffer, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register: %d", ret);
    }
    
    return ret;
}

/**
 * @brief Update specific bits in a register
 * 
 * @param handle Device handle
 * @param reg Register address
 * @param mask Bit mask
 * @param shift Bit shift
 * @param value Value to set
 * @return esp_err_t ESP_OK on success, error otherwise
 */
static esp_err_t bq25896_update_bits(bq25896_handle_t handle, uint8_t reg, 
                                      uint8_t mask, uint8_t shift, uint8_t value)
{
    uint8_t reg_data;
    
    // Read current value
    esp_err_t ret = bq25896_read_reg(handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Clear bits we're updating
    reg_data &= ~(mask);
    
    // Set new bits
    reg_data |= ((value << shift) & mask);
    
    // Write back the updated value
    return bq25896_write_reg(handle, reg, reg_data);
}

/**
 * @brief Initialize the BQ25896 device
 * 
 * @param i2c_bus I2C bus handle
 * @param dev_addr Device I2C address
 * @param handle Pointer to store the device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_init(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr, bq25896_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(i2c_bus != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid I2C bus handle");
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle pointer");
    
    bq25896_handle_t dev = calloc(1, sizeof(struct bq25896_dev_t));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for device");
    
    dev->i2c_bus = i2c_bus;
    dev->dev_addr = dev_addr;
    
    // Verify device communication by reading Part Number register
    uint8_t chip_id;
    esp_err_t ret = bq25896_read_reg(dev, BQ25896_REG_0A, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID register");
        free(dev);
        return ret;
    }
    
    *handle = dev;
    ESP_LOGI(TAG, "BQ25896 initialized successfully");
    return ESP_OK;
}

/**
 * @brief Delete the BQ25896 device instance
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_delete(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    free(handle);
    return ESP_OK;
}

/**
 * @brief Reset the BQ25896 to default state
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_reset(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Set the REG_RST bit in REG14
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG_14, 
                                        BQ25896_REG14_REG_RST_MASK,
                                        BQ25896_REG14_REG_RST_SHIFT, 1);
    
    if (ret == ESP_OK) {
        // Wait for reset to complete (datasheet suggests it auto-clears when done)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ret;
}

/**
 * @brief Enable/disable High Impedance mode (HIZ)
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_hiz(bq25896_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    return bq25896_update_bits(handle, BQ25896_REG_00, 
                               BQ25896_REG00_ENHIZ_MASK,
                               BQ25896_REG00_ENHIZ_SHIFT, enable ? 1 : 0);
}

/**
 * @brief Enable/disable charging
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_charging(bq25896_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Updated to use REG03 instead of REG01
    return bq25896_update_bits(handle, BQ25896_REG_03, 
                               BQ25896_REG03_CHG_CONFIG_MASK,
                               BQ25896_REG03_CHG_CONFIG_SHIFT, enable ? 1 : 0);
}

/**
 * @brief Enable/disable OTG mode
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_otg(bq25896_handle_t handle, bool enable)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Updated to use REG03 instead of REG01
    return bq25896_update_bits(handle, BQ25896_REG_03, 
                               BQ25896_REG03_OTG_CONFIG_MASK,
                               BQ25896_REG03_OTG_CONFIG_SHIFT, enable ? 1 : 0);
}

/**
 * @brief Set charge current
 * 
 * @param handle Device handle
 * @param current_ma Current in mA (range 0-5056, resolution 64mA)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_current(bq25896_handle_t handle, uint16_t current_ma)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Limit to maximum value
    if (current_ma > 5056) {
        current_ma = 5056;
        ESP_LOGW(TAG, "Charge current capped at 5056mA");
    }
    
    // Convert to register value (0mA = 0, 5056mA = 0x7F)
    uint8_t reg_val = current_ma / 64;
    
    return bq25896_update_bits(handle, BQ25896_REG_04, 
                              BQ25896_REG04_ICHG_MASK,
                              BQ25896_REG04_ICHG_SHIFT, reg_val);
}

/**
 * @brief Set charge voltage
 * 
 * @param handle Device handle
 * @param voltage_mv Voltage in mV (range 3840-4608, resolution 16mV)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_voltage(bq25896_handle_t handle, uint16_t voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Limit to valid range
    if (voltage_mv < 3840) {
        voltage_mv = 3840;
        ESP_LOGW(TAG, "Charge voltage increased to minimum 3840mV");
    } else if (voltage_mv > 4608) {
        voltage_mv = 4608;
        ESP_LOGW(TAG, "Charge voltage capped at 4608mV");
    }
    
    // Convert to register value (3840mV = 0, 4608mV = 0x30)
    uint8_t reg_val = (voltage_mv - 3840) / 16;
    
    return bq25896_update_bits(handle, BQ25896_REG_06, 
                              BQ25896_REG06_VREG_MASK,
                              BQ25896_REG06_VREG_SHIFT, reg_val);
}

/**
 * @brief Set input current limit
 * 
 * @param handle Device handle
 * @param ilim Input current limit value from enum
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_input_current_limit(bq25896_handle_t handle, bq25896_ilim_t ilim)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(ilim <= BQ25896_ILIM_3000MA, ESP_ERR_INVALID_ARG, TAG, "Invalid ILIM value");
    
    // Update IINLIM register bits
    return bq25896_update_bits(handle, BQ25896_REG_00, 
                              BQ25896_REG00_IINLIM_MASK,
                              BQ25896_REG00_IINLIM_SHIFT, ilim);
}

/**
 * @brief Set input voltage limit in mV
 * 
 * @param handle Device handle
 * @param voltage_mv Voltage in mV (range 3900-14000, resolution 100mV)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_input_voltage_limit(bq25896_handle_t handle, uint16_t voltage_mv)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // Limit to valid range
    if (voltage_mv < 3900) {
        voltage_mv = 3900;
        ESP_LOGW(TAG, "Input voltage limit increased to minimum 3900mV");
    } else if (voltage_mv > 14000) {
        voltage_mv = 14000;
        ESP_LOGW(TAG, "Input voltage limit capped at 14000mV");
    }
    
    // Calculate register value (3900mV base, 100mV steps)
    uint8_t reg_val = BQ25896_VINDPM_TO_REG(voltage_mv);
    
    // First enable absolute VINDPM threshold mode
    esp_err_t ret = bq25896_update_bits(handle, BQ25896_REG_0D,
                                      BQ25896_REG0D_FORCE_VINDPM_MASK,
                                      BQ25896_REG0D_FORCE_VINDPM_SHIFT, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Then set the VINDPM threshold value
    return bq25896_update_bits(handle, BQ25896_REG_0D,
                             BQ25896_REG0D_VINDPM_MASK,
                             BQ25896_REG0D_VINDPM_SHIFT, reg_val);
}

/**
 * @brief Configure the BQ25896 with the given settings
 * 
 * @param handle Device handle
 * @param config Configuration structure
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_configure(bq25896_handle_t handle, const bq25896_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
    
    // Store configuration in device handle
    memcpy(&handle->config, config, sizeof(bq25896_config_t));
    
    esp_err_t ret;
    
    // 1. Enable/disable HIZ mode
    ret = bq25896_enable_hiz(handle, config->enable_hiz);
    if (ret != ESP_OK) return ret;
    
    // 2. Set input current limit
    ret = bq25896_set_input_current_limit(handle, config->input_current_limit);
    if (ret != ESP_OK) return ret;
    
    // 3. Set charge current
    ret = bq25896_set_charge_current(handle, config->charge_current_ma);
    if (ret != ESP_OK) return ret;
    
    // 4. Set charge voltage
    ret = bq25896_set_charge_voltage(handle, config->charge_voltage_mv);
    if (ret != ESP_OK) return ret;
    
    // 5. Enable/disable charging
    ret = bq25896_enable_charging(handle, config->enable_charging);
    if (ret != ESP_OK) return ret;
    
    // 6. Enable/disable OTG
    ret = bq25896_enable_otg(handle, config->enable_otg);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "BQ25896 configured: charging=%d, current=%dmA, voltage=%dmV",
             config->enable_charging, config->charge_current_ma, config->charge_voltage_mv);
    
    return ESP_OK;
}

/**
 * @brief Get the current status of the BQ25896
 * 
 * @param handle Device handle
 * @param status Pointer to store status information
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_status(bq25896_handle_t handle, bq25896_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid status pointer");
    
    uint8_t reg_val;
    esp_err_t ret;
    
    // Read status register (REG0B)
    ret = bq25896_read_reg(handle, BQ25896_REG_0B, &reg_val);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse VBUS status
    status->vbus_status = (reg_val & BQ25896_REG0B_VBUS_STAT_MASK) >> BQ25896_REG0B_VBUS_STAT_SHIFT;
    
    // Parse charge status
    status->chg_status = (reg_val & BQ25896_REG0B_CHRG_STAT_MASK) >> BQ25896_REG0B_CHRG_STAT_SHIFT;
    
    // Parse power good
    status->power_good = (reg_val & BQ25896_REG0B_PG_STAT_MASK) ? true : false;
    
    // Parse VSYS status (1 when VSYS drops below VSYSMIN)
    status->vsys_below_min = (reg_val & BQ25896_REG0B_VSYS_STAT_MASK) ? true : false;
    
    // Read Input Current Limit register (REG13)
    ret = bq25896_read_reg(handle, BQ25896_REG_13, &reg_val);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Check for VINDPM and IINDPM regulation
    status->in_vindpm_state = (reg_val & BQ25896_REG13_VDPM_STAT_MASK) ? true : false;
    status->in_iindpm_state = (reg_val & BQ25896_REG13_IDPM_STAT_MASK) ? true : false;
    
    // Check thermal regulation status (REG10 bit 7)
    ret = bq25896_read_reg(handle, BQ25896_REG_10, &reg_val);
    if (ret != ESP_OK) return ret;
    
    status->thermal_regulation = (reg_val & BQ25896_REG10_THERM_STAT_MASK) ? true : false;
    ESP_LOGD(TAG, "Thermal regulation: %s", status->thermal_regulation ? "active" : "inactive");
    
    return ESP_OK;
}

/**
 * @brief Get the current readings from the BQ25896
 * 
 * @param handle Device handle
 * @param readings Pointer to store reading information
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_readings(bq25896_handle_t handle, bq25896_readings_t *readings)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(readings != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid readings pointer");
    
    uint8_t reg_val;
    esp_err_t ret;
    
    // Read battery voltage (REG0E - BATV)
    ret = bq25896_read_reg(handle, BQ25896_REG_0E, &reg_val);
    if (ret != ESP_OK) return ret;
    
    // Convert register value to voltage using the VBAT conversion macro
    readings->vbat_mv = BQ25896_VBAT_CONV(reg_val & BQ25896_REG0E_BATV_MASK);
    
    // Get system voltage (REG0F - SYSV)
    ret = bq25896_read_reg(handle, BQ25896_REG_0F, &reg_val);
    if (ret != ESP_OK) return ret;
    
    // Convert register value to voltage using the VSYS conversion macro
    readings->vsys_mv = BQ25896_VSYS_CONV(reg_val & BQ25896_REG0F_SYSV_MASK);
    
    // Read VBUS voltage (REG11 - VBUSV)
    ret = bq25896_read_reg(handle, BQ25896_REG_11, &reg_val);
    if (ret != ESP_OK) return ret;
    
    // Check VBUS present status in bit 7
    bool vbus_present = (reg_val & BQ25896_REG11_VBUS_GD_MASK) ? true : false;
    
    // Convert register value to VBUS voltage using the VBUS conversion macro
    readings->vbus_mv = vbus_present ? BQ25896_VBUS_CONV(reg_val & BQ25896_REG11_VBUSV_MASK) : 0;
    
    // Get charging current (REG12 - ICHGR)
    ret = bq25896_read_reg(handle, BQ25896_REG_12, &reg_val);
    if (ret != ESP_OK) return ret;
    
    // Convert register value to current using the ICHG conversion macro
    readings->ichg_ma = BQ25896_ICHG_CONV(reg_val & BQ25896_REG12_ICHGR_MASK);
    
    // Read battery temperature (REG10 - TSPCT)
    ret = bq25896_read_reg(handle, BQ25896_REG_10, &reg_val);
    if (ret != ESP_OK) return ret;
    
    // Since no thermistor is installed, don't attempt to convert to temperature
    // Set to a sentinel value to indicate no valid temperature reading
    readings->die_temp_c = BQ25896_TEMP_INVALID;
    
    return ESP_OK;
}

/**
 * @brief Enter shipping mode by setting the BATFET_DIS bit in REG09
 * 
 * This will disconnect the battery from the system (VSYS) by turning off the BATFET.
 * The device will power down completely and can only be reactivated by applying
 * power to VBUS.
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enter_shipping_mode(bq25896_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    ESP_LOGI(TAG, "Entering shipping mode (BATFET disabled)");
    
    // Set the BATFET_DIS bit in REG09 to disconnect the battery
    return bq25896_update_bits(handle, BQ25896_REG_09, 
                              BQ25896_REG09_BATFET_DIS_MASK,
                              BQ25896_REG09_BATFET_DIS_SHIFT, 1);
}
