#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kode_max17048.h"
#include "kode_max17048_priv.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX17048";

/**
 * @brief Default configuration for MAX17048
 */
const max17048_config_t MAX17048_DEFAULT_CONFIG = {
    .rcomp = MAX17048_CONFIG_RCOMP_DEFAULT,
    .alert_threshold = 4,                    // 4% empty alert
    .enable_soc_change_alert = false,
    .enable_voltage_alerts = false,
    .voltage_min_mv = 3000,                  // 3.0V min voltage
    .voltage_max_mv = 4200,                  // 4.2V max voltage
    .enable_hibernate = false,
    .hibernate_threshold_pct_per_hr = 0.0f,
    .active_threshold_mv = 0.0f,
    .disable_comparator = false,
    .vreset_threshold_mv = 2500,             // 2.5V for captive batteries
    .enable_vreset_alert = false
};

esp_err_t max17048_init(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr, max17048_handle_t *handle)
{
    if (handle == NULL || i2c_bus == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate memory for device structure
    max17048_dev_t *dev = (max17048_dev_t *)calloc(1, sizeof(max17048_dev_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }

    // Initialize device structure
    dev->i2c_bus = i2c_bus;
    dev->dev_addr = dev_addr;
    
    // Copy default configuration
    memcpy(&dev->config, &MAX17048_DEFAULT_CONFIG, sizeof(max17048_config_t));

    // Verify device is present by reading the version register
    max17048_reg_t version;
    esp_err_t ret = max17048_read_reg(dev, MAX17048_VERSION_REG, &version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read version register");
        free(dev);
        return ret;
    }

    if (version != MAX17048_VERSION_DEFAULT) {
        ESP_LOGW(TAG, "Unexpected version: 0x%04X, expected 0x%04X", version, MAX17048_VERSION_DEFAULT);
    }
    
    // Check if device needs configuration
    bool needs_config;
    ret = max17048_needs_config(dev, &needs_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to check configuration status");
        free(dev);
        return ret;
    }
    
    if (needs_config) {
        ESP_LOGI(TAG, "Device needs configuration, applying defaults");
        ret = max17048_init_config_default(dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to apply default configuration");
            free(dev);
            return ret;
        }
    }

    // Return the handle
    *handle = dev;
    
    ESP_LOGI(TAG, "MAX17048 initialized successfully");
    return ESP_OK;
}

esp_err_t max17048_delete(max17048_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    free(handle);
    return ESP_OK;
}

esp_err_t max17048_read_reg(max17048_handle_t handle, uint8_t reg_addr, max17048_reg_t *value)
{
    if (handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    max17048_dev_t *dev = (max17048_dev_t *)handle;
    uint8_t buffer[2] = {0};
    uint8_t write_buf = reg_addr;
    
    // Configure I2C read transaction
    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev->dev_addr,
        .scl_speed_hz = 400000, // 400 KHz standard for most I2C devices
    };
    
    // Create device for this transaction
    esp_err_t ret = i2c_master_bus_add_device(dev->i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read register (write address, then read data)
    ret = i2c_master_transmit_receive(dev_handle, &write_buf, 1, buffer, 2, -1);
    
    // Clean up device
    i2c_master_bus_rm_device(dev_handle);
    
    if (ret == ESP_OK) {
        // MAX17048 uses MSB first
        *value = (buffer[0] << 8) | buffer[1];
    }
    
    return ret;
}

esp_err_t max17048_write_reg(max17048_handle_t handle, uint8_t reg_addr, max17048_reg_t value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    max17048_dev_t *dev = (max17048_dev_t *)handle;
    
    // Create I2C commands
    uint8_t buffer[3];
    buffer[0] = reg_addr;
    buffer[1] = (value >> 8) & 0xFF;  // MSB
    buffer[2] = value & 0xFF;         // LSB
    
    // Configure I2C write transaction
    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev->dev_addr,
        .scl_speed_hz = 400000, // 400 KHz standard for most I2C devices
    };
    
    // Create device for this transaction
    esp_err_t ret = i2c_master_bus_add_device(dev->i2c_bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write register data
    ret = i2c_master_transmit(dev_handle, buffer, sizeof(buffer), -1);
    
    // Clean up device
    i2c_master_bus_rm_device(dev_handle);
    
    return ret;
}

/* Example of implementing other functions */

esp_err_t max17048_get_vcell_mv(max17048_handle_t handle, float *voltage_mv)
{
    if (handle == NULL || voltage_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t vcell_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_VCELL_REG, &vcell_raw);
    if (ret == ESP_OK) {
        *voltage_mv = MAX17048_VCELL_TO_MV(vcell_raw);
    }
    
    return ret;
}

esp_err_t max17048_get_vcell_raw(max17048_handle_t handle, max17048_reg_t *vcell_raw)
{
    if (handle == NULL || vcell_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_VCELL_REG, vcell_raw);
}

esp_err_t max17048_get_soc_percent(max17048_handle_t handle, float *soc_percent)
{
    if (handle == NULL || soc_percent == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t soc_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_SOC_REG, &soc_raw);
    if (ret == ESP_OK) {
        *soc_percent = MAX17048_SOC_TO_PERCENT(soc_raw);
    }
    
    return ret;
}

esp_err_t max17048_get_soc_raw(max17048_handle_t handle, max17048_reg_t *soc_raw)
{
    if (handle == NULL || soc_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_SOC_REG, soc_raw);
}

esp_err_t max17048_get_soc_parts(max17048_handle_t handle, uint8_t *soc_integer, uint8_t *soc_fraction)
{
    if (handle == NULL || soc_integer == NULL || soc_fraction == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t soc_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_SOC_REG, &soc_raw);
    if (ret == ESP_OK) {
        *soc_integer = (soc_raw & MAX17048_SOC_INT_MASK) >> MAX17048_SOC_INT_SHIFT;
        *soc_fraction = soc_raw & MAX17048_SOC_DEC_MASK;
    }
    
    return ret;
}

/* Charge Rate Functions */

esp_err_t max17048_get_charge_rate(max17048_handle_t handle, float *rate_pct_per_hr)
{
    if (handle == NULL || rate_pct_per_hr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t crate_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CRATE_REG, &crate_raw);
    if (ret == ESP_OK) {
        // CRATE is signed - positive is charging, negative is discharging
        *rate_pct_per_hr = MAX17048_CRATE_TO_PCTH(crate_raw);
    }
    
    return ret;
}

esp_err_t max17048_get_crate_raw(max17048_handle_t handle, max17048_reg_t *crate_raw)
{
    if (handle == NULL || crate_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_CRATE_REG, crate_raw);
}

esp_err_t max17048_get_battery_data(max17048_handle_t handle, max17048_data_t *data)
{
    if (handle == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    // Get voltage
    ret = max17048_get_vcell_mv(handle, &data->voltage);
    if (ret != ESP_OK) return ret;
    data->voltage /= 1000.0f; // Convert to volts
    
    // Get SOC
    ret = max17048_get_soc_percent(handle, &data->soc);
    if (ret != ESP_OK) return ret;
    
    // Get charge rate
    ret = max17048_get_charge_rate(handle, &data->charge_rate);
    if (ret != ESP_OK) return ret;
    
    // Get version
    ret = max17048_read_reg(handle, MAX17048_VERSION_REG, &data->version);
    if (ret != ESP_OK) return ret;
    
    // Get status
    ret = max17048_read_reg(handle, MAX17048_STATUS_REG, &data->status);
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

/* 
 * Additional function implementations would follow the same pattern,
 * reading/writing registers using the device handle
 */

esp_err_t max17048_reset(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Write the POR command to the CMD register
    esp_err_t ret = max17048_write_reg(handle, MAX17048_CMD_REG, MAX17048_CMD_POR_VALUE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device");
        return ret;
    }
    
    // Wait a moment for the device to reset
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Apply default configuration after reset
    return max17048_init_config_default(handle);
}

esp_err_t max17048_configure(max17048_handle_t handle, const max17048_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    esp_err_t ret = ESP_OK;
    
    // Store the configuration
    memcpy(&dev->config, config, sizeof(max17048_config_t));
    
    // Configure RCOMP and alert threshold
    ret = max17048_set_rcomp(handle, config->rcomp);
    if (ret != ESP_OK) return ret;
    
    ret = max17048_set_empty_alert_threshold(handle, config->alert_threshold);
    if (ret != ESP_OK) return ret;
    
    // Configure SOC change alerts
    ret = max17048_set_soc_change_alert(handle, config->enable_soc_change_alert);
    if (ret != ESP_OK) return ret;
    
    // Configure voltage alerts if enabled
    if (config->enable_voltage_alerts) {
        ret = max17048_set_voltage_alerts(handle, config->voltage_max_mv, config->voltage_min_mv);
    } else {
        ret = max17048_disable_voltage_alerts(handle);
    }
    if (ret != ESP_OK) return ret;
    
    // Configure hibernate mode if enabled
    if (config->enable_hibernate) {
        ret = max17048_configure_hibernate(handle, 
                                         config->hibernate_threshold_pct_per_hr, 
                                         config->active_threshold_mv);
    } else {
        ret = max17048_disable_hibernate(handle);
    }
    if (ret != ESP_OK) return ret;
    
    // Configure VRESET 
    ret = max17048_configure_vreset(handle, 
                                  config->vreset_threshold_mv,
                                  config->disable_comparator);
    if (ret != ESP_OK) return ret;
    
    // Configure voltage reset alert
    ret = max17048_set_voltage_reset_alert(handle, config->enable_vreset_alert);
    if (ret != ESP_OK) return ret;
    
    // Clear any existing alerts
    ret = max17048_clear_alert(handle);
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

uint32_t max17048_get_driver_version(void)
{
    return (KODE_MAX17048_DRIVER_MAJOR << 16) | 
           (KODE_MAX17048_DRIVER_MINOR << 8) | 
           KODE_MAX17048_DRIVER_PATCH;
}

esp_err_t max17048_get_default_config(max17048_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(config, &MAX17048_DEFAULT_CONFIG, sizeof(max17048_config_t));
    return ESP_OK;
}

/**
 * @brief Initialize the CONFIG register with default values
 */
esp_err_t max17048_init_config_default(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set default RCOMP and ATHD values
    return max17048_write_reg(handle, MAX17048_CONFIG_REG, MAX17048_CONFIG_DEFAULT);
}

/**
 * @brief Check if the device needs configuration by checking the reset indicator
 */
esp_err_t max17048_needs_config(max17048_handle_t handle, bool *needs_config)
{
    if (handle == NULL || needs_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t status;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_STATUS_REG, &status);
    if (ret == ESP_OK) {
        *needs_config = (status & MAX17048_STATUS_RI_BIT) ? true : false;
    }
    
    return ret;
}

/* Add more function implementations to match the header file */

/* Power Mode Functions */

esp_err_t max17048_quick_start(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set the quick-start bit in MODE register
    return max17048_write_reg(handle, MAX17048_MODE_REG, MAX17048_MODE_QUICKSTART_BIT);
}

esp_err_t max17048_set_sleep_mode(max17048_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current CONFIG value
    max17048_reg_t config_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CONFIG_REG, &config_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Update sleep bit
    if (enable) {
        config_raw |= MAX17048_CONFIG_SLEEP_BIT;
    } else {
        config_raw &= ~MAX17048_CONFIG_SLEEP_BIT;
    }
    
    // Write updated CONFIG value
    return max17048_write_reg(handle, MAX17048_CONFIG_REG, config_raw);
}

esp_err_t max17048_is_hibernating(max17048_handle_t handle, bool *is_hibernating)
{
    if (handle == NULL || is_hibernating == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t mode_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_MODE_REG, &mode_raw);
    if (ret == ESP_OK) {
        *is_hibernating = (mode_raw & MAX17048_MODE_HIBSTAT_BIT) ? true : false;
    }
    
    return ret;
}

esp_err_t max17048_get_mode_raw(max17048_handle_t handle, max17048_reg_t *mode_raw)
{
    if (handle == NULL || mode_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_MODE_REG, mode_raw);
}

/* Version Functions */

esp_err_t max17048_get_version(max17048_handle_t handle, max17048_reg_t *version)
{
    if (handle == NULL || version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_VERSION_REG, version);
}

esp_err_t max17048_verify_version(max17048_handle_t handle, bool *version_ok)
{
    if (handle == NULL || version_ok == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t version;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_VERSION_REG, &version);
    if (ret == ESP_OK) {
        *version_ok = (version == MAX17048_VERSION_DEFAULT);
    }
    
    return ret;
}

/* Hibernate Functions */

esp_err_t max17048_configure_hibernate(max17048_handle_t handle, float hib_thr_pct_per_hr, float act_thr_mv)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate register values
    uint8_t hib_thr = (uint8_t)(hib_thr_pct_per_hr / MAX17048_HIBRT_HIB_THR_STEP_PCTH);
    uint8_t act_thr = (uint8_t)(act_thr_mv / MAX17048_HIBRT_ACT_THR_STEP_MV);
    
    // Create HIBRT register value
    max17048_reg_t hibrt_val = ((uint16_t)hib_thr << MAX17048_HIBRT_HIB_THR_SHIFT) | act_thr;
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.hibernate_threshold_pct_per_hr = hib_thr_pct_per_hr;
    dev->config.active_threshold_mv = act_thr_mv;
    dev->config.enable_hibernate = true;
    
    // Write to HIBRT register
    return max17048_write_reg(handle, MAX17048_HIBRT_REG, hibrt_val);
}

esp_err_t max17048_disable_hibernate(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.enable_hibernate = false;
    
    // Write disable value (0) to HIBRT register
    return max17048_write_reg(handle, MAX17048_HIBRT_REG, MAX17048_HIBRT_DISABLE);
}

esp_err_t max17048_force_hibernate(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Force hibernate by setting both thresholds to maximum values
    return max17048_write_reg(handle, MAX17048_HIBRT_REG, MAX17048_HIBRT_ALWAYS_HIBERNATE);
}

esp_err_t max17048_get_hibernate_config(max17048_handle_t handle, float *hib_thr_pct_per_hr, float *act_thr_mv)
{
    if (handle == NULL || hib_thr_pct_per_hr == NULL || act_thr_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t hibrt_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_HIBRT_REG, &hibrt_raw);
    if (ret == ESP_OK) {
        uint8_t hib_thr = (hibrt_raw & MAX17048_HIBRT_HIB_THR_MASK) >> MAX17048_HIBRT_HIB_THR_SHIFT;
        uint8_t act_thr = hibrt_raw & MAX17048_HIBRT_ACT_THR_MASK;
        
        *hib_thr_pct_per_hr = (float)hib_thr * MAX17048_HIBRT_HIB_THR_STEP_PCTH;
        *act_thr_mv = (float)act_thr * MAX17048_HIBRT_ACT_THR_STEP_MV;
    }
    
    return ret;
}

esp_err_t max17048_get_hibrt_raw(max17048_handle_t handle, max17048_reg_t *hibrt_raw)
{
    if (handle == NULL || hibrt_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_HIBRT_REG, hibrt_raw);
}

/* Config Register Functions */

esp_err_t max17048_set_rcomp(max17048_handle_t handle, uint8_t rcomp)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current CONFIG value
    max17048_reg_t config_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CONFIG_REG, &config_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Update RCOMP value (top byte)
    config_raw = (config_raw & ~MAX17048_CONFIG_RCOMP_MASK) | ((uint16_t)rcomp << MAX17048_CONFIG_RCOMP_SHIFT);
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.rcomp = rcomp;
    
    // Write updated CONFIG value
    return max17048_write_reg(handle, MAX17048_CONFIG_REG, config_raw);
}

esp_err_t max17048_set_empty_alert_threshold(max17048_handle_t handle, uint8_t percent)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check for valid range (1-32%)
    if (percent < 1 || percent > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current CONFIG value
    max17048_reg_t config_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CONFIG_REG, &config_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert percentage to ATHD value
    uint8_t athd = MAX17048_PERCENT_TO_ATHD(percent);
    
    // Update ATHD value (bottom 5 bits)
    config_raw = (config_raw & ~MAX17048_CONFIG_ATHD_MASK) | athd;
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.alert_threshold = percent;
    
    // Write updated CONFIG value
    return max17048_write_reg(handle, MAX17048_CONFIG_REG, config_raw);
}

esp_err_t max17048_set_soc_change_alert(max17048_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current CONFIG value
    max17048_reg_t config_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CONFIG_REG, &config_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Update ALSC bit
    if (enable) {
        config_raw |= MAX17048_CONFIG_ALSC_BIT;
    } else {
        config_raw &= ~MAX17048_CONFIG_ALSC_BIT;
    }
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.enable_soc_change_alert = enable;
    
    // Write updated CONFIG value
    return max17048_write_reg(handle, MAX17048_CONFIG_REG, config_raw);
}

esp_err_t max17048_clear_alert(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current CONFIG value
    max17048_reg_t config_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CONFIG_REG, &config_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Clear ALRT bit
    config_raw &= ~MAX17048_CONFIG_ALRT_BIT;
    
    // Write updated CONFIG value
    return max17048_write_reg(handle, MAX17048_CONFIG_REG, config_raw);
}

esp_err_t max17048_config_sleep(max17048_handle_t handle, bool sleep)
{
    return max17048_set_sleep_mode(handle, sleep);
}

esp_err_t max17048_get_config(max17048_handle_t handle, uint8_t *rcomp, bool *sleep, bool *alsc, 
                            bool *alert, uint8_t *empty_alert_threshold)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t config_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_CONFIG_REG, &config_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse CONFIG register fields
    if (rcomp != NULL) {
        *rcomp = (config_raw & MAX17048_CONFIG_RCOMP_MASK) >> MAX17048_CONFIG_RCOMP_SHIFT;
    }
    
    if (sleep != NULL) {
        *sleep = (config_raw & MAX17048_CONFIG_SLEEP_BIT) ? true : false;
    }
    
    if (alsc != NULL) {
        *alsc = (config_raw & MAX17048_CONFIG_ALSC_BIT) ? true : false;
    }
    
    if (alert != NULL) {
        *alert = (config_raw & MAX17048_CONFIG_ALRT_BIT) ? true : false;
    }
    
    if (empty_alert_threshold != NULL) {
        uint8_t athd = config_raw & MAX17048_CONFIG_ATHD_MASK;
        *empty_alert_threshold = MAX17048_ATHD_TO_PERCENT(athd);
    }
    
    return ESP_OK;
}

esp_err_t max17048_get_config_raw(max17048_handle_t handle, max17048_reg_t *config_raw)
{
    if (handle == NULL || config_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_CONFIG_REG, config_raw);
}

/* Voltage Alert Functions */

esp_err_t max17048_set_voltage_alerts(max17048_handle_t handle, float max_mv, float min_mv)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check for valid range
    if (max_mv < 0 || max_mv > 5100 || min_mv < 0 || min_mv > 5100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert to register values
    uint8_t max_val = MAX17048_MV_TO_VALRT(max_mv);
    uint8_t min_val = MAX17048_MV_TO_VALRT(min_mv);
    
    // Create VALRT register value
    max17048_reg_t valrt_val = ((uint16_t)max_val << MAX17048_VALRT_MAX_SHIFT) | min_val;
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.voltage_max_mv = max_mv;
    dev->config.voltage_min_mv = min_mv;
    dev->config.enable_voltage_alerts = true;
    
    // Write to VALRT register
    return max17048_write_reg(handle, MAX17048_VALRT_REG, valrt_val);
}

esp_err_t max17048_disable_voltage_alerts(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.enable_voltage_alerts = false;
    
    // To disable alerts, set minimum to 0 and maximum to 0xFF
    return max17048_write_reg(handle, MAX17048_VALRT_REG, MAX17048_VALRT_DEFAULT);
}

esp_err_t max17048_get_voltage_alerts(max17048_handle_t handle, float *max_mv, float *min_mv)
{
    if (handle == NULL || max_mv == NULL || min_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t valrt_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_VALRT_REG, &valrt_raw);
    if (ret == ESP_OK) {
        uint8_t max_val = (valrt_raw & MAX17048_VALRT_MAX_MASK) >> MAX17048_VALRT_MAX_SHIFT;
        uint8_t min_val = valrt_raw & MAX17048_VALRT_MIN_MASK;
        
        *max_mv = MAX17048_VALRT_TO_MV(max_val);
        *min_mv = MAX17048_VALRT_TO_MV(min_val);
    }
    
    return ret;
}

esp_err_t max17048_get_valrt_raw(max17048_handle_t handle, max17048_reg_t *valrt_raw)
{
    if (handle == NULL || valrt_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_VALRT_REG, valrt_raw);
}

/* VRESET Functions */

esp_err_t max17048_configure_vreset(max17048_handle_t handle, float reset_threshold_mv, bool disable_comparator)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check for valid range
    if (reset_threshold_mv < 0 || reset_threshold_mv > 5100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current register value to preserve ID field
    max17048_reg_t vreset_id_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_VRESET_ID_REG, &vreset_id_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert threshold to register value
    uint8_t vreset_val = MAX17048_MV_TO_VRESET(reset_threshold_mv);
    
    // Create VRESET register value, preserving ID field
    max17048_reg_t new_val = (vreset_id_raw & MAX17048_ID_MASK);
    new_val |= (uint16_t)vreset_val << 9;  // VRESET[7:1] is in bits 15:9
    
    // Set comparator disable bit if requested
    if (disable_comparator) {
        new_val |= MAX17048_VRESET_DIS_BIT;
    }
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.vreset_threshold_mv = reset_threshold_mv;
    dev->config.disable_comparator = disable_comparator;
    
    // Write to VRESET/ID register
    return max17048_write_reg(handle, MAX17048_VRESET_ID_REG, new_val);
}

esp_err_t max17048_set_vreset_captive(max17048_handle_t handle, bool disable_comparator)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For captive batteries, set VRESET to 2.5V (0x3E = 62 * 40mV = 2480mV)
    return max17048_configure_vreset(handle, 2480, disable_comparator);
}

esp_err_t max17048_set_vreset_removable(max17048_handle_t handle, float empty_voltage_mv, bool disable_comparator)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For removable batteries, set VRESET to (empty_voltage - 300mV)
    float reset_threshold_mv = empty_voltage_mv - MAX17048_VRESET_MIN_REMOVABLE_MV;
    
    // Ensure minimum reset threshold
    if (reset_threshold_mv < 0) {
        reset_threshold_mv = 0;
    }
    
    return max17048_configure_vreset(handle, reset_threshold_mv, disable_comparator);
}

esp_err_t max17048_get_id(max17048_handle_t handle, uint8_t *id)
{
    if (handle == NULL || id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t vreset_id_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_VRESET_ID_REG, &vreset_id_raw);
    if (ret == ESP_OK) {
        *id = vreset_id_raw & MAX17048_ID_MASK;
    }
    
    return ret;
}

esp_err_t max17048_get_vreset_config(max17048_handle_t handle, float *reset_threshold_mv, bool *comp_disabled)
{
    if (handle == NULL || reset_threshold_mv == NULL || comp_disabled == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t vreset_id_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_VRESET_ID_REG, &vreset_id_raw);
    if (ret == ESP_OK) {
        uint8_t vreset_val = (vreset_id_raw & MAX17048_VRESET_MASK) >> 9;
        *reset_threshold_mv = MAX17048_VRESET_TO_MV(vreset_val);
        *comp_disabled = (vreset_id_raw & MAX17048_VRESET_DIS_BIT) ? true : false;
    }
    
    return ret;
}

esp_err_t max17048_get_vreset_id_raw(max17048_handle_t handle, max17048_reg_t *vreset_id_raw)
{
    if (handle == NULL || vreset_id_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_VRESET_ID_REG, vreset_id_raw);
}

/* Status Register Functions */

esp_err_t max17048_get_status(max17048_handle_t handle, max17048_status_t *status)
{
    if (handle == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    max17048_reg_t status_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_STATUS_REG, &status_raw);
    if (ret == ESP_OK) {
        // Parse status register bits
        status->reset_indicator = (status_raw & MAX17048_STATUS_RI_BIT) ? true : false;
        status->voltage_high = (status_raw & MAX17048_STATUS_VH_BIT) ? true : false;
        status->voltage_low = (status_raw & MAX17048_STATUS_VL_BIT) ? true : false;
        status->voltage_reset = (status_raw & MAX17048_STATUS_VR_BIT) ? true : false;
        status->soc_low = (status_raw & MAX17048_STATUS_HD_BIT) ? true : false;
        status->soc_change = (status_raw & MAX17048_STATUS_SC_BIT) ? true : false;
        status->voltage_reset_en = (status_raw & MAX17048_STATUS_ENVR_BIT) ? true : false;
    }
    
    return ret;
}

esp_err_t max17048_clear_status_flags(max17048_handle_t handle, uint16_t flags)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current STATUS value
    max17048_reg_t status_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_STATUS_REG, &status_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Clear specified flags (write 0 to clear)
    status_raw &= ~flags;
    
    // Write updated STATUS value
    return max17048_write_reg(handle, MAX17048_STATUS_REG, status_raw);
}

esp_err_t max17048_set_voltage_reset_alert(max17048_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current STATUS value
    max17048_reg_t status_raw;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_STATUS_REG, &status_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Update EnVr bit
    if (enable) {
        status_raw |= MAX17048_STATUS_ENVR_BIT;
    } else {
        status_raw &= ~MAX17048_STATUS_ENVR_BIT;
    }
    
    // Update device config
    max17048_dev_t *dev = (max17048_dev_t *)handle;
    dev->config.enable_vreset_alert = enable;
    
    // Write updated STATUS value
    return max17048_write_reg(handle, MAX17048_STATUS_REG, status_raw);
}

esp_err_t max17048_get_status_raw(max17048_handle_t handle, max17048_reg_t *status_raw)
{
    if (handle == NULL || status_raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_read_reg(handle, MAX17048_STATUS_REG, status_raw);
}

/* Model Table Functions */

esp_err_t max17048_unlock_table(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Write unlock values to the unlock registers
    esp_err_t ret = max17048_write_reg(handle, MAX17048_TABLE_UNLOCK_1_REG, MAX17048_TABLE_UNLOCK_1_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return max17048_write_reg(handle, MAX17048_TABLE_UNLOCK_2_REG, MAX17048_TABLE_UNLOCK_2_VALUE);
}

esp_err_t max17048_lock_table(max17048_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Write lock value to the unlock registers
    esp_err_t ret = max17048_write_reg(handle, MAX17048_TABLE_UNLOCK_1_REG, MAX17048_TABLE_LOCK_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return max17048_write_reg(handle, MAX17048_TABLE_UNLOCK_2_REG, MAX17048_TABLE_LOCK_VALUE);
}

esp_err_t max17048_write_custom_model(max17048_handle_t handle, const uint16_t *model_data, uint8_t length)
{
    if (handle == NULL || model_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check for valid model data length
    if (length > (MAX17048_TABLE_END_REG - MAX17048_TABLE_START_REG + 1)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // First unlock the model table
    esp_err_t ret = max17048_unlock_table(handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write model data to table registers
    for (uint8_t i = 0; i < length; i++) {
        ret = max17048_write_reg(handle, MAX17048_TABLE_START_REG + i, model_data[i]);
        if (ret != ESP_OK) {
            // Lock the table before returning on error
            max17048_lock_table(handle);
            return ret;
        }
    }
    
    // Lock the table when done
    return max17048_lock_table(handle);
}

esp_err_t max17048_is_table_unlocked(max17048_handle_t handle, bool *is_unlocked)
{
    if (handle == NULL || is_unlocked == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read first unlock register - if it's not zero, the table is unlocked
    max17048_reg_t unlock1_val;
    esp_err_t ret = max17048_read_reg(handle, MAX17048_TABLE_UNLOCK_1_REG, &unlock1_val);
    if (ret == ESP_OK) {
        *is_unlocked = (unlock1_val != MAX17048_TABLE_LOCK_VALUE);
    }
    
    return ret;
}

/* CMD Register Functions */

esp_err_t max17048_write_cmd(max17048_handle_t handle, max17048_reg_t cmd_value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return max17048_write_reg(handle, MAX17048_CMD_REG, cmd_value);
}
