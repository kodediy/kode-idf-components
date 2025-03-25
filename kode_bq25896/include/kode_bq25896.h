#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Default I2C address for BQ25896
 */
#define BQ25896_I2C_ADDR                    0x6B

/**
 * @brief BQ25896 Register Addresses
 */
typedef enum {
    BQ25896_REG_00                  = 0x00, // Input Source Control
    BQ25896_REG_01                  = 0x01, // Power-On Configuration
    BQ25896_REG_02                  = 0x02, // Charge Current Control
    BQ25896_REG_03                  = 0x03, // Charge Control
    BQ25896_REG_04                  = 0x04, // Fast Charge Current Control
    BQ25896_REG_05                  = 0x05, // Pre-Charge/Termination Current Control
    BQ25896_REG_06                  = 0x06, // Charge Voltage Control
    BQ25896_REG_07                  = 0x07, // Charge Termination/Timer Control
    BQ25896_REG_08                  = 0x08, // IR Compensation/Thermal Regulation Control
    BQ25896_REG_09                  = 0x09, // Misc Operation Control
    BQ25896_REG_0A                  = 0x0A, // Boost Mode Control
    BQ25896_REG_0B                  = 0x0B, // Status Register
    BQ25896_REG_0C                  = 0x0C, // Fault Register
    BQ25896_REG_0D                  = 0x0D, // VINDPM Register
    BQ25896_REG_0E                  = 0x0E, // BATV Register
    BQ25896_REG_0F                  = 0x0F, // SYSV Register
    BQ25896_REG_10                  = 0x10, // TS Voltage Register
    BQ25896_REG_11                  = 0x11, // VBUS Voltage Register
    BQ25896_REG_12                  = 0x12, // Charge Current Register
    BQ25896_REG_13                  = 0x13, // Input Current Limit Register
    BQ25896_REG_14                  = 0x14, // Device Information Register
} bq25896_reg_t;

/**
 * @brief Charging status values (REG0B bits 4-3)
 */
typedef enum {
    BQ25896_CHG_STATUS_NOT_CHARGING = 0, // 00 - Not Charging
    BQ25896_CHG_STATUS_PRE_CHARGE   = 1, // 01 - Pre-charge (< VBATLOWV)
    BQ25896_CHG_STATUS_FAST_CHARGE  = 2, // 10 - Fast Charging
    BQ25896_CHG_STATUS_DONE         = 3, // 11 - Charge Termination Done
} bq25896_chg_status_t;

/**
 * @brief VBUS status values (REG0B bits 7-5)
 */
typedef enum {
    BQ25896_VBUS_STATUS_NO_INPUT     = 0, // 000 - No Input
    BQ25896_VBUS_STATUS_USB_SDP      = 1, // 001 - USB Host SDP
    BQ25896_VBUS_STATUS_ADAPTER      = 2, // 010 - Adapter (3.25A)
    BQ25896_VBUS_STATUS_OTG          = 7, // 111 - OTG
} bq25896_vbus_status_t;

/**
 * @brief Charge termination setting values
 */
typedef enum {
    BQ25896_CHG_TERM_DISABLE = 0,
    BQ25896_CHG_TERM_ENABLE  = 1,
} bq25896_chg_term_t;

/**
 * @brief Input current limit values
 */
typedef enum {
    BQ25896_ILIM_100MA  = 0,
    BQ25896_ILIM_150MA  = 1,
    BQ25896_ILIM_500MA  = 2,
    BQ25896_ILIM_900MA  = 3,
    BQ25896_ILIM_1200MA = 4,
    BQ25896_ILIM_1500MA = 5,
    BQ25896_ILIM_2000MA = 6,
    BQ25896_ILIM_3000MA = 7,
} bq25896_ilim_t;

/**
 * @brief Battery system status information
 */
typedef struct {
    bq25896_vbus_status_t vbus_status;
    bq25896_chg_status_t chg_status;
    bool power_good;
    bool in_vindpm_state;
    bool in_iindpm_state;
    bool vbat_gtr_vreg;  // VBAT > VREG?
    bool thermal_regulation;
    bool vsys_below_min; // VSYS < VSYSMIN
} bq25896_status_t;

/**
 * @brief Battery and charger information
 */
typedef struct {
    uint16_t vbat_mv;    // Battery voltage in mV
    uint16_t vsys_mv;    // System voltage in mV
    uint16_t vbus_mv;    // VBUS voltage in mV
    uint16_t ichg_ma;    // Charge current in mA
    int16_t  die_temp_c; // Temperature in Celsius
} bq25896_readings_t;

/**
 * @brief BQ25896 charger configuration
 */
typedef struct {
    // Input Current Limit
    bq25896_ilim_t input_current_limit;
    
    // Input Voltage Limit (VINDPM) in mV (3.9V - 14V)
    uint16_t input_voltage_limit_mv;
    
    // Enable HIZ mode (High Impedance mode)
    bool enable_hiz;
    
    // Charge settings
    bool enable_charging;
    bool enable_otg;
    bool enable_termination;
    
    // Fast Charge Current in mA (0-5056mA)
    uint16_t charge_current_ma;
    
    // Pre-charge Current in mA (64-1024mA)
    uint16_t pre_charge_current_ma;
    
    // Termination Current in mA (64-1024mA)
    uint16_t termination_current_ma;
    
    // Charge Voltage Limit in mV (3.840V - 4.608V)
    uint16_t charge_voltage_mv;
    
    // Min System Voltage in mV (3.0V - 3.7V)
    uint16_t min_system_voltage_mv;
    
    // Set Thermal Regulation Threshold in Celsius (60-120°C)
    uint8_t thermal_regulation_threshold_c;
} bq25896_config_t;

/**
 * @brief BQ25896 device handle
 */
typedef struct bq25896_dev_t *bq25896_handle_t;

/**
 * @brief Default configuration for BQ25896
 */
#define BQ25896_DEFAULT_CONFIG() { \
    .input_current_limit = BQ25896_ILIM_500MA, \
    .input_voltage_limit_mv = 4500, \
    .enable_hiz = false, \
    .enable_charging = true, \
    .enable_otg = false, \
    .enable_termination = true, \
    .charge_current_ma = 1000, \
    .pre_charge_current_ma = 200, \
    .termination_current_ma = 200, \
    .charge_voltage_mv = 4200, \
    .min_system_voltage_mv = 3400, \
    .thermal_regulation_threshold_c = 80, \
}

/**
 * @brief Initialize a new BQ25896 device
 * 
 * @param i2c_bus The I2C bus handle
 * @param dev_addr The device I2C address (default BQ25896_I2C_ADDR)
 * @param[out] handle Pointer to store the device handle
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_init(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr, bq25896_handle_t *handle);

/**
 * @brief Delete a BQ25896 device instance
 * 
 * @param handle The BQ25896 device handle
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_delete(bq25896_handle_t handle);

/**
 * @brief Configure the BQ25896 with the specified settings
 * 
 * @param handle The BQ25896 device handle
 * @param config Configuration parameters
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_configure(bq25896_handle_t handle, const bq25896_config_t *config);

/**
 * @brief Get the current status information from the BQ25896
 * 
 * @param handle The BQ25896 device handle
 * @param[out] status Pointer to store the status information
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_get_status(bq25896_handle_t handle, bq25896_status_t *status);

/**
 * @brief Get the current readings from the BQ25896
 * 
 * @param handle The BQ25896 device handle
 * @param[out] readings Pointer to store the readings
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_get_readings(bq25896_handle_t handle, bq25896_readings_t *readings);

/**
 * @brief Enable or disable charging
 * 
 * @param handle The BQ25896 device handle
 * @param enable True to enable, false to disable
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_enable_charging(bq25896_handle_t handle, bool enable);

/**
 * @brief Enable or disable OTG (On-The-Go) mode
 * 
 * @param handle The BQ25896 device handle
 * @param enable True to enable, false to disable
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_enable_otg(bq25896_handle_t handle, bool enable);

/**
 * @brief Reset the BQ25896 to default register values
 * 
 * @param handle The BQ25896 device handle
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_reset(bq25896_handle_t handle);

/**
 * @brief Enable or disable High-Z (High Impedance) mode
 * 
 * @param handle The BQ25896 device handle
 * @param enable True to enable, false to disable
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_enable_hiz(bq25896_handle_t handle, bool enable);

/**
 * @brief Set the charging current in mA
 * 
 * @param handle The BQ25896 device handle
 * @param current_ma Current in mA (0-5056mA)
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_set_charge_current(bq25896_handle_t handle, uint16_t current_ma);

/**
 * @brief Set the charging voltage in mV
 * 
 * @param handle The BQ25896 device handle
 * @param voltage_mv Voltage in mV (3840-4608mV)
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_set_charge_voltage(bq25896_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Set the input current limit
 * 
 * @param handle The BQ25896 device handle
 * @param ilim Input current limit setting
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_set_input_current_limit(bq25896_handle_t handle, bq25896_ilim_t ilim);

/**
 * @brief Set the input voltage limit in mV
 * 
 * @param handle The BQ25896 device handle
 * @param voltage_mv Voltage in mV (3900-14000mV)
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_set_input_voltage_limit(bq25896_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Enter shipping mode by disabling BATFET
 * 
 * This disconnects the battery from the system (VSYS), useful for storage/shipping.
 * It works by setting the BATFET_DIS bit in REG09 (bit 5).
 * Note: Once enabled, the device will power off immediately until power is removed
 * and reapplied or the device is reset.
 * 
 * @param handle The BQ25896 device handle
 * @return esp_err_t ESP_OK on success, otherwise error code
 */
esp_err_t bq25896_enter_shipping_mode(bq25896_handle_t handle);

/**
 * @brief BQ25896 temperature reading constants
 */
#define BQ25896_TEMP_INVALID (-32768) ///< Indicates no valid temperature reading

#ifdef __cplusplus
}
#endif
