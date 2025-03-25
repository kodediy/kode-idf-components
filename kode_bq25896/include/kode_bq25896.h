#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Default I2C address for BQ25896 (7-bit)
 */
#define BQ25896_I2C_ADDR_DEFAULT  0x6A

/**
 * @brief Input Current Limit options (in mA) for REG00 [5:0]
 * Range: 100mA to 3250mA with 50mA resolution
 */
typedef enum {
    BQ25896_ILIM_100MA  = 0x00,  // 100mA
    BQ25896_ILIM_150MA  = 0x01,  // 150mA
    BQ25896_ILIM_200MA  = 0x02,  // 200mA
    BQ25896_ILIM_250MA  = 0x03,  // 250mA
    BQ25896_ILIM_300MA  = 0x04,  // 300mA
    BQ25896_ILIM_350MA  = 0x05,  // 350mA
    BQ25896_ILIM_400MA  = 0x06,  // 400mA
    BQ25896_ILIM_450MA  = 0x07,  // 450mA
    BQ25896_ILIM_500MA  = 0x08,  // 500mA (default)
    BQ25896_ILIM_600MA  = 0x0A,  // 600mA
    BQ25896_ILIM_700MA  = 0x0C,  // 700mA
    BQ25896_ILIM_800MA  = 0x0E,  // 800mA
    BQ25896_ILIM_900MA  = 0x10,  // 900mA
    BQ25896_ILIM_1000MA = 0x12,  // 1000mA
    BQ25896_ILIM_1200MA = 0x14,  // 1200mA
    BQ25896_ILIM_1400MA = 0x16,  // 1400mA
    BQ25896_ILIM_1600MA = 0x18,  // 1600mA
    BQ25896_ILIM_1800MA = 0x1A,  // 1800mA
    BQ25896_ILIM_2000MA = 0x1C,  // 2000mA
    BQ25896_ILIM_2200MA = 0x1E,  // 2200mA
    BQ25896_ILIM_2400MA = 0x20,  // 2400mA
    BQ25896_ILIM_2600MA = 0x22,  // 2600mA
    BQ25896_ILIM_2800MA = 0x24,  // 2800mA
    BQ25896_ILIM_3000MA = 0x26,  // 3000mA
    BQ25896_ILIM_3250MA = 0x3F,  // 3250mA (maximum)
} bq25896_ilim_t;

/**
 * @brief Boost Mode Hot Temperature Monitor Threshold settings (REG01 [7:6])
 */
typedef enum {
    BQ25896_BHOT_THRESHOLD1 = 0x00,  // VBHOT1 Threshold (34.75%) (default)
    BQ25896_BHOT_THRESHOLD0 = 0x01,  // VBHOT0 Threshold (37.75%)
    BQ25896_BHOT_THRESHOLD2 = 0x02,  // VBHOT2 Threshold (31.25%)
    BQ25896_BHOT_DISABLED   = 0x03,  // Disable boost mode thermal protection
} bq25896_bhot_t;

/**
 * @brief Boost Mode Cold Temperature Monitor Threshold settings (REG01 [5])
 */
typedef enum {
    BQ25896_BCOLD_THRESHOLD0 = 0x00,  // VBCOLD0 Threshold (77%) (default)
    BQ25896_BCOLD_THRESHOLD1 = 0x01,  // VBCOLD1 Threshold (80%)
} bq25896_bcold_t;

/**
 * @brief ADC Conversion Rate settings (REG02 [6])
 */
typedef enum {
    BQ25896_ADC_CONV_RATE_ONESHOT = 0x00,  // One shot conversion (default)
    BQ25896_ADC_CONV_RATE_CONTINUOUS = 0x01,  // Continuous conversion
} bq25896_adc_conv_rate_t;

/**
 * @brief Boost Mode Frequency settings (REG02 [5])
 */
typedef enum {
    BQ25896_BOOST_FREQ_1500KHZ = 0x00,  // 1.5MHz (default)
    BQ25896_BOOST_FREQ_500KHZ  = 0x01,  // 500KHz
} bq25896_boost_freq_t;

/**
 * @brief Minimum System Voltage settings (REG03 [3:1])
 */
typedef enum {
    BQ25896_SYS_MIN_3000MV = 0x00,  // 3.0V
    BQ25896_SYS_MIN_3100MV = 0x01,  // 3.1V
    BQ25896_SYS_MIN_3200MV = 0x02,  // 3.2V
    BQ25896_SYS_MIN_3300MV = 0x03,  // 3.3V
    BQ25896_SYS_MIN_3400MV = 0x04,  // 3.4V
    BQ25896_SYS_MIN_3500MV = 0x05,  // 3.5V (default)
    BQ25896_SYS_MIN_3600MV = 0x06,  // 3.6V
    BQ25896_SYS_MIN_3700MV = 0x07,  // 3.7V
} bq25896_sys_min_t;

/**
 * @brief Battery Precharge to Fast Charge Threshold settings (REG06 [1])
 */
typedef enum {
    BQ25896_BATLOWV_2800MV = 0x00,  // 2.8V
    BQ25896_BATLOWV_3000MV = 0x01,  // 3.0V (default)
} bq25896_batlowv_t;

/**
 * @brief Battery Recharge Threshold Offset settings (REG06 [0])
 */
typedef enum {
    BQ25896_VRECHG_100MV = 0x00,  // 100mV below VREG (default)
    BQ25896_VRECHG_200MV = 0x01,  // 200mV below VREG
} bq25896_vrechg_t;

/**
 * @brief I2C Watchdog Timer settings (REG07 [5:4])
 */
typedef enum {
    BQ25896_WATCHDOG_DISABLE = 0x00,  // Disable watchdog timer
    BQ25896_WATCHDOG_40S     = 0x01,  // 40s (default)
    BQ25896_WATCHDOG_80S     = 0x02,  // 80s
    BQ25896_WATCHDOG_160S    = 0x03,  // 160s
} bq25896_watchdog_t;

/**
 * @brief Fast Charge Timer settings (REG07 [2:1])
 */
typedef enum {
    BQ25896_CHG_TIMER_5H  = 0x00,  // 5 hours
    BQ25896_CHG_TIMER_8H  = 0x01,  // 8 hours
    BQ25896_CHG_TIMER_12H = 0x02,  // 12 hours (default)
    BQ25896_CHG_TIMER_20H = 0x03,  // 20 hours
} bq25896_chg_timer_t;

/**
 * @brief JEITA Low Temperature Current Setting (REG07 [0])
 */
typedef enum {
    BQ25896_JEITA_ISET_50PCT = 0x00,  // 50% of fast charge current
    BQ25896_JEITA_ISET_20PCT = 0x01,  // 20% of fast charge current (default)
} bq25896_jeita_iset_t;

/**
 * @brief Thermal Regulation Threshold settings (REG08 [1:0])
 */
typedef enum {
    BQ25896_TREG_60C  = 0x00,  // 60°C
    BQ25896_TREG_80C  = 0x01,  // 80°C
    BQ25896_TREG_100C = 0x02,  // 100°C
    BQ25896_TREG_120C = 0x03,  // 120°C (default)
} bq25896_treg_t;

/**
 * @brief JEITA High Temperature Voltage Setting (REG09 [4])
 */
typedef enum {
    BQ25896_JEITA_VSET_VREG_MINUS_200MV = 0x00,  // VREG-200mV during high temp (default)
    BQ25896_JEITA_VSET_VREG             = 0x01,  // VREG during high temp
} bq25896_jeita_vset_t;

/**
 * @brief Boost Mode Current Limit settings (REG0A [2:0])
 */
typedef enum {
    BQ25896_BOOST_LIM_500MA  = 0x00,  // 0.5A
    BQ25896_BOOST_LIM_750MA  = 0x01,  // 0.75A
    BQ25896_BOOST_LIM_1200MA = 0x02,  // 1.2A
    BQ25896_BOOST_LIM_1400MA = 0x03,  // 1.4A (default)
    BQ25896_BOOST_LIM_1650MA = 0x04,  // 1.65A
    BQ25896_BOOST_LIM_1875MA = 0x05,  // 1.875A
    BQ25896_BOOST_LIM_2150MA = 0x06,  // 2.15A
    // 0x07 is reserved
} bq25896_boost_lim_t;

/**
 * @brief VBUS Status values (REG0B [7:5])
 */
typedef enum {
    BQ25896_VBUS_NO_INPUT    = 0x00,  // No input
    BQ25896_VBUS_USB_HOST    = 0x01,  // USB host SDP
    BQ25896_VBUS_ADAPTER     = 0x02,  // Adapter (3.25A)
    BQ25896_VBUS_OTG         = 0x03,  // OTG
} bq25896_vbus_stat_t;

/**
 * @brief Charging Status values (REG0B [4:3])
 */
typedef enum {
    BQ25896_CHRG_NOT_CHARGING   = 0x00,  // Not charging
    BQ25896_CHRG_PRE_CHARGE     = 0x01,  // Pre-charge (<VBATLOWV)
    BQ25896_CHRG_FAST_CHARGING  = 0x02,  // Fast charging
    BQ25896_CHRG_TERM_DONE      = 0x03,  // Charge termination done
} bq25896_chrg_stat_t;

/**
 * @brief Charge Fault Status values (REG0C [5:4])
 */
typedef enum {
    BQ25896_CHRG_FAULT_NORMAL        = 0x00,  // Normal
    BQ25896_CHRG_FAULT_INPUT         = 0x01,  // Input fault (VBUS > VACOV or VBAT < VBUS < VBUSMIN)
    BQ25896_CHRG_FAULT_THERMAL       = 0x02,  // Thermal shutdown
    BQ25896_CHRG_FAULT_TIMER_EXPIRED = 0x03,  // Charge safety timer expired
} bq25896_chrg_fault_t;

/**
 * @brief NTC Fault Status values (REG0C [2:0]) for Buck Mode
 */
typedef enum {
    BQ25896_NTC_FAULT_NORMAL   = 0x00,  // Normal
    BQ25896_NTC_FAULT_TS_WARM  = 0x02,  // TS Warm
    BQ25896_NTC_FAULT_TS_COOL  = 0x03,  // TS Cool
    BQ25896_NTC_FAULT_TS_COLD  = 0x05,  // TS Cold
    BQ25896_NTC_FAULT_TS_HOT   = 0x06,  // TS Hot
} bq25896_ntc_fault_t;

/**
 * @brief VINDPM Threshold Setting Method (REG0D [7])
 */
typedef enum {
    BQ25896_VINDPM_RELATIVE = 0x00,  // Use relative VINDPM threshold (default)
    BQ25896_VINDPM_ABSOLUTE = 0x01,  // Use absolute VINDPM threshold
} bq25896_vindpm_mode_t;

/**
 * @brief BQ25896 device handle
 */
typedef struct bq25896_dev_t *bq25896_handle_t;

/**
 * @brief Configuration structure for BQ25896
 * Contains configurable parameters for REG00, REG01, REG02, REG03, REG04, REG05, REG06, REG07, REG08, REG09, REG0A, and REG0D
 */
typedef struct {
    /* REG00 - Input Source Control */
    bool enable_hiz;               // Enable High-Impedance mode
    bool enable_ilim_pin;          // Enable ILIM Pin control
    bq25896_ilim_t input_current_limit; // Input current limit

    /* REG01 - Power-On Configuration */
    bq25896_bhot_t bhot_threshold; // Boost mode hot temperature threshold
    bq25896_bcold_t bcold_threshold; // Boost mode cold temperature threshold
    uint16_t vindpm_offset_mv;     // Input voltage limit offset (0-3100mV)
    
    /* REG02 - Charge Current Control */
    bq25896_adc_conv_rate_t adc_conv_rate;  // ADC conversion rate
    bq25896_boost_freq_t boost_frequency;   // Boost mode frequency
    bool enable_ico;               // Enable Input Current Optimizer
    bool auto_dpdm_detection;      // Enable automatic USB input source detection
    
    /* REG03 - Charge Control */
    bool enable_bat_load;          // Enable battery load
    bool enable_charging;          // Enable charging
    bool enable_otg;               // Enable OTG (boost) mode
    bq25896_sys_min_t sys_min_voltage; // Minimum system voltage
    bool min_vbat_sel;             // Minimum battery voltage selection (0=2.9V, 1=2.5V)
    
    /* REG04 - Fast Charge Current Control */
    bool enable_pumpx;             // Enable current pulse control (PUMPX) for charging
    uint16_t charge_current_ma;    // Charge current in mA (0-3008mA, 64mA step)
    
    /* REG05 - Pre-Charge/Termination Current Control */
    uint16_t prechg_current_ma;    // Precharge current in mA (64-1024mA, 64mA step)
    uint16_t term_current_ma;      // Termination current in mA (64-1024mA, 64mA step)
    
    /* REG06 - Charge Voltage Control */
    uint16_t charge_voltage_mv;    // Charge voltage in mV (3840-4608mV, 16mV step)
    bq25896_batlowv_t batlowv;     // Battery precharge to fast charge threshold
    bq25896_vrechg_t vrechg;       // Battery recharge threshold offset
    
    /* REG07 - Termination/Timer Control */
    bool enable_term;              // Enable charge termination
    bool disable_stat_pin;         // Disable STAT pin functionality
    bq25896_watchdog_t watchdog;   // I2C watchdog timer setting
    bool enable_safety_timer;      // Enable charging safety timer
    bq25896_chg_timer_t chg_timer; // Fast charge timer setting
    bq25896_jeita_iset_t jeita_iset; // JEITA low temperature current setting
    
    /* REG08 - IR Compensation/Thermal Regulation Control */
    uint8_t bat_comp_mohm;         // Battery compensation resistor (0-140mΩ, 20mΩ step)
    uint16_t vclamp_mv;            // IR compensation voltage clamp (0-224mV, 32mV step)
    bq25896_treg_t treg;           // Thermal regulation threshold
    
    /* REG09 - Operation Control */
    bool extend_safety_timer;      // Slow safety timer by 2x during input DPM or thermal regulation
    bool force_batfet_off;         // Force BATFET off to enable ship mode
    bool enable_batfet_delay;      // Delay BATFET turn off when BATFET_DIS is set
    bq25896_jeita_vset_t jeita_vset; // JEITA high temperature voltage setting
    bool enable_batfet_reset;      // Enable BATFET full system reset (auto-reset after 32s)
    
    /* REG0A - Boost Mode Control */
    uint16_t boost_voltage_mv;     // Boost mode voltage in mV (4550-5510mV, 64mV step)
    bool disable_pfm_otg;          // Disable PFM mode in boost (OTG) mode
    bq25896_boost_lim_t boost_current_limit; // Boost mode current limit
    
    /* REG0D - VINDPM/Input Voltage Limit */
    bq25896_vindpm_mode_t vindpm_mode; // VINDPM threshold setting method
    uint16_t vindpm_voltage_mv;    // Absolute VINDPM threshold in mV (3900-15300mV)
} bq25896_config_t;

/**
 * @brief Default configuration settings for BQ25896
 * These values provide a safe starting point for most applications
 */
extern const bq25896_config_t BQ25896_DEFAULT_CONFIG;

/**
 * @brief Initialize configuration structure with safe default values
 * This can be used as a starting point to customize the configuration
 * 
 * @param config Pointer to configuration structure to initialize
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_default_config(bq25896_config_t *config);

/* High-level charging operations */

/**
 * @brief Start charging with default or current configuration
 * Configures the device for battery charging mode
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_start_charging(bq25896_handle_t handle);

/**
 * @brief Stop charging
 * Disables charging but maintains other functions
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_stop_charging(bq25896_handle_t handle);

/**
 * @brief Enter OTG (boost) mode
 * Configures the device to supply power from battery to VBUS
 * 
 * @param handle Device handle
 * @param boost_voltage_mv OTG mode output voltage in mV (4550-5510mV)
 * @param current_limit Current limit option
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_start_otg_mode(bq25896_handle_t handle, uint16_t boost_voltage_mv, bq25896_boost_lim_t current_limit);

/**
 * @brief Exit OTG (boost) mode
 * Returns to normal (non-boost) operation
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_stop_otg_mode(bq25896_handle_t handle);

/**
 * @brief Enter shipping mode (low power state)
 * Disconnects the battery from the system by turning off BATFET
 * Note: Device will remain in this state until power is cycled or BATFET is re-enabled
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enter_ship_mode(bq25896_handle_t handle);

/**
 * @brief Initialize the BQ25896 device
 * 
 * @param i2c_bus I2C bus handle
 * @param dev_addr Device I2C address
 * @param handle Pointer to store the device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_init(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr, bq25896_handle_t *handle);

/**
 * @brief Delete the BQ25896 device instance
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_delete(bq25896_handle_t handle);

/**
 * @brief Reset the BQ25896 to default state
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_reset(bq25896_handle_t handle);

/**
 * @brief Configure the BQ25896 with the given settings
 * 
 * @param handle Device handle
 * @param config Configuration structure
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_configure(bq25896_handle_t handle, const bq25896_config_t *config);

/* REG00 Functions - Input Source Control */

/**
 * @brief Enable/disable High Impedance mode (HIZ)
 * When enabled, the input current from the input source is reduced to 0
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_hiz(bq25896_handle_t handle, bool enable);

/**
 * @brief Enable/disable ILIM pin current limit control
 * When enabled, the input current limit is determined by ILIM pin
 * When disabled, input current limit is set by I2C register
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_ilim_pin(bq25896_handle_t handle, bool enable);

/**
 * @brief Set input current limit
 * This setting only applies when EN_ILIM=0 (ILIM pin disabled)
 * The actual input current limit is the lower of I2C or ILIM pin
 * 
 * @param handle Device handle
 * @param ilim Input current limit value from enum
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_input_current_limit(bq25896_handle_t handle, bq25896_ilim_t ilim);

/**
 * @brief Set input current limit using a direct value in mA
 * Converts mA value to the closest appropriate register setting
 * 
 * @param handle Device handle
 * @param current_ma Current in mA (range 100-3250mA)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_input_current_limit_ma(bq25896_handle_t handle, uint16_t current_ma);

/**
 * @brief Get the current input source control settings
 * 
 * @param handle Device handle
 * @param hiz_enabled Pointer to store HIZ mode status
 * @param ilim_enabled Pointer to store ILIM pin status
 * @param input_limit Pointer to store input current limit setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_input_source_settings(bq25896_handle_t handle, bool *hiz_enabled, 
                                           bool *ilim_enabled, bq25896_ilim_t *input_limit);

/* REG01 Functions - Power-On Configuration */

/**
 * @brief Set boost mode hot temperature threshold
 * 
 * @param handle Device handle
 * @param threshold Hot temperature threshold setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_hot_threshold(bq25896_handle_t handle, bq25896_bhot_t threshold);

/**
 * @brief Set boost mode cold temperature threshold
 * 
 * @param handle Device handle
 * @param threshold Cold temperature threshold setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_cold_threshold(bq25896_handle_t handle, bq25896_bcold_t threshold);

/**
 * @brief Set input voltage limit offset
 * 
 * This offset is added to the measured VBUS when calculating VINDPM threshold
 * when FORCE_VINDPM = 0 (REG0D[7] = 0). When VBUS at no load is ≤ 6V, this offset
 * is used directly. When VBUS at no load is > 6V, this offset is multiplied by 2.
 * 
 * @param handle Device handle
 * @param offset_mv Offset in mV (0-3100mV in 100mV steps)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vindpm_offset(bq25896_handle_t handle, uint16_t offset_mv);

/**
 * @brief Get the current power-on configuration settings
 * 
 * @param handle Device handle
 * @param bhot Pointer to store boost hot threshold setting
 * @param bcold Pointer to store boost cold threshold setting
 * @param vindpm_offset_mv Pointer to store input voltage limit offset in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_power_on_config(bq25896_handle_t handle, bq25896_bhot_t *bhot,
                                     bq25896_bcold_t *bcold, uint16_t *vindpm_offset_mv);

/* REG02 Functions - Charge Current Control */

/**
 * @brief Start ADC conversion
 * Triggers a one-shot ADC conversion to measure various voltages and currents
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_start_adc_conversion(bq25896_handle_t handle);

/**
 * @brief Set ADC conversion rate
 * 
 * @param handle Device handle
 * @param rate ADC conversion rate (one-shot or continuous)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_adc_conversion_rate(bq25896_handle_t handle, bq25896_adc_conv_rate_t rate);

/**
 * @brief Set boost mode switching frequency
 * 
 * @param handle Device handle
 * @param freq Boost mode frequency (1.5MHz or 500KHz)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_frequency(bq25896_handle_t handle, bq25896_boost_freq_t freq);

/**
 * @brief Enable/disable Input Current Optimizer (ICO)
 * When enabled, the input current is automatically optimized to the maximum
 * available from the input source without pulling VBUS below VINDPM threshold
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_ico(bq25896_handle_t handle, bool enable);

/**
 * @brief Force input detection cycle
 * Triggers a new detection of the input source type (SDP, CDP, DCP)
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_force_input_detection(bq25896_handle_t handle);

/**
 * @brief Enable/disable automatic input detection
 * When enabled, automatically detects the input source type upon connection
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_auto_dpdm(bq25896_handle_t handle, bool enable);

/**
 * @brief Get current charge current control settings
 * 
 * @param handle Device handle
 * @param adc_rate Pointer to store ADC conversion rate
 * @param boost_freq Pointer to store boost frequency
 * @param ico_enabled Pointer to store ICO status
 * @param auto_dpdm_enabled Pointer to store automatic input detection status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_current_control(bq25896_handle_t handle, 
                                           bq25896_adc_conv_rate_t *adc_rate,
                                           bq25896_boost_freq_t *boost_freq,
                                           bool *ico_enabled,
                                           bool *auto_dpdm_enabled);

/* REG03 Functions - Charge Control */

/**
 * @brief Enable/disable battery load
 * When enabled, IBATLOAD current source from battery is enabled
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_battery_load(bq25896_handle_t handle, bool enable);

/**
 * @brief Reset I2C watchdog timer
 * Writing 1 to this bit resets the watchdog timer
 * This bit always reads 0
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_reset_watchdog(bq25896_handle_t handle);

/**
 * @brief Enable/disable OTG (boost) mode
 * When enabled, the IC acts as a boost converter to supply power from battery to VBUS
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_otg(bq25896_handle_t handle, bool enable);

/**
 * @brief Enable/disable charging
 * When enabled, the IC charges the battery when power is available
 * 
 * @param handle Device handle
 * @param enable true to enable (default), false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_charging(bq25896_handle_t handle, bool enable);

/**
 * @brief Set minimum system voltage
 * This sets the minimum voltage on VBUS during boost mode
 * 
 * @param handle Device handle
 * @param sys_min Minimum system voltage setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_sys_min_voltage(bq25896_handle_t handle, bq25896_sys_min_t sys_min);

/**
 * @brief Set minimum battery voltage selection to exit boost mode
 * 
 * @param handle Device handle
 * @param use_lower_threshold true to select 2.5V, false to select 2.9V (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_min_vbat_sel(bq25896_handle_t handle, bool use_lower_threshold);

/**
 * @brief Get current charge control settings
 * 
 * @param handle Device handle
 * @param bat_load_enabled Pointer to store battery load status
 * @param otg_enabled Pointer to store OTG mode status
 * @param charging_enabled Pointer to store charging status
 * @param sys_min Pointer to store minimum system voltage setting
 * @param min_vbat_sel Pointer to store minimum battery voltage selection
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_control(bq25896_handle_t handle, 
                                    bool *bat_load_enabled,
                                    bool *otg_enabled,
                                    bool *charging_enabled,
                                    bq25896_sys_min_t *sys_min,
                                    bool *min_vbat_sel);

/* REG04 Functions - Fast Charge Current Control */

/**
 * @brief Enable/disable current pulse control (PUMPX)
 * When enabled, allows PUMPX_UP and PUMPX_DN control in REG09
 * 
 * @param handle Device handle
 * @param enable true to enable, false to disable (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_pumpx(bq25896_handle_t handle, bool enable);

/**
 * @brief Set fast charge current
 * Sets the battery charge current during fast charge phase
 * Setting to 0 disables charging
 * 
 * @param handle Device handle
 * @param current_ma Current in mA (0-3008mA, 64mA step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_current(bq25896_handle_t handle, uint16_t current_ma);

/**
 * @brief Get current fast charge current settings
 * 
 * @param handle Device handle
 * @param pumpx_enabled Pointer to store current pulse control status
 * @param charge_current_ma Pointer to store fast charge current in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_fast_charge_current(bq25896_handle_t handle, 
                                        bool *pumpx_enabled, 
                                        uint16_t *charge_current_ma);

/* REG05 Functions - Pre-Charge/Termination Current Control */

/**
 * @brief Set precharge current
 * Sets the battery charge current during precharge phase (when battery voltage is below BATLOWV)
 * 
 * @param handle Device handle
 * @param current_ma Current in mA (64-1024mA, 64mA step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_precharge_current(bq25896_handle_t handle, uint16_t current_ma);

/**
 * @brief Set termination current
 * Sets the current threshold to terminate charging when charge current falls below this value
 * 
 * @param handle Device handle
 * @param current_ma Current in mA (64-1024mA, 64mA step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_termination_current(bq25896_handle_t handle, uint16_t current_ma);

/**
 * @brief Get precharge and termination current settings
 * 
 * @param handle Device handle
 * @param prechg_current_ma Pointer to store precharge current in mA
 * @param term_current_ma Pointer to store termination current in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_precharge_termination_current(bq25896_handle_t handle, 
                                                  uint16_t *prechg_current_ma, 
                                                  uint16_t *term_current_ma);

/* REG06 Functions - Charge Voltage Control */

/**
 * @brief Set battery charge voltage
 * Sets the target voltage during constant voltage phase of charging
 * Note: Values above 4.608V (VREG > 110000) are clamped to 4.608V
 * 
 * @param handle Device handle
 * @param voltage_mv Voltage in mV (3840-4608mV, 16mV step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_charge_voltage(bq25896_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Set battery precharge to fast charge threshold
 * Sets the voltage threshold for transitioning from precharge to fast charge mode
 * 
 * @param handle Device handle
 * @param threshold BATLOWV threshold setting (2.8V or 3.0V)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_batlowv_threshold(bq25896_handle_t handle, bq25896_batlowv_t threshold);

/**
 * @brief Set battery recharge threshold offset
 * Sets the voltage offset below charge voltage to trigger recharge
 * 
 * @param handle Device handle
 * @param threshold VRECHG offset setting (100mV or 200mV below VREG)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vrechg_threshold(bq25896_handle_t handle, bq25896_vrechg_t threshold);

/**
 * @brief Get charge voltage control settings
 * 
 * @param handle Device handle
 * @param charge_voltage_mv Pointer to store charge voltage in mV
 * @param batlowv Pointer to store precharge to fast charge threshold setting
 * @param vrechg Pointer to store recharge threshold offset setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_voltage_control(bq25896_handle_t handle, 
                                          uint16_t *charge_voltage_mv, 
                                          bq25896_batlowv_t *batlowv, 
                                          bq25896_vrechg_t *vrechg);

/* REG07 Functions - Termination/Timer Control */

/**
 * @brief Enable/disable charge termination
 * When enabled, charging is terminated when charge current falls below termination current threshold
 * 
 * @param handle Device handle
 * @param enable true to enable (default), false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_termination(bq25896_handle_t handle, bool enable);

/**
 * @brief Enable/disable STAT pin functionality
 * When disabled, the STAT pin is forced to high-impedance state
 * 
 * @param handle Device handle
 * @param disable true to disable, false to enable (default) STAT pin function
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_disable_stat_pin(bq25896_handle_t handle, bool disable);

/**
 * @brief Set I2C watchdog timer
 * Sets the I2C watchdog timer to reset the IC to default settings if I2C is inactive
 * 
 * @param handle Device handle
 * @param watchdog Watchdog timer setting (disable, 40s, 80s, 160s)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_watchdog_timer(bq25896_handle_t handle, bq25896_watchdog_t watchdog);

/**
 * @brief Enable/disable charging safety timer
 * When enabled, charging is terminated after the fast charge timer expires
 * 
 * @param handle Device handle
 * @param enable true to enable (default), false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_safety_timer(bq25896_handle_t handle, bool enable);

/**
 * @brief Set fast charge timer duration
 * Sets the maximum time allowed for fast charge phase
 * 
 * @param handle Device handle
 * @param timer Fast charge timer setting (5h, 8h, 12h, 20h)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_fast_charge_timer(bq25896_handle_t handle, bq25896_chg_timer_t timer);

/**
 * @brief Set JEITA low temperature current setting
 * Sets the percentage of fast charge current to use in JEITA low temperature region
 * 
 * @param handle Device handle
 * @param iset JEITA current setting (20% or 50% of ICHG)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_jeita_low_temp_current(bq25896_handle_t handle, bq25896_jeita_iset_t iset);

/**
 * @brief Get termination/timer control settings
 * 
 * @param handle Device handle
 * @param term_enabled Pointer to store termination enable status
 * @param stat_disabled Pointer to store STAT pin disable status
 * @param watchdog Pointer to store watchdog timer setting
 * @param safety_timer_enabled Pointer to store safety timer enable status
 * @param chg_timer Pointer to store fast charge timer setting
 * @param jeita_iset Pointer to store JEITA low temperature current setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_termination_timer_control(bq25896_handle_t handle,
                                              bool *term_enabled,
                                              bool *stat_disabled,
                                              bq25896_watchdog_t *watchdog,
                                              bool *safety_timer_enabled,
                                              bq25896_chg_timer_t *chg_timer,
                                              bq25896_jeita_iset_t *jeita_iset);

/* REG08 Functions - IR Compensation/Thermal Regulation Control */

/**
 * @brief Set battery compensation resistor
 * Sets the resistor value used for IR compensation
 * 
 * @param handle Device handle
 * @param mohm Compensation resistor value in milliohms (0-140mΩ, 20mΩ step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_bat_comp(bq25896_handle_t handle, uint8_t mohm);

/**
 * @brief Set IR compensation voltage clamp
 * Sets the maximum voltage compensation above charge voltage
 * 
 * @param handle Device handle
 * @param voltage_mv Voltage in mV (0-224mV, 32mV step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vclamp(bq25896_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Set thermal regulation threshold
 * Sets the die temperature threshold where charge current is reduced to prevent overheating
 * 
 * @param handle Device handle
 * @param threshold Thermal regulation threshold (60°C, 80°C, 100°C, 120°C)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_thermal_regulation_threshold(bq25896_handle_t handle, bq25896_treg_t threshold);

/**
 * @brief Get IR compensation and thermal regulation settings
 * 
 * @param handle Device handle
 * @param bat_comp_mohm Pointer to store battery compensation resistor in mΩ
 * @param vclamp_mv Pointer to store voltage clamp in mV
 * @param treg Pointer to store thermal regulation threshold
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ir_thermal_regulation(bq25896_handle_t handle, 
                                          uint8_t *bat_comp_mohm, 
                                          uint16_t *vclamp_mv, 
                                          bq25896_treg_t *treg);

/* REG09 Functions - Operation Control */

/**
 * @brief Force start Input Current Optimizer (ICO)
 * This bit is always auto-cleared after ICO starts
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_force_ico(bq25896_handle_t handle);

/**
 * @brief Enable/disable safety timer slowdown during input DPM or thermal regulation
 * When enabled, safety timer is slowed by 2X during input DPM or thermal regulation
 * 
 * @param handle Device handle
 * @param enable true to enable (default), false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_extend_safety_timer(bq25896_handle_t handle, bool enable);

/**
 * @brief Force BATFET off to enable ship mode
 * When set, the BATFET is turned off, disconnecting the battery from the system
 * 
 * @param handle Device handle
 * @param force true to force off, false to allow normal operation (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_force_batfet_off(bq25896_handle_t handle, bool force);

/**
 * @brief Set JEITA high temperature voltage setting
 * Sets the charge voltage during JEITA high temperature region
 * 
 * @param handle Device handle
 * @param vset JEITA high temperature voltage setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_jeita_high_temp_voltage(bq25896_handle_t handle, bq25896_jeita_vset_t vset);

/**
 * @brief Enable/disable BATFET turn off delay
 * When enabled and BATFET_DIS is set, BATFET turn off is delayed by tSM_DLY
 * 
 * @param handle Device handle
 * @param enable true to enable delay, false to disable delay (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_batfet_delay(bq25896_handle_t handle, bool enable);

/**
 * @brief Enable/disable BATFET full system reset
 * When enabled, BATFET will be briefly disconnected to reset the system if needed
 * Auto resets after 32s
 * 
 * @param handle Device handle
 * @param enable true to enable (default), false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enable_batfet_reset(bq25896_handle_t handle, bool enable);

/**
 * @brief Request voltage pulse up (only when EN_PUMPX is set)
 * This bit auto-clears after operation completes
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_request_pumpx_up(bq25896_handle_t handle);

/**
 * @brief Request voltage pulse down (only when EN_PUMPX is set)
 * This bit auto-clears after operation completes
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_request_pumpx_down(bq25896_handle_t handle);

/**
 * @brief Get operation control settings
 * 
 * @param handle Device handle
 * @param timer_extended Pointer to store safety timer extension status
 * @param batfet_off Pointer to store BATFET forced off status
 * @param batfet_delay_enabled Pointer to store BATFET delay enable status
 * @param jeita_vset Pointer to store JEITA high temperature voltage setting
 * @param batfet_reset_enabled Pointer to store BATFET reset enable status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_operation_control(bq25896_handle_t handle,
                                      bool *timer_extended,
                                      bool *batfet_off,
                                      bool *batfet_delay_enabled,
                                      bq25896_jeita_vset_t *jeita_vset,
                                      bool *batfet_reset_enabled);

/* REG0A Functions - Boost Mode Control */

/**
 * @brief Set boost mode voltage
 * Sets the output voltage when in boost (OTG) mode
 * 
 * @param handle Device handle
 * @param voltage_mv Voltage in mV (4550-5510mV, 64mV step)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_voltage(bq25896_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Enable/disable PFM mode in boost (OTG) mode
 * When disabled, forces PWM mode; when enabled, allows PFM mode for efficiency
 * 
 * @param handle Device handle
 * @param disable true to disable PFM (force PWM), false to allow PFM (default)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_disable_pfm_in_boost(bq25896_handle_t handle, bool disable);

/**
 * @brief Set boost mode current limit
 * Sets the maximum output current when in boost (OTG) mode
 * 
 * @param handle Device handle
 * @param limit Boost mode current limit setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_boost_current_limit(bq25896_handle_t handle, bq25896_boost_lim_t limit);

/**
 * @brief Get boost mode control settings
 * 
 * @param handle Device handle
 * @param boost_voltage_mv Pointer to store boost voltage in mV
 * @param pfm_disabled Pointer to store PFM mode disable status
 * @param boost_limit Pointer to store boost current limit setting
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_boost_mode_control(bq25896_handle_t handle, 
                                      uint16_t *boost_voltage_mv,
                                      bool *pfm_disabled,
                                      bq25896_boost_lim_t *boost_limit);

/* REG0B Functions - Status Register (Read-Only) */

/**
 * @brief Get VBUS status
 * Returns the current status of the VBUS input
 * 
 * @param handle Device handle
 * @param vbus_stat Pointer to store VBUS status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_status(bq25896_handle_t handle, bq25896_vbus_stat_t *vbus_stat);

/**
 * @brief Get charging status
 * Returns the current charging state
 * 
 * @param handle Device handle
 * @param chrg_stat Pointer to store charging status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charging_status(bq25896_handle_t handle, bq25896_chrg_stat_t *chrg_stat);

/**
 * @brief Get power good status
 * Returns whether input power is valid or not
 * 
 * @param handle Device handle
 * @param power_good Pointer to store power good status (true if power is good)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_power_good(bq25896_handle_t handle, bool *power_good);

/**
 * @brief Get VSYS regulation status
 * Returns whether battery is below VSYSMIN threshold
 * 
 * @param handle Device handle
 * @param in_vsysmin Pointer to store VSYS regulation status 
 *                   (true if battery is below VSYSMIN threshold)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vsys_status(bq25896_handle_t handle, bool *in_vsysmin);

/**
 * @brief Get complete status register information
 * Returns all status information from REG0B in a single call
 * 
 * @param handle Device handle
 * @param vbus_stat Pointer to store VBUS status
 * @param chrg_stat Pointer to store charging status
 * @param power_good Pointer to store power good status
 * @param in_vsysmin Pointer to store VSYS regulation status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_status_register(bq25896_handle_t handle, 
                           bq25896_vbus_stat_t *vbus_stat,
                           bq25896_chrg_stat_t *chrg_stat,
                           bool *power_good,
                           bool *in_vsysmin);

/* REG0C Functions - Fault Register (Read-Only) */

/**
 * @brief Get watchdog fault status
 * Returns whether watchdog timer has expired
 * 
 * @param handle Device handle
 * @param watchdog_fault Pointer to store watchdog fault status (true if fault occurred)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_watchdog_fault(bq25896_handle_t handle, bool *watchdog_fault);

/**
 * @brief Get boost mode fault status
 * Returns whether a boost mode fault has occurred (VBUS overloaded, OVP, or low battery)
 * 
 * @param handle Device handle
 * @param boost_fault Pointer to store boost fault status (true if fault occurred)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_boost_fault(bq25896_handle_t handle, bool *boost_fault);

/**
 * @brief Get charge fault status
 * Returns information about charging faults
 * 
 * @param handle Device handle
 * @param chrg_fault Pointer to store charge fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_fault(bq25896_handle_t handle, bq25896_chrg_fault_t *chrg_fault);

/**
 * @brief Get battery fault status
 * Returns whether battery OVP has occurred
 * 
 * @param handle Device handle
 * @param bat_fault Pointer to store battery fault status (true if fault occurred)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_battery_fault(bq25896_handle_t handle, bool *bat_fault);

/**
 * @brief Get NTC fault status
 * Returns temperature sensor fault information
 * 
 * @param handle Device handle
 * @param ntc_fault Pointer to store NTC fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ntc_fault(bq25896_handle_t handle, bq25896_ntc_fault_t *ntc_fault);

/**
 * @brief Get complete fault register information
 * Returns all fault information from REG0C in a single call
 * 
 * @param handle Device handle
 * @param watchdog_fault Pointer to store watchdog fault status
 * @param boost_fault Pointer to store boost fault status
 * @param chrg_fault Pointer to store charge fault status
 * @param bat_fault Pointer to store battery fault status
 * @param ntc_fault Pointer to store NTC fault status
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_fault_register(bq25896_handle_t handle, 
                                  bool *watchdog_fault,
                                  bool *boost_fault,
                                  bq25896_chrg_fault_t *chrg_fault,
                                  bool *bat_fault,
                                  bq25896_ntc_fault_t *ntc_fault);

/* REG0D Functions - VINDPM/Input Voltage Limit */

/**
 * @brief Set VINDPM threshold setting method
 * Selects whether to use relative (based on measured VBUS) or absolute VINDPM threshold
 * 
 * @param handle Device handle
 * @param mode VINDPM threshold setting method
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vindpm_mode(bq25896_handle_t handle, bq25896_vindpm_mode_t mode);

/**
 * @brief Set absolute VINDPM threshold
 * Sets the minimum input voltage threshold directly
 * Note: Only effective when FORCE_VINDPM=1 (absolute mode)
 * Note: Values < 3.9V are clamped to 3.9V
 * 
 * @param handle Device handle
 * @param voltage_mv Voltage in mV (3900-15300mV)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_vindpm_voltage(bq25896_handle_t handle, uint16_t voltage_mv);

/**
 * @brief Get VINDPM settings
 * 
 * @param handle Device handle
 * @param mode Pointer to store VINDPM threshold setting method
 * @param voltage_mv Pointer to store absolute VINDPM threshold in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vindpm_settings(bq25896_handle_t handle, 
                                   bq25896_vindpm_mode_t *mode,
                                   uint16_t *voltage_mv);

/* REG0E Functions - Battery Voltage Monitor (Read-Only) */

/**
 * @brief Get thermal regulation status
 * Returns whether the IC is in thermal regulation
 * 
 * @param handle Device handle
 * @param in_thermal_regulation Pointer to store thermal regulation status 
 *                              (true if in thermal regulation)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_thermal_regulation_status(bq25896_handle_t handle, bool *in_thermal_regulation);

/**
 * @brief Get battery voltage
 * Returns the battery voltage measured by the ADC
 * 
 * @param handle Device handle
 * @param battery_mv Pointer to store battery voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_battery_voltage(bq25896_handle_t handle, uint16_t *battery_mv);

/**
 * @brief Get all battery voltage monitor information
 * Returns both thermal regulation status and battery voltage in a single call
 * 
 * @param handle Device handle
 * @param in_thermal_regulation Pointer to store thermal regulation status
 * @param battery_mv Pointer to store battery voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_battery_voltage_monitor(bq25896_handle_t handle, 
                                           bool *in_thermal_regulation,
                                           uint16_t *battery_mv);

/* REG0F Functions - System Voltage Monitor (Read-Only) */

/**
 * @brief Get system voltage
 * Returns the system voltage measured by the ADC
 * 
 * @param handle Device handle
 * @param system_mv Pointer to store system voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_system_voltage(bq25896_handle_t handle, uint16_t *system_mv);

/* REG10 Functions - Thermal Sensor Voltage Monitor (Read-Only) */

/**
 * @brief Get TS voltage percentage
 * Returns the TS pin voltage as a percentage of REGN voltage
 * Used for battery temperature monitoring through NTC resistor
 * 
 * @param handle Device handle
 * @param ts_percentage Pointer to store TS voltage percentage
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ts_voltage_percentage(bq25896_handle_t handle, float *ts_percentage);

/* REG11 Functions - VBUS Voltage Monitor (Read-Only) */

/**
 * @brief Get VBUS good status
 * Returns whether VBUS is attached and within valid range
 * 
 * @param handle Device handle
 * @param vbus_attached Pointer to store VBUS attached status (true if attached)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_good(bq25896_handle_t handle, bool *vbus_attached);

/**
 * @brief Get VBUS voltage
 * Returns the VBUS voltage measured by the ADC
 * 
 * @param handle Device handle
 * @param vbus_mv Pointer to store VBUS voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_voltage(bq25896_handle_t handle, uint16_t *vbus_mv);

/**
 * @brief Get all VBUS voltage monitor information
 * Returns both VBUS good status and VBUS voltage in a single call
 * 
 * @param handle Device handle
 * @param vbus_attached Pointer to store VBUS attached status
 * @param vbus_mv Pointer to store VBUS voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vbus_voltage_monitor(bq25896_handle_t handle, 
                                        bool *vbus_attached,
                                        uint16_t *vbus_mv);

/* REG12 Functions - Charge Current Monitor (Read-Only) */

/**
 * @brief Get charge current
 * Returns the charge current measured by the ADC
 * Note: Returns 0 when VBAT < VBATSHORT
 * 
 * @param handle Device handle
 * @param current_ma Pointer to store charge current in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_charge_current(bq25896_handle_t handle, uint16_t *current_ma);

/* REG13 Functions - Input Current Limit Monitor (Read-Only) */

/**
 * @brief Get VINDPM status
 * Returns whether the input voltage is below VINDPM threshold
 * 
 * @param handle Device handle
 * @param in_vindpm Pointer to store VINDPM status (true if in VINDPM)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_vindpm_status(bq25896_handle_t handle, bool *in_vindpm);

/**
 * @brief Get IINDPM status
 * Returns whether the input current is in DPM mode (limited by ILIM)
 * 
 * @param handle Device handle
 * @param in_iindpm Pointer to store IINDPM status (true if in IINDPM)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_iindpm_status(bq25896_handle_t handle, bool *in_iindpm);

/**
 * @brief Get input current limit in effect during ICO
 * Returns the actual input current limit when Input Current Optimizer is enabled
 * 
 * @param handle Device handle
 * @param current_ma Pointer to store input current limit in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_idpm_limit(bq25896_handle_t handle, uint16_t *current_ma);

/**
 * @brief Get all input current limit monitor information
 * Returns VINDPM status, IINDPM status, and input current limit in a single call
 * 
 * @param handle Device handle
 * @param in_vindpm Pointer to store VINDPM status
 * @param in_iindpm Pointer to store IINDPM status
 * @param current_ma Pointer to store input current limit in mA
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_input_current_limit_monitor(bq25896_handle_t handle, 
                                              bool *in_vindpm,
                                              bool *in_iindpm,
                                              uint16_t *current_ma);

/* REG14 Functions - Device Information & Control */

/**
 * @brief Perform register reset
 * Resets all registers to default values and resets safety timer
 * This bit automatically clears after reset is complete
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_register_reset(bq25896_handle_t handle);

/**
 * @brief Get Input Current Optimizer (ICO) status
 * Returns whether ICO has completed optimization and detected maximum current
 * 
 * @param handle Device handle
 * @param ico_optimized Pointer to store ICO optimization status
 *                     (true if maximum current detected, false if still optimizing)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_ico_status(bq25896_handle_t handle, bool *ico_optimized);

/**
 * @brief Get device part number
 * Returns the device part number (bq25896)
 * 
 * @param handle Device handle
 * @param is_bq25896 Pointer to store part number check (true if device is bq25896)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_part_number(bq25896_handle_t handle, bool *is_bq25896);

/**
 * @brief Get device temperature profile
 * Returns whether device uses JEITA profile (always true for bq25896)
 * 
 * @param handle Device handle
 * @param is_jeita Pointer to store temperature profile status (true for JEITA)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_temperature_profile(bq25896_handle_t handle, bool *is_jeita);

/**
 * @brief Get device revision
 * Returns the device revision number
 * 
 * @param handle Device handle
 * @param revision Pointer to store device revision (10 for current revision)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_device_revision(bq25896_handle_t handle, uint8_t *revision);

/**
 * @brief Get all device information
 * Returns all device information and ICO status in a single call
 * 
 * @param handle Device handle
 * @param ico_optimized Pointer to store ICO optimization status
 * @param is_bq25896 Pointer to store part number check
 * @param is_jeita Pointer to store temperature profile status
 * @param revision Pointer to store device revision
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_device_info(bq25896_handle_t handle,
                               bool *ico_optimized,
                               bool *is_bq25896,
                               bool *is_jeita,
                               uint8_t *revision);

/**
 * @brief Structure containing all ADC readings from the BQ25896
 */
typedef struct {
    uint16_t battery_mv;          // Battery voltage in mV
    uint16_t system_mv;           // System voltage in mV
    float ts_percentage;          // TS pin voltage as percentage
    uint16_t vbus_mv;             // VBUS voltage in mV
    uint16_t charge_current_ma;   // Charge current in mA
    uint16_t idpm_limit_ma;       // IDPM limit in mA
    bool vbus_present;            // VBUS good status
    bool in_thermal_regulation;   // Thermal regulation status
    bq25896_vbus_stat_t vbus_stat; // VBUS status
    bq25896_chrg_stat_t chrg_stat; // Charging status
} bq25896_adc_readings_t;

/**
 * @brief JEITA temperature profile threshold levels
 */
typedef enum {
    BQ25896_JEITA_COLD,   // Below cold threshold - Charging disabled
    BQ25896_JEITA_COOL,   // Between cold and cool threshold - Reduced charging
    BQ25896_JEITA_NORMAL, // Between cool and warm threshold - Normal charging
    BQ25896_JEITA_WARM,   // Between warm and hot threshold - Reduced charging
    BQ25896_JEITA_HOT     // Above hot threshold - Charging disabled
} bq25896_jeita_zone_t;

/**
 * @brief JEITA temperature profile configuration
 * Defines charging parameters for different temperature ranges
 */
typedef struct {
    /* NTC Thresholds */
    float cold_threshold_pct;      // TS percentage for cold threshold (default: 80%)
    float cool_threshold_pct;      // TS percentage for cool threshold (default: 70%)
    float warm_threshold_pct;      // TS percentage for warm threshold (default: 40%)
    float hot_threshold_pct;       // TS percentage for hot threshold (default: 30%)
    
    /* Charge parameters for each temperature zone */
    uint16_t cool_charge_voltage_mv; // Charge voltage in cool zone (default: 4100mV)
    uint16_t cool_charge_current_pct; // Charge current in cool zone as % of fast charge (default: 50%)
    uint16_t warm_charge_voltage_mv; // Charge voltage in warm zone (default: 4100mV)
    uint16_t warm_charge_current_pct; // Charge current in warm zone as % of fast charge (default: 50%)
} bq25896_jeita_profile_t;

/**
 * @brief Configure JEITA temperature profile and related charging parameters
 * 
 * @param handle Device handle
 * @param profile JEITA profile to configure
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_configure_jeita_profile(bq25896_handle_t handle, const bq25896_jeita_profile_t *profile);

/**
 * @brief Get current JEITA zone based on battery temperature
 * Measures the TS pin voltage and determines the current JEITA temperature zone
 * 
 * @param handle Device handle
 * @param zone Pointer to store the current JEITA zone
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_jeita_zone(bq25896_handle_t handle, bq25896_jeita_zone_t *zone);

/**
 * @brief BQ25896 event types for interrupt notifications
 */
typedef enum {
    BQ25896_EVENT_CHARGE_COMPLETE,      // Charging complete
    BQ25896_EVENT_CHARGING,             // Charging in progress
    BQ25896_EVENT_VBUS_PRESENT,         // VBUS connected
    BQ25896_EVENT_VBUS_ABSENT,          // VBUS disconnected
    BQ25896_EVENT_BOOST_MODE,           // Entered boost (OTG) mode
    BQ25896_EVENT_FAULT_WATCHDOG,       // Watchdog timer expired
    BQ25896_EVENT_FAULT_BOOST,          // Boost mode fault
    BQ25896_EVENT_FAULT_CHARGE,         // Charge fault
    BQ25896_EVENT_FAULT_BATTERY,        // Battery fault
    BQ25896_EVENT_FAULT_NTC,            // NTC (temperature) fault
    BQ25896_EVENT_VINDPM_ACTIVE,        // VINDPM active (input voltage low)
    BQ25896_EVENT_IINDPM_ACTIVE,        // IINDPM active (input current limit)
    BQ25896_EVENT_THERMAL_REGULATION,   // Thermal regulation active
} bq25896_event_t;

/**
 * @brief Callback function type for BQ25896 events
 */
typedef void (*bq25896_event_callback_t)(bq25896_event_t event, void *user_data);

/**
 * @brief Register an interrupt callback for STAT pin events
 * Note: The STAT pin must be connected to a GPIO with interrupt capability
 * 
 * @param handle Device handle
 * @param callback Function to call when events occur
 * @param user_data User data to pass to the callback function
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_register_event_handler(bq25896_handle_t handle, bq25896_event_callback_t callback, void *user_data);

/**
 * @brief Manually process events (poll event status)
 * This can be used instead of interrupt-based event handling
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_process_events(bq25896_handle_t handle);

/**
 * @brief Get all ADC readings from the BQ25896
 * Performs ADC conversion and collects all readings in a single call
 * 
 * @param handle Device handle
 * @param readings Pointer to store ADC readings
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_all_adc_readings(bq25896_handle_t handle, bq25896_adc_readings_t *readings);

/* Power Path Management */

/**
 * @brief Configure the device for power path management
 * Sets up optimized settings for cases where the BQ25896 powers the system directly
 * 
 * @param handle Device handle
 * @param sys_min_voltage Minimum system voltage to maintain
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_configure_power_path(bq25896_handle_t handle, bq25896_sys_min_t sys_min_voltage);

/**
 * @brief Set Power Path Mode
 * Controls whether the BQ25896 operates in 'supplement mode' where
 * system load can exceed input current capability
 * 
 * @param handle Device handle
 * @param enable_supplement true to enable supplement mode (system can draw from battery if needed)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_supplement_mode(bq25896_handle_t handle, bool enable_supplement);

/**
 * @brief Set input current optimization
 * The Input Current Optimizer automatically adjusts input current to
 * maximize power draw without tripping input source voltage limits
 * 
 * @param handle Device handle
 * @param enable true to enable ICO, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_input_current_optimization(bq25896_handle_t handle, bool enable);

/**
 * @brief BQ25896 error codes
 */
typedef enum {
    BQ25896_ERROR_NONE = 0,                 // No error
    BQ25896_ERROR_INVALID_PARAM = -1,       // Invalid parameter
    BQ25896_ERROR_I2C_FAILED = -2,          // I2C communication error
    BQ25896_ERROR_DEVICE_NOT_FOUND = -3,    // Device not found or not responding
    BQ25896_ERROR_WATCHDOG_EXPIRED = -4,    // Watchdog timer expired
    BQ25896_ERROR_BOOST_FAULT = -5,         // Boost mode fault
    BQ25896_ERROR_CHARGE_FAULT = -6,        // Charging fault
    BQ25896_ERROR_BATTERY_FAULT = -7,       // Battery fault
    BQ25896_ERROR_NTC_FAULT = -8,           // Temperature sensor fault
    BQ25896_ERROR_UNSUPPORTED = -9,         // Operation not supported
} bq25896_error_t;

/**
 * @brief Convert error code to string
 * 
 * @param error Error code to convert
 * @return const char* String representation of the error
 */
const char* bq25896_error_to_string(bq25896_error_t error);

/**
 * @brief Dump all register contents for debugging
 * 
 * @param handle Device handle
 * @param output_buf Buffer to store the register dump text
 * @param buf_size Size of the output buffer
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_dump_registers(bq25896_handle_t handle, char *output_buf, size_t buf_size);

/**
 * @brief Get specific error information after a fault
 * Reads the fault register and provides detailed information about the fault
 * 
 * @param handle Device handle
 * @param error_code Pointer to store the error code
 * @param error_details Pointer to store detailed error information
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_error_info(bq25896_handle_t handle, 
                              bq25896_error_t *error_code, 
                              char *error_details,
                              size_t details_size);

/**
 * @brief Battery health status
 */
typedef enum {
    BQ25896_BATTERY_HEALTH_UNKNOWN = 0,    // Health status unknown
    BQ25896_BATTERY_HEALTH_GOOD,           // Battery health is good
    BQ25896_BATTERY_HEALTH_COLD,           // Battery is too cold
    BQ25896_BATTERY_HEALTH_HOT,            // Battery is too hot
    BQ25896_BATTERY_HEALTH_OVERVOLTAGE,    // Battery voltage is too high
    BQ25896_BATTERY_HEALTH_DEAD,           // Battery is dead or very low capacity
    BQ25896_BATTERY_HEALTH_DEGRADED        // Battery capacity has degraded significantly
} bq25896_battery_health_t;

/**
 * @brief Battery status information
 */
typedef struct {
    bq25896_battery_health_t health;         // Battery health status
    float soc_percent;                       // State of charge (0-100%)
    float voltage_mv;                        // Current battery voltage in mV
    float current_ma;                        // Current battery current in mA
    float temperature_c;                     // Battery temperature in Celsius
    float input_voltage_mv;                  // Input voltage in mV
    float charge_power_mw;                   // Charging power in mW
    bool is_charging;                        // Whether the battery is charging
    bq25896_chrg_stat_t charge_state;        // Charging state
    int32_t remaining_capacity_mah;          // Estimated remaining capacity in mAh
} bq25896_battery_status_t;

/**
 * @brief Get comprehensive battery status information
 * 
 * @param handle Device handle
 * @param status Pointer to store the battery status information
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_battery_status(bq25896_handle_t handle, bq25896_battery_status_t *status);

/**
 * @brief Estimate battery state of charge
 * This is a simplified estimation based on voltage curve. For precise SoC,
 * external fuel gauge is recommended.
 * 
 * @param handle Device handle
 * @param soc_percent Pointer to store the state of charge percentage (0-100)
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_estimate_battery_soc(bq25896_handle_t handle, float *soc_percent);

/**
 * @brief Set battery capacity for SoC calculations
 * 
 * @param handle Device handle
 * @param capacity_mah Battery capacity in mAh
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_battery_capacity(bq25896_handle_t handle, uint32_t capacity_mah);

/**
 * @brief Get estimated time to full charge
 * 
 * @param handle Device handle
 * @param minutes Pointer to store the estimated time in minutes
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_time_to_full(bq25896_handle_t handle, uint32_t *minutes);

/**
 * @brief Power saving mode configuration
 */
typedef enum {
    BQ25896_POWER_SAVE_NORMAL = 0,      // Normal operation mode
    BQ25896_POWER_SAVE_LOW,             // Low power mode with reduced current
    BQ25896_POWER_SAVE_SHIP,            // Ship mode (ultra-low power)
    BQ25896_POWER_SAVE_HIBERNATE        // Hibernate mode
} bq25896_power_save_mode_t;

/**
 * @brief Configure power saving mode
 * 
 * @param handle Device handle
 * @param mode Power saving mode to set
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_power_save_mode(bq25896_handle_t handle, bq25896_power_save_mode_t mode);

/**
 * @brief Power management options
 */
typedef struct {
    bool dynamic_power_management;       // Dynamically adjust power settings based on load
    bool enable_low_power_mode;          // Enable low power mode during idle
    bool disable_watchdog_in_sleep;      // Disable watchdog timer during sleep mode
    uint32_t auto_wakeup_time_ms;        // Auto wake-up time in milliseconds (0 = disabled)
    bool force_dpm;                      // Force Dynamic Power Management active
} bq25896_power_mgmt_opts_t;

/**
 * @brief Configure advanced power management options
 * 
 * @param handle Device handle
 * @param opts Power management options
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_config_power_management(bq25896_handle_t handle, const bq25896_power_mgmt_opts_t *opts);

/**
 * @brief Put the device into ultra-low power mode (ship mode)
 * Note: Device will remain in ship mode until power is cycled
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_enter_ship_mode(bq25896_handle_t handle);

/**
 * @brief Set hibernate mode (lower power than normal but can wake up via I2C)
 * 
 * @param handle Device handle
 * @param enable True to enable hibernate mode, false to disable
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_set_hibernate_mode(bq25896_handle_t handle, bool enable);

/**
 * @brief Thermal regulation configuration
 */
typedef struct {
    bool enable_thermal_regulation;              // Enable thermal regulation feature
    bq25896_treg_t threshold;                    // Thermal regulation threshold
    uint8_t thermal_recovery_hysteresis_c;       // Degrees C to recover from thermal regulation
    bool enable_charge_current_reduction;        // Reduce charge current during thermal regulation
} bq25896_thermal_config_t;

/**
 * @brief Configure thermal regulation settings
 * 
 * @param handle Device handle
 * @param config Thermal regulation configuration
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_configure_thermal_regulation(bq25896_handle_t handle, const bq25896_thermal_config_t *config);

/**
 * @brief Get current thermal status
 * 
 * @param handle Device handle
 * @param is_in_thermal_regulation Pointer to store whether device is in thermal regulation
 * @param temperature_c Pointer to store current temperature in Celsius
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_get_thermal_status(bq25896_handle_t handle, 
                                 bool *is_in_thermal_regulation,
                                 float *temperature_c);

/**
 * @brief Command batch structure for efficient register operations
 */
typedef struct {
    void *internal;                            // Internal state for batching
    uint8_t batch_count;                       // Number of operations in the batch
    bool auto_commit;                          // Whether to automatically commit when batch is full
} bq25896_batch_t;

/**
 * @brief Create a new command batch
 * 
 * @param handle Device handle
 * @param batch Pointer to batch structure to initialize
 * @param auto_commit Automatically commit when batch buffer is full
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_batch_begin(bq25896_handle_t handle, bq25896_batch_t *batch, bool auto_commit);

/**
 * @brief Add a register write operation to the batch
 * 
 * @param handle Device handle
 * @param batch Pointer to batch structure
 * @param reg Register address to write
 * @param value Value to write to the register
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_batch_write_reg(bq25896_handle_t handle, bq25896_batch_t *batch, 
                              uint8_t reg, uint8_t value);

/**
 * @brief Execute all batched operations
 * 
 * @param handle Device handle
 * @param batch Pointer to batch structure
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_batch_commit(bq25896_handle_t handle, bq25896_batch_t *batch);

/**
 * @brief Cancel all batched operations without executing them
 * 
 * @param handle Device handle
 * @param batch Pointer to batch structure
 */
void bq25896_batch_cancel(bq25896_handle_t handle, bq25896_batch_t *batch);

/**
 * @brief Example of a batched configuration
 * Sets multiple charging parameters in a single transaction
 * 
 * @param handle Device handle
 * @param input_current_ma Input current limit in mA
 * @param charge_current_ma Charge current in mA
 * @param charge_voltage_mv Charge voltage in mV
 * @return esp_err_t ESP_OK on success, error otherwise
 */
esp_err_t bq25896_configure_charging_batched(bq25896_handle_t handle,
                                         uint16_t input_current_ma,
                                         uint16_t charge_current_ma,
                                         uint16_t charge_voltage_mv);

#ifdef __cplusplus
}
#endif
