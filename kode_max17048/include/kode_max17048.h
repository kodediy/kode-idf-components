#ifndef KODE_MAX17048_H
#define KODE_MAX17048_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @file kode_max17048.h
 * @brief Driver for MAX17048/MAX17049 1-Cell/2-Cell Fuel Gauge with ModelGauge
 * 
 * This driver provides a comprehensive interface to the MAX17048/MAX17049
 * fuel gauge IC. It supports all register operations, battery monitoring,
 * alerts configuration, and power management features.
 * 
 * @note All registers in this device are 16-bit (2 bytes) and must be 
 *       accessed using 16-bit operations.
 */

/* Driver Version Information */
#define KODE_MAX17048_DRIVER_MAJOR         1     /* Major version number */
#define KODE_MAX17048_DRIVER_MINOR         0     /* Minor version number */
#define KODE_MAX17048_DRIVER_PATCH         0     /* Patch version number */

/* MAX17048 I2C Address */
#define MAX17048_I2C_ADDR                  0x36

/* Register Access Specifications */
#define MAX17048_REG_SIZE                  2    /* All registers are 16-bit (2 bytes) */
typedef uint16_t max17048_reg_t;                /* Type for register values */

/* Timing Specifications */
#define MAX17048_VCELL_UPDATE_MS           250  /* VCELL updates every 250ms in active mode */
#define MAX17048_VCELL_HIBERNATE_UPDATE_S  45   /* VCELL updates every 45s in hibernate mode */
#define MAX17048_SOC_FIRST_UPDATE_MS      1000  /* First SOC update after POR */
#define MAX17048_HIBERNATE_ENTER_TIME_S    360  /* Time threshold to enter hibernate mode (6 minutes) */

/**
 * @brief MAX17048 device handle
 */
typedef struct max17048_dev_t *max17048_handle_t;

/* Forward declaration of configuration structure */
typedef struct max17048_config_s max17048_config_t;

/* STATUS Register Alert Types Structure */
typedef struct {
    bool reset_indicator;     /* RI: Set on power-up, indicates device needs configuration */
    bool voltage_high;        /* VH: VCELL > VALRT.MAX */
    bool voltage_low;         /* VL: VCELL < VALRT.MIN */
    bool voltage_reset;       /* VR: Reset occurred (if EnVr set) */
    bool soc_low;            /* HD: SOC crossed CONFIG.ATHD threshold */
    bool soc_change;         /* SC: SOC changed by at least 1% (if ALSC set) */
    bool voltage_reset_en;   /* EnVr: Enable voltage reset alerts */
} max17048_status_t;

/**
 * @brief Battery data structure
 */
typedef struct {
    float voltage;       /* Battery voltage in volts */
    float soc;           /* State of charge in percentage */
    float charge_rate;   /* Charge/discharge rate in %/hr */
    uint16_t version;    /* Chip version */
    uint16_t status;     /* Status register value */
} max17048_data_t;

/**
 * @brief Configuration structure for device initialization
 */
typedef struct max17048_config_s {
    uint8_t rcomp;                /* RCOMP value (default: 0x97) */
    uint8_t alert_threshold;      /* SOC alert threshold (1-32%, default: 4%) */
    bool enable_soc_change_alert; /* Enable SOC change alerts */
    bool enable_voltage_alerts;   /* Enable voltage alerts */
    float voltage_min_mv;         /* Minimum voltage alert threshold (mV) */
    float voltage_max_mv;         /* Maximum voltage alert threshold (mV) */
    bool enable_hibernate;        /* Enable hibernate mode */
    float hibernate_threshold_pct_per_hr; /* Hibernate threshold (%/hr) */
    float active_threshold_mv;    /* Active threshold (mV) */
    bool disable_comparator;      /* Disable analog comparator in hibernate mode */
    float vreset_threshold_mv;    /* Reset voltage threshold (mV) */
    bool enable_vreset_alert;     /* Enable voltage reset alerts */
} max17048_config_t;

/**
 * @brief Default configuration for MAX17048
 * 
 * These values provide a safe starting point for most applications
 */
extern const max17048_config_t MAX17048_DEFAULT_CONFIG;

/* I2C Operation Helpers */
esp_err_t max17048_read_reg(max17048_handle_t handle, uint8_t reg_addr, max17048_reg_t *value);
esp_err_t max17048_write_reg(max17048_handle_t handle, uint8_t reg_addr, max17048_reg_t value);

/* Battery Voltage Functions */
esp_err_t max17048_get_vcell_mv(max17048_handle_t handle, float *voltage_mv);
esp_err_t max17048_get_vcell_raw(max17048_handle_t handle, max17048_reg_t *vcell_raw);

/* State of Charge Functions */
esp_err_t max17048_get_soc_percent(max17048_handle_t handle, float *soc_percent);
esp_err_t max17048_get_soc_raw(max17048_handle_t handle, max17048_reg_t *soc_raw);
esp_err_t max17048_get_soc_parts(max17048_handle_t handle, uint8_t *soc_integer, uint8_t *soc_fraction);

/* Power Mode Functions */
esp_err_t max17048_quick_start(max17048_handle_t handle);
esp_err_t max17048_set_sleep_mode(max17048_handle_t handle, bool enable);
esp_err_t max17048_is_hibernating(max17048_handle_t handle, bool *is_hibernating);
esp_err_t max17048_get_mode_raw(max17048_handle_t handle, max17048_reg_t *mode_raw);

/* Version Functions */
esp_err_t max17048_get_version(max17048_handle_t handle, max17048_reg_t *version);
esp_err_t max17048_verify_version(max17048_handle_t handle, bool *version_ok);

/* Hibernate Functions */
esp_err_t max17048_configure_hibernate(max17048_handle_t handle, float hib_thr_pct_per_hr, float act_thr_mv);
esp_err_t max17048_disable_hibernate(max17048_handle_t handle);
esp_err_t max17048_force_hibernate(max17048_handle_t handle);
esp_err_t max17048_get_hibernate_config(max17048_handle_t handle, float *hib_thr_pct_per_hr, float *act_thr_mv);
esp_err_t max17048_get_hibrt_raw(max17048_handle_t handle, max17048_reg_t *hibrt_raw);

/* Config Register Functions */
esp_err_t max17048_set_rcomp(max17048_handle_t handle, uint8_t rcomp);
esp_err_t max17048_set_empty_alert_threshold(max17048_handle_t handle, uint8_t percent);
esp_err_t max17048_set_soc_change_alert(max17048_handle_t handle, bool enable);
esp_err_t max17048_clear_alert(max17048_handle_t handle);
esp_err_t max17048_config_sleep(max17048_handle_t handle, bool sleep);
esp_err_t max17048_get_config(max17048_handle_t handle, uint8_t *rcomp, bool *sleep, bool *alsc, 
                            bool *alert, uint8_t *empty_alert_threshold);
esp_err_t max17048_get_config_raw(max17048_handle_t handle, max17048_reg_t *config_raw);

/* Voltage Alert Functions */
esp_err_t max17048_set_voltage_alerts(max17048_handle_t handle, float max_mv, float min_mv);
esp_err_t max17048_disable_voltage_alerts(max17048_handle_t handle);
esp_err_t max17048_get_voltage_alerts(max17048_handle_t handle, float *max_mv, float *min_mv);
esp_err_t max17048_get_valrt_raw(max17048_handle_t handle, max17048_reg_t *valrt_raw);

/* Charge Rate Functions */
esp_err_t max17048_get_charge_rate(max17048_handle_t handle, float *rate_pct_per_hr);
esp_err_t max17048_get_crate_raw(max17048_handle_t handle, max17048_reg_t *crate_raw);

/* VRESET Functions */
esp_err_t max17048_configure_vreset(max17048_handle_t handle, float reset_threshold_mv, bool disable_comparator);
esp_err_t max17048_set_vreset_captive(max17048_handle_t handle, bool disable_comparator);
esp_err_t max17048_set_vreset_removable(max17048_handle_t handle, float empty_voltage_mv, bool disable_comparator);
esp_err_t max17048_get_id(max17048_handle_t handle, uint8_t *id);
esp_err_t max17048_get_vreset_config(max17048_handle_t handle, float *reset_threshold_mv, bool *comp_disabled);
esp_err_t max17048_get_vreset_id_raw(max17048_handle_t handle, max17048_reg_t *vreset_id_raw);

/* Status Register Functions */
esp_err_t max17048_get_status(max17048_handle_t handle, max17048_status_t *status);
esp_err_t max17048_clear_status_flags(max17048_handle_t handle, uint16_t flags);
esp_err_t max17048_set_voltage_reset_alert(max17048_handle_t handle, bool enable);
esp_err_t max17048_needs_config(max17048_handle_t handle, bool *needs_config);
esp_err_t max17048_get_status_raw(max17048_handle_t handle, max17048_reg_t *status_raw);

/* Model Table Functions */
esp_err_t max17048_unlock_table(max17048_handle_t handle);
esp_err_t max17048_lock_table(max17048_handle_t handle);
esp_err_t max17048_write_custom_model(max17048_handle_t handle, const uint16_t *model_data, uint8_t length);
esp_err_t max17048_is_table_unlocked(max17048_handle_t handle, bool *is_unlocked);

/* CMD Register Functions */
esp_err_t max17048_write_cmd(max17048_handle_t handle, max17048_reg_t cmd_value);

/* General Device Functions */
esp_err_t max17048_init(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr, max17048_handle_t *handle);
esp_err_t max17048_delete(max17048_handle_t handle);
esp_err_t max17048_reset(max17048_handle_t handle);
esp_err_t max17048_configure(max17048_handle_t handle, const max17048_config_t *config);
uint32_t max17048_get_driver_version(void);
esp_err_t max17048_init_config_default(max17048_handle_t handle);
esp_err_t max17048_get_default_config(max17048_config_t *config);
esp_err_t max17048_get_battery_data(max17048_handle_t handle, max17048_data_t *data);

#endif /* KODE_MAX17048_H */ 