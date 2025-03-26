#pragma once

#include <stdint.h>
#include "kode_bq25896.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register 00h (Input Source Control) bit definitions
 */
#define BQ25896_REG00_ENHIZ_MASK         0x80  // Bit 7: Enable HIZ Mode
#define BQ25896_REG00_ENHIZ_SHIFT        7
#define BQ25896_REG00_EN_ILIM_MASK       0x40  // Bit 6: Enable ILIM Pin
#define BQ25896_REG00_EN_ILIM_SHIFT      6
#define BQ25896_REG00_IINLIM_MASK        0x3F  // Bits 5-0: Input Current Limit
#define BQ25896_REG00_IINLIM_SHIFT       0

/**
 * @brief Register 01h (Power-On Configuration) bit definitions
 */
#define BQ25896_REG01_BHOT1_MASK         0x80  // Bit 7: Boost Mode Hot Temperature Monitor
#define BQ25896_REG01_BHOT1_SHIFT        7
#define BQ25896_REG01_BHOT0_MASK         0x40  // Bit 6: Boost Mode Hot Temperature Monitor
#define BQ25896_REG01_BHOT0_SHIFT        6
#define BQ25896_REG01_BCOLD_MASK         0x20  // Bit 5: Boost Mode Cold Temperature Monitor 
#define BQ25896_REG01_BCOLD_SHIFT        5
#define BQ25896_REG01_VINDPM_OS4_MASK    0x10  // Bit 4: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS4_SHIFT   4
#define BQ25896_REG01_VINDPM_OS3_MASK    0x08  // Bit 3: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS3_SHIFT   3
#define BQ25896_REG01_VINDPM_OS2_MASK    0x04  // Bit 2: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS2_SHIFT   2
#define BQ25896_REG01_VINDPM_OS1_MASK    0x02  // Bit 1: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS1_SHIFT   1
#define BQ25896_REG01_VINDPM_OS0_MASK    0x01  // Bit 0: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS0_SHIFT   0

// Combined masks for easier use
#define BQ25896_REG01_BHOT_MASK          0xC0  // Bits 7-6: Boost Hot Temp Monitor
#define BQ25896_REG01_BHOT_SHIFT         6
#define BQ25896_REG01_VINDPM_OS_MASK     0x1F  // Bits 4-0: Input Voltage Limit Offset
#define BQ25896_REG01_VINDPM_OS_SHIFT    0

/**
 * @brief Register 02h (Charge Current Control) bit definitions
 */
#define BQ25896_REG02_CONV_START_MASK    0x80  // Bit 7: ADC Conversion Start Control
#define BQ25896_REG02_CONV_START_SHIFT   7
#define BQ25896_REG02_CONV_RATE_MASK     0x40  // Bit 6: ADC Conversion Rate Selection
#define BQ25896_REG02_CONV_RATE_SHIFT    6
#define BQ25896_REG02_BOOST_FREQ_MASK    0x20  // Bit 5: Boost Mode Frequency Selection
#define BQ25896_REG02_BOOST_FREQ_SHIFT   5
#define BQ25896_REG02_ICO_EN_MASK        0x10  // Bit 4: Input Current Optimizer Enable
#define BQ25896_REG02_ICO_EN_SHIFT       4
// Bits 3-2 are Reserved
#define BQ25896_REG02_FORCE_DPDM_MASK    0x02  // Bit 1: Force Input Detection
#define BQ25896_REG02_FORCE_DPDM_SHIFT   1
#define BQ25896_REG02_AUTO_DPDM_EN_MASK  0x01  // Bit 0: Automatic Input Detection Enable
#define BQ25896_REG02_AUTO_DPDM_EN_SHIFT 0

/**
 * @brief Register 03h (Charge Control) bit definitions
 */
#define BQ25896_REG03_BAT_LOADEN_MASK    0x80  // Bit 7: Battery Load (IBATLOAD) Enable
#define BQ25896_REG03_BAT_LOADEN_SHIFT   7
#define BQ25896_REG03_WD_RST_MASK        0x40  // Bit 6: I2C Watchdog Timer Reset
#define BQ25896_REG03_WD_RST_SHIFT       6
#define BQ25896_REG03_OTG_CONFIG_MASK    0x20  // Bit 5: Boost (OTG) Mode Configuration
#define BQ25896_REG03_OTG_CONFIG_SHIFT   5
#define BQ25896_REG03_CHG_CONFIG_MASK    0x10  // Bit 4: Charge Enable Configuration
#define BQ25896_REG03_CHG_CONFIG_SHIFT   4
#define BQ25896_REG03_SYS_MIN_MASK       0x0E  // Bits 3-1: Minimum System Voltage Limit
#define BQ25896_REG03_SYS_MIN_SHIFT      1
#define BQ25896_REG03_MIN_VBAT_SEL_MASK  0x01  // Bit 0: Minimum Battery Voltage Selection
#define BQ25896_REG03_MIN_VBAT_SEL_SHIFT 0

/**
 * @brief Register 04h (Fast Charge Current Control) bit definitions
 */
#define BQ25896_REG04_EN_PUMPX_MASK      0x80  // Bit 7: Current pulse control Enable
#define BQ25896_REG04_EN_PUMPX_SHIFT     7
#define BQ25896_REG04_ICHG_MASK          0x7F  // Bits 6-0: Fast Charge Current Limit
#define BQ25896_REG04_ICHG_SHIFT         0

/**
 * @brief Register 05h (Pre-Charge/Termination Current Control) bit definitions
 */
#define BQ25896_REG05_IPRECHG_MASK       0xF0  // Bits 7-4: Precharge Current Limit
#define BQ25896_REG05_IPRECHG_SHIFT      4
#define BQ25896_REG05_ITERM_MASK         0x0F  // Bits 3-0: Termination Current Limit
#define BQ25896_REG05_ITERM_SHIFT        0

/**
 * @brief Register 06h (Charge Voltage Control) bit definitions
 */
#define BQ25896_REG06_VREG_MASK          0xFC  // Bits 7-2: Charge Voltage Limit
#define BQ25896_REG06_VREG_SHIFT         2
#define BQ25896_REG06_BATLOWV_MASK       0x02  // Bit 1: Battery Precharge to Fast Charge Threshold
#define BQ25896_REG06_BATLOWV_SHIFT      1
#define BQ25896_REG06_VRECHG_MASK        0x01  // Bit 0: Battery Recharge Threshold
#define BQ25896_REG06_VRECHG_SHIFT       0

/**
 * @brief Register 07h (Charge Termination/Timer Control) bit definitions
 */
#define BQ25896_REG07_EN_TERM_MASK       0x80  // Bit 7: Charging Termination Enable
#define BQ25896_REG07_EN_TERM_SHIFT      7
#define BQ25896_REG07_STAT_DIS_MASK      0x40  // Bit 6: STAT Pin Disable
#define BQ25896_REG07_STAT_DIS_SHIFT     6
#define BQ25896_REG07_WATCHDOG_MASK      0x30  // Bits 5-4: I2C Watchdog Timer Setting
#define BQ25896_REG07_WATCHDOG_SHIFT     4
#define BQ25896_REG07_EN_TIMER_MASK      0x08  // Bit 3: Charging Safety Timer Enable
#define BQ25896_REG07_EN_TIMER_SHIFT     3
#define BQ25896_REG07_CHG_TIMER_MASK     0x06  // Bits 2-1: Fast Charge Timer Setting
#define BQ25896_REG07_CHG_TIMER_SHIFT    1
#define BQ25896_REG07_JEITA_ISET_MASK    0x01  // Bit 0: JEITA Low Temperature Current Setting
#define BQ25896_REG07_JEITA_ISET_SHIFT   0

/**
 * @brief Register 08h (IR Compensation/Thermal Regulation Control) bit definitions
 */
#define BQ25896_REG08_BAT_COMP_MASK      0xE0  // Bits 7-5: IR Compensation Resistor Setting
#define BQ25896_REG08_BAT_COMP_SHIFT     5
#define BQ25896_REG08_VCLAMP_MASK        0x1C  // Bits 4-2: IR Compensation Voltage Clamp
#define BQ25896_REG08_VCLAMP_SHIFT       2
#define BQ25896_REG08_TREG_MASK          0x03  // Bits 1-0: Thermal Regulation Threshold
#define BQ25896_REG08_TREG_SHIFT         0

/**
 * @brief Register 09h (Misc Operation Control) bit definitions
 */
#define BQ25896_REG09_FORCE_ICO_MASK     0x80  // Bit 7: Force Start Input Current Optimizer (ICO)
#define BQ25896_REG09_FORCE_ICO_SHIFT    7
#define BQ25896_REG09_TMR2X_EN_MASK      0x40  // Bit 6: Safety Timer Setting during DPM or Thermal Regulation
#define BQ25896_REG09_TMR2X_EN_SHIFT     6
#define BQ25896_REG09_BATFET_DIS_MASK    0x20  // Bit 5: Force BATFET off to enable ship mode
#define BQ25896_REG09_BATFET_DIS_SHIFT   5
#define BQ25896_REG09_JEITA_VSET_MASK    0x10  // Bit 4: JEITA High Temperature Voltage Setting
#define BQ25896_REG09_JEITA_VSET_SHIFT   4
#define BQ25896_REG09_BATFET_DLY_MASK    0x08  // Bit 3: BATFET turn off delay control
#define BQ25896_REG09_BATFET_DLY_SHIFT   3
#define BQ25896_REG09_BATFET_RST_EN_MASK 0x04  // Bit 2: BATFET full system reset enable
#define BQ25896_REG09_BATFET_RST_EN_SHIFT 2
#define BQ25896_REG09_PUMPX_UP_MASK      0x02  // Bit 1: Current pulse control voltage up enable
#define BQ25896_REG09_PUMPX_UP_SHIFT     1
#define BQ25896_REG09_PUMPX_DN_MASK      0x01  // Bit 0: Current pulse control voltage down enable
#define BQ25896_REG09_PUMPX_DN_SHIFT     0

/**
 * @brief Register 0Ah (Boost Mode Control) bit definitions
 */
#define BQ25896_REG0A_BOOSTV_MASK        0xF0  // Bits 7-4: Boost Mode Voltage Regulation
#define BQ25896_REG0A_BOOSTV_SHIFT       4
#define BQ25896_REG0A_PFM_OTG_DIS_MASK   0x08  // Bit 3: PFM mode allowed in boost mode
#define BQ25896_REG0A_PFM_OTG_DIS_SHIFT  3
#define BQ25896_REG0A_BOOST_LIM_MASK     0x07  // Bits 2-0: Boost Mode Current Limit
#define BQ25896_REG0A_BOOST_LIM_SHIFT    0

/**
 * @brief Register 0Bh (Status Register) bit definitions
 */
#define BQ25896_REG0B_VBUS_STAT_MASK     0xE0  // Bits 7-5: VBUS Status register
#define BQ25896_REG0B_VBUS_STAT_SHIFT    5
#define BQ25896_REG0B_CHRG_STAT_MASK     0x18  // Bits 4-3: Charging Status
#define BQ25896_REG0B_CHRG_STAT_SHIFT    3
#define BQ25896_REG0B_PG_STAT_MASK       0x04  // Bit 2: Power Good Status
#define BQ25896_REG0B_PG_STAT_SHIFT      2
// Bit 1 is Reserved, always reads 1
#define BQ25896_REG0B_VSYS_STAT_MASK     0x01  // Bit 0: VSYS Regulation Status
#define BQ25896_REG0B_VSYS_STAT_SHIFT    0

/**
 * @brief Register 0Ch (Fault Register) bit definitions
 */
#define BQ25896_REG0C_WATCHDOG_FAULT_MASK 0x80  // Bit 7: Watchdog Fault Status
#define BQ25896_REG0C_WATCHDOG_FAULT_SHIFT 7
#define BQ25896_REG0C_BOOST_FAULT_MASK    0x40  // Bit 6: Boost Mode Fault Status
#define BQ25896_REG0C_BOOST_FAULT_SHIFT   6
#define BQ25896_REG0C_CHRG_FAULT_MASK     0x30  // Bits 5-4: Charge Fault Status
#define BQ25896_REG0C_CHRG_FAULT_SHIFT    4
#define BQ25896_REG0C_BAT_FAULT_MASK      0x08  // Bit 3: Battery Fault Status
#define BQ25896_REG0C_BAT_FAULT_SHIFT     3
#define BQ25896_REG0C_NTC_FAULT_MASK      0x07  // Bits 2-0: NTC Fault Status
#define BQ25896_REG0C_NTC_FAULT_SHIFT     0

/**
 * @brief Register 0Dh (VINDPM Register) bit definitions
 */
#define BQ25896_REG0D_FORCE_VINDPM_MASK  0x80  // Bit 7: VINDPM Threshold Setting Method
#define BQ25896_REG0D_FORCE_VINDPM_SHIFT 7
#define BQ25896_REG0D_VINDPM_MASK        0x7F  // Bits 6-0: Absolute VINDPM Threshold
#define BQ25896_REG0D_VINDPM_SHIFT       0

/**
 * @brief Register 0Eh - Battery Voltage
 * 
 * Bits 6-0: BATV - Battery Voltage Reading
 */
#define BQ25896_REG_0E                  0x0E

#define BQ25896_REG0E_BATV_MASK         0x7F
#define BQ25896_REG0E_BATV_SHIFT        0

/**
 * @brief Register 0Fh - System Voltage
 * 
 * Bits 6-0: SYSV - System Voltage Reading
 */
#define BQ25896_REG_0F                  0x0F

#define BQ25896_REG0F_SYSV_MASK         0x7F
#define BQ25896_REG0F_SYSV_SHIFT        0

/**
 * @brief Register 10h - TS Voltage / THERM_STAT
 * 
 * Bit 7: THERM_STAT - Thermal Regulation Status
 * Bits 6-0: TSPCT - TS Voltage as Percentage of REGN
 */
#define BQ25896_REG_10                  0x10

#define BQ25896_REG10_THERM_STAT_MASK   0x80
#define BQ25896_REG10_THERM_STAT_SHIFT  7

#define BQ25896_REG10_TSPCT_MASK        0x7F
#define BQ25896_REG10_TSPCT_SHIFT       0

/**
 * @brief Register 11h - VBUS Voltage
 * 
 * Bit 7: VBUS_GD - VBUS Good Status
 * Bits 6-0: VBUSV - VBUS Voltage Reading
 */
#define BQ25896_REG_11                  0x11

#define BQ25896_REG11_VBUS_GD_MASK      0x80
#define BQ25896_REG11_VBUS_GD_SHIFT     7

#define BQ25896_REG11_VBUSV_MASK        0x7F
#define BQ25896_REG11_VBUSV_SHIFT       0

/**
 * @brief Register 12h - Charge Current
 * 
 * Bits 6-0: ICHGR - Charge Current Reading
 */
#define BQ25896_REG_12                  0x12

#define BQ25896_REG12_ICHGR_MASK        0x7F
#define BQ25896_REG12_ICHGR_SHIFT       0

/**
 * @brief Register 13h - VDPM / IDPM Status
 * 
 * Bit 7: VDPM_STAT - VINDPM or FORCE_VINDPM Status
 * Bit 6: IDPM_STAT - IINDPM Status
 * Bit 5: IDPM_LIM - IINDPM Loop Active
 * Bits 4-0: IDPM_LIM - Current Limit Setting
 */
#define BQ25896_REG_13                  0x13

#define BQ25896_REG13_VDPM_STAT_MASK    0x80
#define BQ25896_REG13_VDPM_STAT_SHIFT   7

#define BQ25896_REG13_IDPM_STAT_MASK    0x40
#define BQ25896_REG13_IDPM_STAT_SHIFT   6

#define BQ25896_REG13_IDPM_LIM_MASK     0x20
#define BQ25896_REG13_IDPM_LIM_SHIFT    5

#define BQ25896_REG13_IDPM_SETTING_MASK     0x1F
#define BQ25896_REG13_IDPM_SETTING_SHIFT    0

/**
 * @brief Register 14h (Device Information Register) bit definitions
 */
#define BQ25896_REG14_REG_RST_MASK       0x80  // Bit 7: Register Reset
#define BQ25896_REG14_REG_RST_SHIFT      7
#define BQ25896_REG14_ICO_OPTIMIZED_MASK 0x40  // Bit 6: Input Current Optimizer Status
#define BQ25896_REG14_ICO_OPTIMIZED_SHIFT 6
#define BQ25896_REG14_PN_MASK            0x38  // Bits 5-3: Device Configuration (000: bq25896)
#define BQ25896_REG14_PN_SHIFT           3
#define BQ25896_REG14_TS_PROFILE_MASK    0x04  // Bit 2: Temperature Profile (1: JEITA)
#define BQ25896_REG14_TS_PROFILE_SHIFT   2
#define BQ25896_REG14_DEV_REV_MASK       0x03  // Bits 1-0: Device Revision (10)
#define BQ25896_REG14_DEV_REV_SHIFT      0

/**
 * @brief ADC conversion functions for voltage, current, and temperature
 */
/* Conversion macros for BQ25896 readings */
/*
 * Battery voltage (REG0E): VBAT = 2304mV + (BATV * 20mV)
 * BATV = Bits 6:0 of REG0E
 */
#define BQ25896_VBAT_CONV(reg_val)     (2304 + ((reg_val) * 20))

/*
 * System voltage (REG0F): VSYS = 2304mV + (SYSV * 20mV)
 * SYSV = Bits 6:0 of REG0F
 */
#define BQ25896_VSYS_CONV(reg_val)     (2304 + ((reg_val) * 20))

/*
 * VBUS voltage (REG11): VBUS = 2600mV + (VBUSV * 100mV)
 * VBUSV = Bits 6:0 of REG11
 */
#define BQ25896_VBUS_CONV(reg_val)     (2600 + ((reg_val) * 100))

/*
 * Charge current (REG12): ICHG = ICHGR * 50mA
 * ICHGR = Bits 6:0 of REG12
 */
#define BQ25896_ICHG_CONV(reg_val)     ((reg_val) * 50)

/*
 * Temperature calculation based on TS percentage (REG10)
 * TSPCT = Bits 6:0 of REG10
 * 60% corresponds to approximately 20°C
 * Above 60%: higher percentage = lower temperature (60% = 20°C, 70% = 0°C, 80% = -20°C)
 * Below 60%: lower percentage = higher temperature (60% = 20°C, 50% = 40°C, 40% = 60°C)
 */
#define BQ25896_TEMP_CONV(ts_pct) \
    ((ts_pct) == 60 ? 20 : \
     (ts_pct) > 60 ? (20 - (((ts_pct) - 60) * 2)) : \
                     (20 + ((60 - (ts_pct)) * 2)))

/**
 * @brief Converts desired charge current to register value
 * Step size: 64mA, range: 0-5056mA
 */
#define BQ25896_ICHG_TO_REG(current_ma) ((current_ma) / 64)

/**
 * @brief Converts desired charge voltage to register value
 * Step size: 16mV, base: 3840mV, range: 3840-4608mV
 */
#define BQ25896_VREG_TO_REG(voltage_mv) (((voltage_mv) - 3840) / 16)

/**
 * @brief Converts desired precharge current to register value
 * Step size: 64mA, range: 64-1024mA
 */
#define BQ25896_IPRECH_TO_REG(current_ma) (((current_ma) / 64) - 1)

/**
 * @brief Converts desired termination current to register value
 * Step size: 64mA, range: 64-1024mA
 */
#define BQ25896_ITERM_TO_REG(current_ma) (((current_ma) / 64) - 1)

/**
 * @brief Converts desired VINDPM threshold to register value
 * Step size: 100mV, base: 3900mV
 */
#define BQ25896_VINDPM_TO_REG(voltage_mv) (((voltage_mv) - 3900) / 100)

/**
 * @brief Converts desired min system voltage to register value
 * Step size: 100mV, base: 3000mV, range: 3000-3700mV
 */
#define BQ25896_SYS_MIN_TO_REG(voltage_mv) (((voltage_mv) - 3000) / 100)

/**
 * @brief BQ25896 device structure
 */
struct bq25896_dev_t {
    i2c_master_bus_handle_t i2c_bus;         /*!< I2C bus handle */
    uint8_t dev_addr;                         /*!< Device I2C address */
    bq25896_config_t config;                  /*!< Current configuration */
};

#ifdef __cplusplus
}
#endif 