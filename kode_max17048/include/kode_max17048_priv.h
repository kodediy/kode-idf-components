#pragma once

#include <stdint.h>
#include "kode_max17048.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAX17048 device structure
 */
typedef struct max17048_dev_t {
    i2c_master_bus_handle_t i2c_bus;        // I2C bus handle
    uint8_t dev_addr;                       // Device I2C address
    max17048_config_t config;               // Device configuration
} max17048_dev_t;

/* Register Addresses */
#define MAX17048_VCELL_REG                 0x02  /* ADC measurement of VCELL, 78.125µV/cell, Read-only */
#define MAX17048_SOC_REG                   0x04  /* Battery state of charge, 1%/256, Read-only */
#define MAX17048_MODE_REG                  0x06  /* Quick-start, hibernate, sleep modes, Write-only, Default: 0x0000 */
#define MAX17048_VERSION_REG               0x08  /* IC production version, Read-only, Default: 0x001 */
#define MAX17048_HIBRT_REG                 0x0A  /* Hibernate thresholds control, Read/Write, Default: 0x8030 */
#define MAX17048_CONFIG_REG                0x0C  /* Configuration register, Read/Write, Default: 0x971C */
#define MAX17048_VALRT_REG                 0x14  /* VCELL range alerts, Read/Write, Default: 0x00FF */
#define MAX17048_CRATE_REG                 0x16  /* Charge/discharge rate, 0.208%/hr, Read-only */
#define MAX17048_VRESET_ID_REG             0x18  /* VCELL reset threshold & ID, Read/Write, Default: 0x96__ */
#define MAX17048_STATUS_REG                0x1A  /* Status flags register, Read/Write, Default: 0x01__ */
#define MAX17048_TABLE_START_REG           0x40  /* Start of custom model table, Write-only */
#define MAX17048_TABLE_END_REG             0x7F  /* End of custom model table */
#define MAX17048_TABLE_UNLOCK_1_REG        0x3E  /* First unlock register */
#define MAX17048_TABLE_UNLOCK_2_REG        0x3F  /* Second unlock register */
#define MAX17048_CMD_REG                   0xFE  /* Command register, Read/Write, Default: 0xFFFF */

/* Register value calculation macros */
#define MAX17048_VCELL_TO_MV(reg)         ((float)(reg) * MAX17048_VCELL_RESOLUTION * 1000.0f)
#define MAX17048_SOC_TO_PERCENT(reg)      ((float)(reg) * MAX17048_SOC_RESOLUTION)
#define MAX17048_CRATE_TO_PERCENT(reg)    ((float)(reg) * MAX17048_CRATE_STEP_PCTH)

/* VCELL Register (0x02) */
#define MAX17048_VCELL_STEP_UV            78.125f /* VCELL LSB: 78.125µV per bit */
#define MAX17048_VCELL_MAX_MV             5120   /* Maximum readable voltage: 5.12V */
#define MAX17048_VCELL_RESOLUTION         0.078125f  /* 78.125µV per bit (same as STEP_UV but in V) */

/* SOC Register (0x04) */
#define MAX17048_SOC_INT_MASK             0xFF00 /* Upper byte for integer % */
#define MAX17048_SOC_DEC_MASK             0x00FF /* Lower byte for decimal % */
#define MAX17048_SOC_INT_SHIFT            8      /* Shift for integer part */
#define MAX17048_SOC_RESOLUTION           0.00390625f /* 1/256% per bit */

/* MODE Register (0x06) */
#define MAX17048_MODE_QUICKSTART_BIT       (1 << 14)  /* Quick-Start bit - MSB[14] */
#define MAX17048_MODE_ENSLEEP_BIT          (1 << 13)  /* Enable Sleep Mode bit - MSB[13] */
#define MAX17048_MODE_HIBSTAT_BIT          (1 << 12)  /* Hibernate Status bit (read-only) - MSB[12] */

/* VERSION Register (0x08) */
#define MAX17048_VERSION_DEFAULT           0x001 /* Expected version number */

/* Register Definitions */
#define MAX17048_HIBRT_DISABLE             0x0000  /* Disable hibernate mode */
#define MAX17048_HIBRT_ALWAYS_HIBERNATE    0xFFFF  /* Always use hibernate mode */
#define MAX17048_HIBRT_DEFAULT             0x8030  /* Default value */

/* HIBRT Register Masks and Shifts */
#define MAX17048_HIBRT_HIB_THR_MASK        0xFF00  /* Hibernate threshold mask (MSB) */
#define MAX17048_HIBRT_ACT_THR_MASK        0x00FF  /* Active threshold mask (LSB) */
#define MAX17048_HIBRT_HIB_THR_SHIFT       8       /* Shift for hibernate threshold */

/* HIBRT Register Resolution */
#define MAX17048_HIBRT_ACT_THR_STEP_MV     1.25f   /* ActThr LSB = 1.25mV */
#define MAX17048_HIBRT_HIB_THR_STEP_PCTH   0.208f  /* HibThr LSB = 0.208%/hr */

/* CONFIG Register (0x0C) */
#define MAX17048_CONFIG_RCOMP_DEFAULT      0x97  /* Default RCOMP value */
#define MAX17048_CONFIG_ATHD_DEFAULT       0x1C  /* Default empty alert threshold (4%) */
#define MAX17048_CONFIG_DEFAULT            ((MAX17048_CONFIG_RCOMP_DEFAULT << 8) | MAX17048_CONFIG_ATHD_DEFAULT)

/* CONFIG Register Bit Positions */
#define MAX17048_CONFIG_RCOMP_MASK         0xFF00  /* RCOMP mask (MSB) */
#define MAX17048_CONFIG_SLEEP_BIT          (1 << 7)  /* Sleep bit */
#define MAX17048_CONFIG_ALSC_BIT           (1 << 6)  /* SOC change alert bit */
#define MAX17048_CONFIG_ALRT_BIT           (1 << 5)  /* Alert status bit */
#define MAX17048_CONFIG_ATHD_MASK          0x001F  /* Empty alert threshold mask (5 bits) */

/* CONFIG Register Shifts */
#define MAX17048_CONFIG_RCOMP_SHIFT        8     /* RCOMP value shift */

/**
 * @brief Convert ATHD value to percentage
 * @param athd ATHD value (0-31)
 * @return Corresponding percentage (1-32)
 */
#define MAX17048_ATHD_TO_PERCENT(athd)     (32 - (athd))

/**
 * @brief Convert percentage to ATHD value
 * @param percent Desired percentage (1-32)
 * @return Corresponding ATHD value (0-31)
 */
#define MAX17048_PERCENT_TO_ATHD(percent)  (32 - (percent))

/* VALRT Register (0x14) */
#define MAX17048_VALRT_MAX_MASK            0xFF00  /* Maximum voltage threshold mask (MSB) */
#define MAX17048_VALRT_MIN_MASK            0x00FF  /* Minimum voltage threshold mask (LSB) */
#define MAX17048_VALRT_MAX_SHIFT           8       /* Shift for maximum threshold */

/* VALRT Register Resolution */
#define MAX17048_VALRT_STEP_MV             20.0f   /* VALRT LSB = 20mV */
#define MAX17048_VALRT_DEFAULT             0x00FF  /* Default value - no alerts */

/**
 * @brief Convert millivolts to VALRT register value
 * @param mv Voltage in millivolts
 * @return Corresponding VALRT value
 */
#define MAX17048_MV_TO_VALRT(mv)          ((uint8_t)((mv) / MAX17048_VALRT_STEP_MV))

/**
 * @brief Convert VALRT register value to millivolts
 * @param valrt VALRT register value
 * @return Corresponding voltage in millivolts
 */
#define MAX17048_VALRT_TO_MV(valrt)       ((float)(valrt) * MAX17048_VALRT_STEP_MV)

/* CRATE Register (0x16) */
#define MAX17048_CRATE_STEP_PCTH          0.208f  /* CRATE LSB = 0.208% per hour */

/**
 * @brief Convert CRATE register value to percent per hour
 * @param crate CRATE register value
 * @return Rate in percent per hour
 */
#define MAX17048_CRATE_TO_PCTH(crate)     ((float)(int16_t)(crate) * MAX17048_CRATE_STEP_PCTH)

/* VRESET/ID Register (0x18) */
#define MAX17048_VRESET_MASK               0xFE00  /* VRESET[7:1] mask */
#define MAX17048_VRESET_DIS_BIT            (1 << 8)  /* Disable analog comparator bit */
#define MAX17048_ID_MASK                   0x00FF  /* Factory-programmed ID mask */

/* VRESET/ID Register Resolution and Defaults */
#define MAX17048_VRESET_STEP_MV            40.0f   /* VRESET LSB = 40mV */
#define MAX17048_VRESET_DEFAULT_CAPTIVE    0x3E    /* 2.5V for captive batteries */
#define MAX17048_VRESET_MIN_REMOVABLE_MV   300     /* Minimum offset below empty voltage for removable batteries */

/* VRESET Timing */
#define MAX17048_VRESET_COMP_DELAY_MS      1    /* Reset delay with comparator enabled */
#define MAX17048_VRESET_ADC_DELAY_MS       250  /* Reset delay with comparator disabled */

/**
 * @brief Convert millivolts to VRESET register value
 * @param mv Voltage in millivolts
 * @return Corresponding VRESET value
 */
#define MAX17048_MV_TO_VRESET(mv)         ((uint8_t)((mv) / MAX17048_VRESET_STEP_MV))

/**
 * @brief Convert VRESET register value to millivolts
 * @param vreset VRESET register value
 * @return Corresponding voltage in millivolts
 */
#define MAX17048_VRESET_TO_MV(vreset)     ((float)(vreset) * MAX17048_VRESET_STEP_MV)

/* STATUS Register (0x1A) */
#define MAX17048_STATUS_ENVR_BIT           (1 << 14)  /* Enable voltage reset alert */
#define MAX17048_STATUS_SC_BIT             (1 << 13)  /* 1% SOC change alert */
#define MAX17048_STATUS_HD_BIT             (1 << 12)  /* SOC low alert */
#define MAX17048_STATUS_VR_BIT             (1 << 11)  /* Voltage reset alert */
#define MAX17048_STATUS_VL_BIT             (1 << 10)  /* Voltage low alert */
#define MAX17048_STATUS_VH_BIT             (1 << 9)   /* Voltage high alert */
#define MAX17048_STATUS_RI_BIT             (1 << 8)   /* Reset indicator */

/* Status Flag Clearing Helper Macros */
#define MAX17048_CLEAR_ALL_ALERTS          (MAX17048_STATUS_SC_BIT | \
                                           MAX17048_STATUS_HD_BIT | \
                                           MAX17048_STATUS_VR_BIT | \
                                           MAX17048_STATUS_VL_BIT | \
                                           MAX17048_STATUS_VH_BIT | \
                                           MAX17048_STATUS_RI_BIT)

#define MAX17048_CLEAR_VOLTAGE_ALERTS      (MAX17048_STATUS_VH_BIT | \
                                           MAX17048_STATUS_VL_BIT | \
                                           MAX17048_STATUS_VR_BIT)

#define MAX17048_CLEAR_SOC_ALERTS          (MAX17048_STATUS_SC_BIT | \
                                           MAX17048_STATUS_HD_BIT)

/* TABLE Unlock/Lock Registers and Values */
#define MAX17048_TABLE_UNLOCK_1_VALUE      0x4A  /* First unlock value */
#define MAX17048_TABLE_UNLOCK_2_VALUE      0x57  /* Second unlock value */
#define MAX17048_TABLE_LOCK_VALUE          0x00  /* Value to lock table registers */

/* CMD Register (0xFE) */
#define MAX17048_CMD_POR_VALUE             0x5400  /* Power-On Reset command value */

#ifdef __cplusplus
}
#endif 