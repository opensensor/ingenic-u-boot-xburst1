# ATBM6441 WiFi Watchdog Solution

## Problem Summary

The ATBM6441 WiFi chip has an internal MCU with a hardware watchdog timer that reboots the system after approximately 5-6 seconds if not fed with keepalive messages. This prevents the system from booting Linux.

## CONFIRMED: Watchdog is in WiFi Chip, NOT PMIC

**Test Result:** We successfully enabled I2C in U-Boot and read the D2041 PMIC CONTROLD register (0x11). The value is 0x00, meaning the PMIC watchdog is already disabled. **The system still reboots after ~5 seconds**, confirming the watchdog is in the ATBM6441 WiFi chip's MCU, not the PMIC.

## Root Cause Analysis

After extensive investigation, we discovered:

1. **The ATBM6441 has an internal MCU** that requires firmware to be loaded
2. **The watchdog is in the MCU** and cannot be fed until firmware is loaded
3. **The SDIO interface is non-functional** until the MCU firmware is loaded
4. **The firmware file is `mcu_fw.bin`** (339,480 bytes) located in `/mcu_fw/` in the filesystem
5. **No GPIO power control** - the chip is always powered by the D2041 PMIC

## Two Solution Approaches

### Solution A: Disable PMIC Watchdog (EASIEST - TRY THIS FIRST!)

**File:** `dist/uboot/u-boot-PMIC-WDT-DISABLE.bin`

The D2041 PMIC has a watchdog control bit in register 0x11 (CONTROLD), bit 7.

**What it does:**
- Reads PMIC CONTROLD register (0x11) via I2C
- Clears bit 7 to disable the watchdog
- If successful, system stays alive indefinitely

**Test this first!** If the watchdog is actually in the PMIC (not the WiFi chip), this will solve the problem immediately.

### Solution B: Load Firmware in U-Boot (COMPLEX)

**Status:** Firmware embedded, loader implementation in progress

The firmware loading process requires:

1. **Initialize chip registers** (`atbm_before_load_firmware`)
   - Read/write SDIO function registers
   - Configure control bits
   - Poll for ready status

2. **Upload firmware** (339KB via SDIO)
   - Write to chip memory via AHB bus
   - Must complete within ~6 seconds

3. **Finalize configuration** (`atbm_after_load_firmware`)
   - Set final control bits
   - Enable SDIO function

**Firmware location:** `board/ingenic/isvp_t23/firmware/mcu_fw.bin`
**Embedded header:** `board/ingenic/isvp_t23/firmware/mcu_fw.h`

## Testing Instructions

### Test 1: PMIC Watchdog Disable
```bash
# Flash this version first
dist/uboot/u-boot-PMIC-WDT-DISABLE.bin

# Expected output if successful:
# "ATBM: SUCCESS! PMIC watchdog disabled!"
# "System alive for X seconds (watchdog disabled!)"
# System should stay alive indefinitely
```

### Test 2: Firmware Loading (if Test 1 fails)
```bash
# This requires implementing the full firmware loader
# See atbm_keepalive_v3.c for implementation details
```

## Key Discoveries

1. **PMIC has watchdog control** - D2041_CONTROLD_REG (0x11), bit 7
2. **Firmware format** - Starts with "ZG" signature (0x5a 0x47)
3. **Firmware size** - 339,480 bytes (332 KB)
4. **No EEPROM firmware** - Firmware is in filesystem, not external EEPROM
5. **SDIO enumeration fails** - CMD5, CMD52 all timeout until firmware loads

## References

- **Decompiled driver functions:**
  - `atbm_before_load_firmware` - Chip initialization
  - `atbm_after_load_firmware` - Finalization
  - `atbm_load_firmware` - Main loader
  - `atbm_wtd_wakeup` - Watchdog control

- **PMIC registers:**
  - `D2041_CONTROLD_REG` (0x11) - Control register D
  - `D2041_CONTROLD_WATCHDOG` (bit 7) - Watchdog enable/disable

- **Firmware file:**
  - Path: `/mcu_fw/mcu_fw.bin`
  - Version: 1.2.5u1
  - Size: 339,480 bytes

## Test Results Summary

| Approach | Status | Result |
|----------|--------|--------|
| PMIC watchdog disable | ✅ TESTED | PMIC watchdog already disabled (reg=0x00), system still reboots |
| I2C communication | ✅ WORKING | Successfully read/write PMIC via soft I2C (GPIO 90/91) |
| SDIO enumeration | ❌ FAILED | Device doesn't respond until firmware loaded |
| Firmware loading in U-Boot | 🚧 TOO COMPLEX | Requires extensive SDIO register manipulation |

## Recommended Solution: Optimize Linux Boot

After testing, we found:
- ✅ I2C is now working in U-Boot (soft I2C on GPIO PC26/PC27)
- ❌ PMIC watchdog is already disabled - not the source of reboots
- ❌ Firmware loading in U-Boot is too complex for 5-second window
- ✅ **Best approach: Make Linux boot faster**

### How to Speed Up Linux Boot

1. **Minimize kernel size** - Remove unnecessary drivers
2. **Use initramfs** - Embed ATBM driver in initramfs
3. **Optimize init** - Load WiFi driver immediately
4. **Reduce delays** - Remove unnecessary sleeps in boot process

### Example initramfs `/init`:
```bash
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sysfs /sys
# Load ATBM driver IMMEDIATELY (within 6 seconds)
insmod /lib/modules/atbm6441_wifi_sdio.ko
sleep 1
exec /sbin/init
```

## Next Steps

1. **Measure current boot time** - How long until driver loads?
2. **Optimize kernel config** - Remove unnecessary drivers
3. **Create minimal initramfs** - Load WiFi driver first
4. **Test boot time** - Ensure driver loads within 6 seconds

## Build Commands

```bash
# PMIC watchdog disable version
export PATH=/home/matteius/output/t23zn_sc2335_atbm6441/host/bin:$PATH
cd /home/matteius/atbm/ingenic-u-boot-xburst1
make isvp_t23n_sfcnor_msc1 -j$(nproc)
cp u-boot-lzo-with-spl.bin /home/matteius/atbm/dist/uboot/u-boot-PMIC-WDT-DISABLE.bin
```

## Success Criteria

- **PMIC disable success:** System stays alive >10 seconds at U-Boot prompt
- **Firmware load success:** SDIO device responds to CMD5, system boots Linux
- **Ultimate success:** System boots Linux and WiFi driver loads successfully

