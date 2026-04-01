# NuttX Patches for Sky v6x-rt Board

This directory contains patches that must be applied to NuttX to fix critical bugs.

## Patches

### 0001-Fix-LPSPI-DMA-buffer-overrun.patch

**Purpose**: Fix critical LPSPI DMA bugs causing buffer overruns and system crashes

**Issues Fixed**:
1. Incorrect byte calculation for 16-bit SPI transfers (multiplied by 4 instead of 2)
2. Incorrect DMA iteration count (used total bytes instead of word count)

**Impact**:
- Fixes hard faults and continuous reboots when using SPI3 with DMA
- Fixes data corruption (reading 0xFFFF instead of valid data)
- Required for SCH16T IMU sensor on SPI3

**Target File**: `arch/arm/src/imxrt/imxrt_lpspi.c`

**Lines Modified**:
- Line 1578: `nwords << 2` → `nwords << 1`
- Line 1629: `config.iter = nbytes` → `config.iter = nwords`
- Line 1643: `config.iter = nbytes` → `config.iter = nwords`

## How to Apply Patches

If NuttX submodule is updated and patches are lost, re-apply them:

```bash
cd platforms/nuttx/NuttX/nuttx
git apply ../../../../boards/sky/T900/nuttx-patches/0001-Fix-LPSPI-DMA-buffer-overrun.patch
```

Or from PX4-Autopilot root:

```bash
cd platforms/nuttx/NuttX/nuttx
git apply ../../../boards/sky/T900/nuttx-patches/0001-Fix-LPSPI-DMA-buffer-overrun.patch
```

## Verify Patches Are Applied

Check the file content:

```bash
grep "nwords << 1" platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c
```

Should output line 1578 with `<< 1` (not `<< 2`).

## Upstream Status

These patches should be submitted to NuttX upstream to benefit all iMXRT users.

**Affected Hardware**:
- All iMXRT117x based boards
- Any board using LPSPI with DMA and 16-bit transfers
- Known affected: FMU-v6XRT, Sky v6x-rt

## Additional Information

See `SPI3_DMA_修复说明.md` for detailed technical explanation in Chinese.
See `SPI3_DMA_FIX.md` for detailed technical explanation in English.
