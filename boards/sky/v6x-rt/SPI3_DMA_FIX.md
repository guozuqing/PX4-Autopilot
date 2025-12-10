# SPI3 DMA Hard Fault Fix

## Problem Summary

When `CONFIG_IMXRT_LPSPI3_DMA=y` was enabled, the system experienced continuous hard faults and reboots with the following signature:

```
Type: Hard Fault in file:armv7-m/arm_memfault.c at line: 101
Running task: wq:SPI3
PC: 0xfffffffe (invalid - corrupted return address)
LR: 0x301ba925
```

## Root Cause

**Critical bug in NuttX LPSPI DMA driver** at line 1578 of `imxrt_lpspi.c`:

```c
// WRONG - multiplies by 4 instead of 2
nbytes = (priv->nbits > 8) ? nwords << 2 : nwords;
```

For 16-bit SPI transfers (used by SCH16T IMU):
- **Bug**: Shifts left by 2 bits (`<< 2`) = multiply by 4
- **Correct**: Should shift left by 1 bit (`<< 1`) = multiply by 2

### Impact

This caused DMA to transfer **double the required bytes**:
1. **TX DMA** reads past the end of TX buffer → stack corruption
2. **RX DMA** writes past the end of RX buffer → stack corruption
3. Return address on stack gets overwritten
4. PC register becomes corrupted to `0xfffffffe`
5. Hard fault and system crash/reboot

## Fix Applied

### 1. Fixed Byte Calculation (Critical)

**File**: `/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c`
**Line**: 1578

```c
// BEFORE (WRONG):
nbytes = (priv->nbits > 8) ? nwords << 2 : nwords;

// AFTER (CORRECT):
nbytes = (priv->nbits > 8) ? nwords << 1 : nwords;
```

This fixed the buffer overrun that caused system crashes, but DMA still couldn't read data correctly.

### 2. Fixed DMA Iteration Count (Critical - Fixed Data Transfer)

**File**: `/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c`
**Lines**: 1629, 1643

```c
// BEFORE (WRONG):
config.iter   = nbytes;   // nbytes = nwords × 2
config.nbytes = adjust;   // adjust = 2 for 16-bit
// Result: DMA transfers (nwords × 2) × 2 = nwords × 4 bytes ❌

// AFTER (CORRECT):
config.iter   = nwords;   // Major loop iterations
config.nbytes = adjust;   // Bytes per iteration
// Result: DMA transfers nwords × 2 bytes ✓
```

The eDMA `iter` field represents the number of major loop iterations, not total bytes. Each iteration transfers `nbytes` bytes.

### 3. Increased Work Queue Stack Size (Safety Measure)

**File**: `/boards/sky/v6x-rt/nuttx-config/nsh/defconfig`
**Line**: 261

```
// BEFORE:
CONFIG_SCHED_HPWORKSTACKSIZE=1800

// AFTER:
CONFIG_SCHED_HPWORKSTACKSIZE=2400
```

Increased from 1800 to 2400 bytes to provide additional safety margin for SPI DMA operations.

## Technical Details

### DMA Transfer Size Calculation

For SPI transfers, byte count must match word size:
- **8-bit transfers**: `nbytes = nwords × 1`
- **16-bit transfers**: `nbytes = nwords × 2` ✓ (FIXED in step 1)
- **32-bit transfers**: `nbytes = nwords × 4`

### eDMA Configuration

The iMXRT eDMA controller uses:
- `config.iter`: Number of major loop iterations
- `config.nbytes`: Bytes to transfer per iteration
- **Total transfer** = `iter × nbytes`

**Correct configuration for 16-bit SPI:**
```c
config.iter   = nwords;  // Number of 16-bit words
config.nbytes = 2;       // 2 bytes per 16-bit word
// Total = nwords × 2 bytes ✓
```

**Incorrect configuration (original bug):**
```c
config.iter   = nbytes;  // nbytes already = nwords × 2
config.nbytes = 2;       // 2 bytes per iteration
// Total = (nwords × 2) × 2 = nwords × 4 bytes ❌
```

### Why This Caused Stack Corruption

The SCH16T IMU driver uses 16-bit SPI transfers. With the bug:

1. Driver allocates buffer for N words (N × 2 bytes)
2. DMA incorrectly configured for N × 4 bytes
3. DMA reads/writes 2× the buffer size
4. Overruns into adjacent stack memory
5. Corrupts return addresses and other stack data
6. Crashes when function returns to invalid address

### Cache Coherency

The board uses write-through cache (`CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y`). The NuttX driver correctly handles cache:
- `up_clean_dcache()` before TX DMA
- `up_invalidate_dcache()` after RX DMA

## Testing

After applying this fix:
1. Rebuild the firmware: `make sky_v6x-rt_default`
2. Flash and verify SPI3 sensor (SCH16T) initializes without faults
3. Verify continuous operation without reboots
4. Monitor `hardfault_log` to confirm no further crashes

## Affected Configurations

This bug affects any iMXRT117x board using:
- LPSPI with DMA enabled
- 16-bit SPI transfers
- Similar boards: fmu-v6x, any iMXRT-based flight controllers

## Related Files

- `/platforms/nuttx/NuttX/nuttx/arch/arm/src/imxrt/imxrt_lpspi.c` - Driver fix
- `/boards/sky/v6x-rt/nuttx-config/nsh/defconfig` - Stack size increase
- `/boards/sky/v6x-rt/src/spi.cpp` - SPI3 configuration (SCH16T sensor)
- `/src/drivers/imu/murata/sch16t/SCH16T.cpp` - SCH16T IMU driver

## Date

Fix applied: 2024-12-02
