#!/usr/bin/env python3
"""
Idempotently patch CubeMX-generated files so our app stays bootloader-friendly
and has proper DMA buffer placement for STM32H7.

Flash layout (must match bl_config.h and kvbl_tool.py):
  Bootloader:  0x08000000 - 0x0801FFFF (128K) - not mapped here
  Application: 0x08020000 - 0x080BFFFF (640K) - FLASH region
  Parameters:  0x080C0000 - 0x080DFFFF (128K) - PARAM_FLASH region
  BL Metadata: 0x080E0000 - 0x080FFFFF (128K) - reserved for bootloader

Linker script changes:
1) FLASH origin -> 0x08020000, length -> 0x000A0000 (640K, stops before params).
2) Add PARAM_FLASH region at 0x080C0000 (128K) for parameter storage.
3) Add linker symbols for parameter flash region access from C code.
4) Ensure a .dma_buffer (NOLOAD, 32-byte aligned) section mapped to RAM_D1.
5) Ensure a .bdma_buffer (NOLOAD, 32-byte aligned) section mapped to RAM_D3.

FATFS changes:
6) Place SDFatFS and SDFile in .dma_buffer section (SDMMC DMA can't access DTCMRAM).
7) Invert SD detect logic: != GPIO_PIN_RESET -> == GPIO_PIN_RESET
   (Hardware: switch grounds PD4 when card NOT present, floats HIGH when present)

Run this after every CubeMX regeneration.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LD_PATH = ROOT / "STM32H723XG_FLASH.ld"
FATFS_PATH = ROOT / "FATFS" / "App" / "fatfs.c"
FATFS_PLATFORM_PATH = ROOT / "FATFS" / "Target" / "fatfs_platform.c"

# Application flash (640 KB) - stops before parameter sector
FLASH_ORIGIN = "0x08020000"
FLASH_LENGTH = "0xA0000"

# Parameter storage flash (128 KB) - Sector 6
PARAM_FLASH_ORIGIN = "0x080C0000"
PARAM_FLASH_LENGTH = "0x20000"

DMA_SECTION = (
    "  .dma_buffer (NOLOAD) : ALIGN(32)\n"
    "  {\n"
    "    *(.dma_buffer)\n"
    "    *(.dma_buffer*)\n"
    "    . = ALIGN(32);\n"
    "  } >RAM_D1\n\n"
)
BDMA_SECTION = (
    "  .bdma_buffer (NOLOAD) : ALIGN(32)\n"
    "  {\n"
    "    *(.bdma_buffer)\n"
    "    *(.bdma_buffer*)\n"
    "    . = ALIGN(32);\n"
    "  } >RAM_D3\n\n"
)

# Parameter flash region definition (inserted into MEMORY block)
PARAM_FLASH_REGION = f"PARAM_FLASH (r)   : ORIGIN = {PARAM_FLASH_ORIGIN}, LENGTH = {PARAM_FLASH_LENGTH}   /* 128K parameters */"

# Linker symbols for parameter flash (inserted after MEMORY block)
PARAM_FLASH_SYMBOLS = """
/* Parameter storage flash region symbols */
_param_flash_start = ORIGIN(PARAM_FLASH);
_param_flash_end   = ORIGIN(PARAM_FLASH) + LENGTH(PARAM_FLASH);
_param_flash_size  = LENGTH(PARAM_FLASH);
"""

# Flash layout comment (inserted before MEMORY block)
FLASH_LAYOUT_COMMENT = """/* Flash layout:
 *   Bootloader:  0x08000000 - 0x0801FFFF (128K) - not mapped here
 *   Application: 0x08020000 - 0x080BFFFF (640K) - FLASH region
 *   Parameters:  0x080C0000 - 0x080DFFFF (128K) - PARAM_FLASH region
 *   BL Metadata: 0x080E0000 - 0x080FFFFF (128K) - reserved for bootloader
 */
"""


def patch_flash_region(text: str) -> tuple[str, bool]:
    # Match FLASH line with optional trailing comment (consume entire line to avoid duplicate comments)
    pattern = re.compile(r"FLASH\s*\(rx\)\s*:\s*ORIGIN\s*=\s*0x[0-9A-Fa-f]+,\s*LENGTH\s*=\s*0x[0-9A-Fa-f]+[^\n]*")
    replacement = f"FLASH (rx)      : ORIGIN = {FLASH_ORIGIN}, LENGTH = {FLASH_LENGTH}   /* 640K application */"
    new_text, count = pattern.subn(replacement, text, count=1)
    return new_text, count > 0


def ensure_param_flash_region(text: str) -> tuple[str, bool]:
    """Add PARAM_FLASH region to MEMORY block if not present."""
    if "PARAM_FLASH" in text:
        return text, False
    # Find the closing brace of the MEMORY block
    # Look for the FLASH line and insert PARAM_FLASH after it
    pattern = re.compile(r"(FLASH\s*\(rx\)\s*:\s*ORIGIN\s*=\s*0x[0-9A-Fa-f]+,\s*LENGTH\s*=\s*0x[0-9A-Fa-f]+[^\n]*\n)")
    match = pattern.search(text)
    if not match:
        return text, False
    insert_pos = match.end()
    new_text = text[:insert_pos] + PARAM_FLASH_REGION + "\n" + text[insert_pos:]
    return new_text, True


def ensure_param_flash_symbols(text: str) -> tuple[str, bool]:
    """Add linker symbols for parameter flash after the MEMORY block."""
    if "_param_flash_start" in text:
        return text, False
    # Find the closing brace of MEMORY block followed by content
    # Insert symbols after the MEMORY block's closing brace
    pattern = re.compile(r"(MEMORY\s*\{[^}]+\})")
    match = pattern.search(text)
    if not match:
        return text, False
    insert_pos = match.end()
    new_text = text[:insert_pos] + PARAM_FLASH_SYMBOLS + text[insert_pos:]
    return new_text, True


def ensure_flash_layout_comment(text: str) -> tuple[str, bool]:
    """Add flash layout comment before MEMORY block if not present."""
    if "Flash layout:" in text:
        return text, False
    # Find MEMORY keyword and insert comment before it
    pattern = re.compile(r"(/\* Specify the memory areas \*/\s*\n)?MEMORY")
    match = pattern.search(text)
    if not match:
        return text, False
    # Replace the matched section with comment + MEMORY
    insert_pos = match.start()
    end_pos = match.end()
    # Check if there's already a "Specify the memory areas" comment
    if match.group(1):
        # Replace the old comment with our flash layout comment
        new_text = text[:insert_pos] + FLASH_LAYOUT_COMMENT + "MEMORY" + text[end_pos:]
    else:
        new_text = text[:insert_pos] + FLASH_LAYOUT_COMMENT + text[insert_pos:]
    return new_text, True


def ensure_dma_section(text: str) -> tuple[str, bool]:
    if ".dma_buffer" in text:
        return text, False
    marker = "  /* User_heap_stack section"
    idx = text.find(marker)
    if idx == -1:
        return text, False
    insert_pos = idx
    new_text = text[:insert_pos] + DMA_SECTION + text[insert_pos:]
    return new_text, True


def ensure_bdma_section(text: str) -> tuple[str, bool]:
    if ".bdma_buffer" in text:
        return text, False
    marker = "  /* User_heap_stack section"
    idx = text.find(marker)
    if idx == -1:
        return text, False
    insert_pos = idx
    new_text = text[:insert_pos] + BDMA_SECTION + text[insert_pos:]
    return new_text, True


def patch_fatfs_dma_buffers(text: str) -> tuple[str, bool]:
    """
    Patch SDFatFS and SDFile declarations to use .dma_buffer section.
    STM32H7 SDMMC IDMA cannot access DTCMRAM, so these must be in RAM_D1.
    """
    changed = False

    # Pattern to match plain declarations (without attributes)
    # CubeMX generates: "FATFS SDFatFS;    /* comment */"
    # We want to replace with attribute, keeping the comment
    # Skip if already has __attribute__
    fatfs_pattern = re.compile(
        r'^(FATFS\s+SDFatFS)\s*;(\s*/\*.*?\*/)?',
        re.MULTILINE
    )
    if fatfs_pattern.search(text) and '__attribute__' not in text.split('FATFS SDFatFS')[1].split('\n')[0]:
        text = fatfs_pattern.sub(
            r'\1 __attribute__((section(".dma_buffer"), aligned(32)));\2',
            text
        )
        changed = True

    fil_pattern = re.compile(
        r'^(FIL\s+SDFile)\s*;(\s*/\*.*?\*/)?',
        re.MULTILINE
    )
    if fil_pattern.search(text) and '__attribute__' not in text.split('FIL SDFile')[1].split('\n')[0]:
        text = fil_pattern.sub(
            r'\1 __attribute__((section(".dma_buffer"), aligned(32)));\2',
            text
        )
        changed = True

    return text, changed


def patch_sd_detect_logic(text: str) -> tuple[str, bool]:
    """
    Patch SD detect logic for inverted hardware.
    Hardware: switch grounds PD4 when card NOT present, floats HIGH when present.
    CubeMX generates: LOW = present, HIGH = not present (opposite of our hardware).
    """
    # Already patched if we see "== GPIO_PIN_RESET"
    if "== GPIO_PIN_RESET" in text:
        return text, False

    # CubeMX generates: if(HAL_GPIO_ReadPin(...) != GPIO_PIN_RESET) { status = SD_NOT_PRESENT; }
    # We need:          if(HAL_GPIO_ReadPin(...) == GPIO_PIN_RESET) { status = SD_NOT_PRESENT; }
    pattern = re.compile(
        r'(if\s*\(\s*HAL_GPIO_ReadPin\s*\(\s*SD_DETECT_GPIO_PORT\s*,\s*SD_DETECT_PIN\s*\)\s*)!=(.*?GPIO_PIN_RESET)',
        re.DOTALL
    )
    if not pattern.search(text):
        return text, False

    text = pattern.sub(r'\1==\2', text)
    return text, True


def main() -> int:
    retval = 0

    # Patch linker script
    if not LD_PATH.exists():
        print(f"[patch-linker] missing linker script: {LD_PATH}")
        retval = 1
    else:
        original = LD_PATH.read_text()

        # Apply all patches in order
        updated, changed_flash = patch_flash_region(original)
        updated, added_param_region = ensure_param_flash_region(updated)
        updated, added_param_symbols = ensure_param_flash_symbols(updated)
        updated, added_layout_comment = ensure_flash_layout_comment(updated)
        updated, added_dma = ensure_dma_section(updated)
        updated, added_bdma = ensure_bdma_section(updated)

        if updated != original:
            LD_PATH.write_text(updated)
            changes = []
            if changed_flash:
                changes.append("flash")
            if added_param_region:
                changes.append("param_region")
            if added_param_symbols:
                changes.append("param_symbols")
            if added_layout_comment:
                changes.append("layout_comment")
            if added_dma:
                changes.append("dma")
            if added_bdma:
                changes.append("bdma")
            print(f"[patch-linker] patched STM32H723XG_FLASH.ld ({', '.join(changes)})")
        else:
            print("[patch-linker] STM32H723XG_FLASH.ld: no changes needed")

    # Patch FATFS for DMA buffer placement
    if not FATFS_PATH.exists():
        print(f"[patch-linker] missing fatfs.c: {FATFS_PATH}")
    else:
        original = FATFS_PATH.read_text()
        updated, patched_fatfs = patch_fatfs_dma_buffers(original)
        if patched_fatfs:
            FATFS_PATH.write_text(updated)
            print(f"[patch-linker] patched fatfs.c (SDFatFS/SDFile -> .dma_buffer)")
        else:
            print("[patch-linker] fatfs.c: no changes needed")

    # Patch SD detect logic (hardware has inverted switch)
    if not FATFS_PLATFORM_PATH.exists():
        print(f"[patch-linker] missing fatfs_platform.c: {FATFS_PLATFORM_PATH}")
    else:
        original = FATFS_PLATFORM_PATH.read_text()
        updated, patched_detect = patch_sd_detect_logic(original)
        if patched_detect:
            FATFS_PLATFORM_PATH.write_text(updated)
            print("[patch-linker] patched fatfs_platform.c (SD detect != -> ==)")
        else:
            print("[patch-linker] fatfs_platform.c: no changes needed")

    return retval


if __name__ == "__main__":
    raise SystemExit(main())
