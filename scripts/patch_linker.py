#!/usr/bin/env python3
"""
Idempotently patch the CubeMX-generated linker script so our app stays
bootloader-friendly and keeps a DMA-safe buffer section.

Changes applied:
1) FLASH origin -> 0x08020000 (bootloader offset), length -> 0x000C0000.
2) Ensure a .dma_buffer (NOLOAD, 32-byte aligned) section mapped to RAM_D1.
3) Ensure a .bdma_buffer (NOLOAD, 32-byte aligned) section mapped to RAM_D3.

Run this after every CubeMX regeneration of STM32H723XG_FLASH.ld.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LD_PATH = ROOT / "STM32H723XG_FLASH.ld"

FLASH_ORIGIN = "0x8020000"
FLASH_LENGTH = "0xC0000"

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


def patch_flash_region(text: str) -> tuple[str, bool]:
    pattern = re.compile(r"FLASH\s*\(rx\)\s*:\s*ORIGIN\s*=\s*0x[0-9A-Fa-f]+,\s*LENGTH\s*=\s*0x[0-9A-Fa-f]+")
    replacement = f"FLASH (rx)      : ORIGIN = {FLASH_ORIGIN}, LENGTH = {FLASH_LENGTH}"
    new_text, count = pattern.subn(replacement, text, count=1)
    return new_text, count > 0


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


def main() -> int:
    if not LD_PATH.exists():
        print(f"[patch-linker] missing linker script: {LD_PATH}")
        return 1

    original = LD_PATH.read_text()
    updated, changed_flash = patch_flash_region(original)
    updated, added_dma = ensure_dma_section(updated)
    updated, added_bdma = ensure_bdma_section(updated)

    if updated != original:
        LD_PATH.write_text(updated)
        print(
            "[patch-linker] patched STM32H723XG_FLASH.ld "
            f"(flash={'yes' if changed_flash else 'no'}, dma={'yes' if added_dma else 'no'}, "
            f"bdma={'yes' if added_bdma else 'no'})"
        )
    else:
        print("[patch-linker] no changes needed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
