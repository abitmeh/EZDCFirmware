"""
Utilities.py — Shared utility functions for the EZDC toolchain.
"""

# ======================== CRC8 Dallas ========================

def _make_crc8_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

_CRC8_TABLE = _make_crc8_table()

def crc8_dallas(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc
