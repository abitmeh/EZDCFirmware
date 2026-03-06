"""
Log.py — Coloured terminal logging for MotorTrace.

Table layout (all indented 6 spaces to align under the [x] prefix):

      ======= TITLE HERE =========
      Key                   Value
      ============================

TABLE_WIDTH is the total content width after the 6-space indent.
It is set to comfortably fit the widest stat row: 26 (key) + ~14 (value).
"""

import re
from enum import Enum, auto


class Level(Enum):
    DEBUG = auto()
    INFO  = auto()
    WARN  = auto()
    ERROR = auto()


class _ANSI:
    RESET  = "\033[0m"
    WHITE  = "\033[97m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"


_PREFIX = {
    Level.DEBUG: "[?]",
    Level.INFO:  "[+]",
    Level.WARN:  "[*]",
    Level.ERROR: "[#]",
}

_COLOUR = {
    Level.DEBUG: _ANSI.WHITE,
    Level.INFO:  _ANSI.GREEN,
    Level.WARN:  _ANSI.YELLOW,
    Level.ERROR: _ANSI.RED,
}

_INDENT     = "      "   # 6 spaces — aligns under [x] prefix
TABLE_WIDTH = 40         # content width of header/footer lines


def log(level: Level, msg: str):
    col = _COLOUR[level]
    pfx = _PREFIX[level]
    print(f"{col}{pfx} {msg}{_ANSI.RESET}")


def log_sub(level: Level, msg: str):
    col = _COLOUR[level]
    print(f"{col}{_INDENT}{msg}{_ANSI.RESET}")


def log_header(level: Level, title: str):
    col   = _COLOUR[level]
    inner = f" {title} "
    pad   = max(0, TABLE_WIDTH - len(inner))
    left  = pad // 2
    right = pad - left
    line  = "=" * left + inner + "=" * right
    print(f"{col}\n{_INDENT}{line}{_ANSI.RESET}")


def log_footer(level: Level):
    col = _COLOUR[level]
    print(f"{col}{_INDENT}{'=' * TABLE_WIDTH}{_ANSI.RESET}\n")


def log_stat(key: str, value, level: Level = Level.INFO):
    """
    Print a key/value stat row inside a table.

    level controls the colour of both key and value:
      Level.DEBUG → white
      Level.INFO  → white key, green value  (default)
      Level.WARN  → yellow key and value
      Level.ERROR → red key and value
    """
    col     = _COLOUR[level]
    key_col = _ANSI.WHITE if level == Level.INFO else col
    print(f"{key_col}{_INDENT}{key:<26}{col}{value}{_ANSI.RESET}")


# ── Firmware log parsing ──────────────────────────────────────────────────────

# Matches ESP-IDF log prefix: D(123), I(42), W(7), E(999)
_FW_LOG_RE = re.compile(r'^([DIWE])\s*\(\s*\d+\s*\)\s*(.*)', re.DOTALL)

_FW_LEVEL = {
    'D': Level.DEBUG,
    'I': Level.INFO,
    'W': Level.WARN,
    'E': Level.ERROR,
}

def log_firmware(raw: str):
    """
    Emit a firmware log line, parsing ESP-IDF log level prefixes.
    Lines starting with D(n), I(n), W(n) or E(n) are routed to the
    matching log level; everything else is treated as debug.
    """
    m = _FW_LOG_RE.match(raw.strip())
    if m:
        level, msg = _FW_LEVEL[m.group(1)], m.group(2).strip()
        log(level, f"[FW] {msg}")
    else:
        log(Level.DEBUG, f"[FW] {raw.rstrip()}")
