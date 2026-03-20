# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
"""Shared cocotb test logging helpers for real-time progress visibility."""

from functools import wraps
import sys

import cocotb
from cocotb.utils import get_sim_time

_ANSI_RESET = "\033[0m"
_ANSI_YELLOW = "\033[33m"
_ANSI_GREEN = "\033[32m"
_ANSI_RED = "\033[31m"


def _color(text, ansi):
    if sys.stdout.isatty():
        return f"{ansi}{text}{_ANSI_RESET}"
    return text


def logged_test(*test_args, **test_kwargs):
    """Wrap `cocotb.test` with clear START/PASS/FAIL log lines."""

    def decorator(func):
        test_name = func.__name__

        @cocotb.test(*test_args, **test_kwargs)
        @wraps(func)
        async def wrapped(dut, *args, **kwargs):
            print(_color(f"[TEST START] {test_name}", _ANSI_YELLOW), flush=True)
            try:
                await func(dut, *args, **kwargs)
            except Exception as exc:
                sim_time_ns = float(get_sim_time(units="ns"))
                print(
                    _color(f"[TEST FAIL] {test_name} @ {sim_time_ns:.3f} ns ({exc})", _ANSI_RED),
                    flush=True,
                )
                raise
            sim_time_ns = float(get_sim_time(units="ns"))
            print(_color(f"[TEST PASS] {test_name} @ {sim_time_ns:.3f} ns", _ANSI_GREEN), flush=True)

        return wrapped

    return decorator
