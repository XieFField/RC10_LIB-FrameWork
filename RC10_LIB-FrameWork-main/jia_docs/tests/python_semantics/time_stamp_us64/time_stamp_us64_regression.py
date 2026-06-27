from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from jia_docs.tests.shared.python_test_support import repo_root


REPO_ROOT = repo_root()
HEADER_PATH = REPO_ROOT / "RC10_LIB" / "BSP_Driver" / "Inc" / "BSP_RtosTimeStampUs64.h"
SOURCE_PATH = REPO_ROOT / "RC10_LIB" / "BSP_Driver" / "Src" / "BSP_RtosTimeStampUs64.cpp"


def ticks_to_us64(ticks: int, tick_rate_hz: int) -> int:
    if tick_rate_hz == 0:
        return 0
    return (ticks * 1_000_000) // tick_rate_hz


def compose_time_us64(ticks: int, tick_rate_hz: int, sub_tick_us: int) -> int:
    tick_period_us = 0 if tick_rate_hz == 0 else (1_000_000 // tick_rate_hz)
    if tick_period_us != 0 and sub_tick_us > tick_period_us:
        sub_tick_us = tick_period_us
    return ticks_to_us64(ticks, tick_rate_hz) + sub_tick_us


def test_ticks_to_us64_zero_rate() -> None:
    assert ticks_to_us64(123, 0) == 0


def test_ticks_to_us64_regular_case() -> None:
    assert ticks_to_us64(500, 1000) == 500_000
    assert ticks_to_us64(1, 1000) == 1000


def test_compose_time_us64_accumulate() -> None:
    assert compose_time_us64(10, 1000, 250) == 10_250


def test_compose_time_us64_clamp_sub_tick() -> None:
    # tick_period_us for 1000 Hz is 1000 us, sub_tick_us should clamp to 1000.
    assert compose_time_us64(2, 1000, 1500) == 3000


def test_source_contract() -> None:
    header = HEADER_PATH.read_text(encoding="utf-8")
    source = SOURCE_PATH.read_text(encoding="utf-8")

    assert "namespace jia {" in header
    assert "namespace jia::time" not in header
    assert "class RtosTimeStampUs64" in header
    assert "getTimeUs" in header
    assert "TicksToUs64" not in header
    assert "ComposeTimeUs64" not in header

    assert '#include "BSP_RtosTimeStampUs64.h"' in source
    assert "namespace jia {" in source
    assert "namespace jia::time" not in source


def run_all() -> None:
    test_ticks_to_us64_zero_rate()
    test_ticks_to_us64_regular_case()
    test_compose_time_us64_accumulate()
    test_compose_time_us64_clamp_sub_tick()
    test_source_contract()


if __name__ == "__main__":
    run_all()
    print("time_stamp_us64 regression passed")
