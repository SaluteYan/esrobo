from collections import deque
import pytest
from esrobo_laptop.app import recent_rate


def test_entry_gap_does_not_pollute_active_rate():
    stamps = deque([1., 1.02, 1.04]+[20.+i*.02 for i in range(101)])
    assert recent_rate(stamps, 22.) == pytest.approx(50.)
    assert stamps[0] == 20.
    assert recent_rate(stamps, 25.) == 0.


def test_display_uses_same_recent_window_as_frequency_gate():
    stamps = deque([i*.02 for i in range(101)]+[2.05+i/30. for i in range(61)])
    assert recent_rate(stamps, 4.05) == pytest.approx(30.)
