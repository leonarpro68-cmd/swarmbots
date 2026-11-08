#!/usr/bin/env python3
"""Configurable, experimental wait budgets. Safety deadlines are unchanged."""
from dataclasses import dataclass

@dataclass(frozen=True)
class TimingProfile:
    observe_seconds: float
    start_delay: float
    preflight_observe_seconds: float
    recorder_start_seconds: float

PROFILES = {
    'normal': TimingProfile(8.0, 15.0, 8.0, 3.0),
    'fast': TimingProfile(8.0, 5.0, 4.0, 1.0),
}

def get_profile(name: str) -> TimingProfile:
    try:
        return PROFILES[name]
    except (KeyError, TypeError):
        raise ValueError(f'Perfil de tiempos desconocido: {name!r}; usa normal o fast') from None
