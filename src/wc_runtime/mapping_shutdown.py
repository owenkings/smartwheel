"""Bounded persistence waits for the one-command mapping session.

Observed eMMC checkpoints reached 43--59 seconds; 10--25 second shutdown
waits interrupted otherwise healthy static maps. These are stop-time storage
budgets, not sensor age, tracking quality, or motor-stop deadlines.
"""
import math

PERSISTENCE_CLOSE_S = 90.0
NATIVE_SIGTERM_S = 105.0
MAPPING_COMPONENT_GRACE_S = 120.0


def persistence_wait(value):
    if isinstance(value, bool):
        raise ValueError('Persistence close wait must be finite and in (0, 90] seconds')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError('Persistence close wait must be numeric') from error
    if not math.isfinite(result) or not 0 < result <= PERSISTENCE_CLOSE_S:
        raise ValueError('Persistence close wait must be finite and in (0, 90] seconds')
    return result


def policy():
    return {'persistence_close_s': PERSISTENCE_CLOSE_S, 'native_sigterm_s': NATIVE_SIGTERM_S,
            'component_sigint_grace_s': MAPPING_COMPONENT_GRACE_S,
            'scope': 'Stop-time persistence only; acquisition freshness and tracking checks unchanged',
            'limitation': 'Observed-latency allowance, not a worst-case kernel I/O or saving guarantee'}
