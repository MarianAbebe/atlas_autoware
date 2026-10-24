"""Shared deterministic geometry for the headless simulation scenarios."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Rect:
    xmin: float
    xmax: float
    ymin: float
    ymax: float


SCENARIO_RECTS = {
    'open': (),
    # Narrow enough for a 0.32 m radius robot to pass, but blocks the straight
    # line to x=4.5 and requires a visible lateral detour.
    'avoidable': (Rect(1.75, 2.25, -0.25, 0.25),),
    # Connects both map boundaries, so there is no legal global route.
    'blocked': (Rect(1.75, 2.25, -2.85, 2.85),),
}


def rectangles(scenario):
    try:
        return SCENARIO_RECTS[scenario]
    except KeyError as exc:
        raise ValueError(f'unknown simulation scenario: {scenario}') from exc


def occupied(x, y, scenario):
    if x <= -1.85 or x >= 5.85 or y <= -2.85 or y >= 2.85:
        return True
    return any(rect.xmin <= x <= rect.xmax and rect.ymin <= y <= rect.ymax
               for rect in rectangles(scenario))


def clearance(x, y, radius, scenario):
    """Signed free-space clearance from robot centre to geometry, in metres."""
    if radius <= 0.0:
        raise ValueError('radius must be positive')
    boundary = min(x + 1.85, 5.85 - x, y + 2.85, 2.85 - y) - radius
    values = [boundary]
    for rect in rectangles(scenario):
        dx = max(rect.xmin - x, 0.0, x - rect.xmax)
        dy = max(rect.ymin - y, 0.0, y - rect.ymax)
        values.append(math.hypot(dx, dy) - radius)
    return min(values)
