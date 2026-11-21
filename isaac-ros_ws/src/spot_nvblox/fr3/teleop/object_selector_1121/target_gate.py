"""Fail-closed state contract for an object-selector integration.

This module has no ROS or robot side effects. The production adapter must supply
fresh observations and synchronize them with tracker, map, and grasp broker.
"""
from dataclasses import dataclass
from enum import Enum

class Target(str, Enum):
    DRILL = 'drill'
    TENNIS_BALL = 'tennis_ball'
    SPRITE = 'sprite'

@dataclass(frozen=True)
class Evidence:
    generation: int
    target: str
    mask_target: str
    mask_age: float
    mask_fraction: float
    tracker_ok: bool
    map_target: str
    map_current: bool
    target_absent: bool
    other_obstacles_present: bool
    binding_target: str
    binding_generation: int
    broker_pass: bool
    held_model_ready: bool

class TargetGate:
    def __init__(self):
        self.generation = 0
        self.selected = None
        self.reason = 'No target selected'

    def select(self, target, *, engaged, busy, held):
        target = Target(target)
        if engaged or busy or held:
            raise RuntimeError('Disengage and release any held object before changing targets')
        self.generation += 1
        self.selected = target.value
        self.reason = 'Awaiting mask, map and grasp validation'
        return self.generation

    def evaluate(self, e):
        checks = [
            (self.selected is not None, 'No target selected'),
            (e.generation == self.generation and e.target == self.selected, 'Selection changed'),
            (e.tracker_ok and e.mask_target == self.selected and 0 <= e.mask_age <= 1.5 and .0001 < e.mask_fraction < .20, 'Target mask missing, stale or invalid'),
            (e.map_target == self.selected and e.map_current and e.target_absent and e.other_obstacles_present, 'Collision map not validated'),
            (e.binding_target == self.selected and e.binding_generation == self.generation and e.broker_pass, 'Grasp not validated for selected target'),
            (e.held_model_ready, 'Held-object collision model unavailable'),
        ]
        for ok, reason in checks:
            if not ok:
                self.reason = reason
                return False
        self.reason = 'Ready'
        return True
