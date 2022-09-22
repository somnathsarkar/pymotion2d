"""Scene information definitions"""
from dataclasses import dataclass
from typing import List

from pymotion2d.typing import vec2

@dataclass
class PhysicsObject:
  static: bool
  colliding: bool
  collision_layer: int
  collision_mask: int
  m: float
  elasticity: float
  x: vec2
  v: vec2

@dataclass
class Particle(PhysicsObject):
  r: float

@dataclass
class Rigidbody(PhysicsObject):
  r: vec2
  ang: float

@dataclass
class Settings(object):
  floor: bool = False
  floor_elasticity: float = 0.0

@dataclass
class Scene(object):
  particles: List[Particle]
  rigidbodies: List[Rigidbody]
  settings: Settings
