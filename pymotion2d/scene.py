"""Scene information definitions"""
from dataclasses import dataclass
from typing import List

import numpy.typing as npt
import numpy as np

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
  w: float

@dataclass
class Settings(object):
  floor: bool = False
  floor_elasticity: float = 0.0

@dataclass
class Scene(object):
  particles: List[Particle]
  rigidbodies: List[Rigidbody]
  settings: Settings

def rigidbody_vertices(rigidbody: Rigidbody) -> npt.NDArray[np.float32]:
  """Return a 4x2 matrix containing rectangle rigidbody vertices
  in CCW order"""
  vert_ang = np.arctan2(rigidbody.r[1],rigidbody.r[0])
  vert_angs = rigidbody.ang + np.array([vert_ang, np.pi-vert_ang,
      -np.pi+vert_ang, -vert_ang]).reshape(-1,1)
  vert_rad = np.linalg.norm(rigidbody.r/2)
  vert_x = np.cos(vert_angs)
  vert_y = np.sin(vert_angs)
  vert_pos = rigidbody.x+vert_rad*np.hstack((vert_x,vert_y))
  return vert_pos
