"""Particle collision resolution"""
import time

import numpy as np

from pymotion2d.typing import ivec2
import pymotion2d as pm

_SPAWN_INTERVAL = 1
_SPAWN_RANGE = 100
_BALL_RADIUS = 10
_PEG_RADIUS = 20
_PEG_COUNT = 10

def spawn_ball(scene: pm.Scene, dim: ivec2):
  """Spawn a new ball at the top of the screen

  Args:
    scene: Scene to modify
    dim: Dimensions of frame
  """
  spawn_x = np.random.rand()*_SPAWN_RANGE
  spawn_x = spawn_x + ((dim[0]-_SPAWN_RANGE)/2.0)
  pos = np.array((spawn_x,dim[1]))
  particle = pm.Particle(False, False, 1, 3, 1.0, 0.6,
      pos, np.zeros(2), _BALL_RADIUS)
  scene.particles.append(particle)

def particle_in_frame(dim: ivec2, particle: pm.Scene) -> bool:
  """Filter function that returns true if particle is in frame

  Args:
    dim: Dimensions of frame
    particle: Particle to read
  Returns:
    result: True if particle in frame, false otherwise
  """
  if np.any(particle.x<0) or np.any(particle.x>dim):
    return False
  return True

def main():
  current_time = time.perf_counter()
  prev_time = current_time
  ball_accumulator = 0
  dim = np.array([640, 480])
  lower_lim = np.array([200, 100])
  upper_lim = np.array([440, 380])
  window = pm.Window(dim)
  scene = pm.Scene([],[],pm.Settings(floor=True,floor_elasticity=1.0))

  # Setup pegs
  peg_x = np.random.rand(_PEG_COUNT,2)*(upper_lim-lower_lim)+lower_lim
  for peg_i in range(_PEG_COUNT):
    particle = pm.Particle(True, False, 1, 2, 1.0, 0.6,
        peg_x[peg_i], np.zeros(2), _PEG_RADIUS)
    scene.particles.append(particle)

  while window.running:
    new_time = time.perf_counter()
    dt = new_time-prev_time
    ball_accumulator += dt
    if ball_accumulator>_SPAWN_INTERVAL:
      spawn_ball(scene, dim)
      ball_accumulator -= _SPAWN_INTERVAL
    window.process_events()
    if not window.running:
      break
    pm.update_particle_physics(scene, dt)
    window.render_scene(scene)
    prev_time = new_time

if __name__ == '__main__':
  main()
