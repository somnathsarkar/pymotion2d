"""Fireworks example for unconstrained particle physics"""
import time

import numpy as np

from pymotion2d.typing import ivec2, vec2
import pymotion2d as pm

_FIREWORK_PARTICLES = 10
_FIREWORK_ANGLE = 180
_FIREWORK_SPEED = 300
_FIREWORK_INTERVAL = 2

def spawn_firework(scene: pm.Scene, pos: vec2):
  """Spawn a collection of particles.

  Args:
    scene: Scene to add particles to.
    pos: 2d world coordinates of center of firework.
  """
  radlim = np.radians(_FIREWORK_ANGLE)
  particle_radians = np.random.rand(_FIREWORK_PARTICLES)
  particle_radians = particle_radians*(2*radlim)-radlim+np.pi/2.0
  particle_x = np.cos(particle_radians)
  particle_y = np.sin(particle_radians)
  particle_v = _FIREWORK_SPEED*np.hstack(
    (particle_x.reshape(-1,1),
    particle_y.reshape(-1,1)))
  for particle_i in range(_FIREWORK_PARTICLES):
    particle = pm.Particle(False, False, 1, 1, 1.0, 0.6,
        pos, particle_v[particle_i], 10)
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
  firework_accumulator = 0
  dim = np.array([640, 480])
  lower_lim = np.array([200, 100])
  upper_lim = np.array([440, 380])
  window = pm.Window(dim)
  scene = pm.Scene([],[],pm.Settings(floor=True,floor_elasticity=1.0))
  while window.running:
    new_time = time.perf_counter()
    dt = new_time-prev_time
    firework_accumulator-=dt
    if firework_accumulator<=0.0:
      spawn_pos = np.random.rand(2)*(upper_lim-lower_lim)+lower_lim
      spawn_firework(scene, spawn_pos)
      firework_accumulator += _FIREWORK_INTERVAL
    scene.particles = list(filter(
      lambda particle: particle_in_frame(dim, particle),
      scene.particles))
    window.process_events()
    if not window.running:
      break
    pm.update_particle_physics(scene, dt)
    window.render_scene(scene)
    prev_time = new_time

if __name__ == '__main__':
  main()
