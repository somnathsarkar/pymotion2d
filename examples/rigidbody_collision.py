"""Example demonstrating rigidbody collision detection"""
import time

import numpy as np

import pymotion2d as pm

_RIGID_A_DIM = (100, 200)
_RIGID_B_DIM = (100, 80)
_RIGID_A_ANG_VEL = 1.25
_RIGID_B_ANG_VEL = -0.75
_RIGID_B_PERIOD = 5.0

def _positive_triangle_wave(t: float, period: float, amplitude: float) -> float:
  """
  Return the position of a positive triangle wave

  Args:
    t: Time to evaluate wave
    period: Time period of wave
    amplitude: Amplitude of wave
  """
  p = 2*t/period
  w = np.floor(p)
  phi = p-w
  z_1 = (-1)**w
  z_0 = (1-z_1)/2.0
  x = amplitude*(z_0+z_1*phi)
  return x

def main():
  start_time = time.perf_counter()
  prev_time = start_time
  elapsed_time = 0.0
  dim = np.array([640, 480])
  window = pm.Window(dim)

  # Setup static rigidbodies
  scene = pm.Scene([],[],pm.Settings(False, 0.0))
  rigid_a = pm.Rigidbody(True, False, 1, 2, 1.0, 0.0, dim/2,
      np.zeros(2), np.array(_RIGID_A_DIM), 0.0)
  rigid_b = pm.Rigidbody(True, False, 1, 2, 1.0, 1.0,
      np.array([0, dim[1]/2]), np.zeros(2),
      np.array(_RIGID_B_DIM), 0.0)
  scene.rigidbodies.extend([rigid_a,rigid_b])
  while window.running:
    new_time = time.perf_counter()
    elapsed_time = new_time-start_time
    dt = new_time-prev_time

    # Update static rigidbodies
    scene.rigidbodies[0].ang += _RIGID_A_ANG_VEL*dt
    scene.rigidbodies[1].x[0] = _positive_triangle_wave(elapsed_time,
        _RIGID_B_PERIOD, dim[0])
    scene.rigidbodies[1].ang += _RIGID_B_ANG_VEL*dt

    window.process_events()
    if not window.running:
      break
    pm.update_particle_physics(scene, dt)
    window.render_scene(scene)
    prev_time = new_time

if __name__ == '__main__':
  main()
