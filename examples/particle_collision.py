"""Example demonstrating particle collision detection"""
import time

import numpy as np

import pymotion2d as pm

_GRID_COLS = 20
_GRID_ROWS = 10

def main():
  current_time = time.perf_counter()
  prev_time = current_time
  dim = np.array([640, 480])
  window = pm.Window(dim)

  # Setup particle grid
  grid_x = np.linspace(10,640-10,_GRID_COLS).reshape(-1,1)
  grid_y = np.linspace(10,480-10,_GRID_ROWS).reshape(-1,1)
  grid_col_offsets = np.random.rand(_GRID_COLS,1)*10+10
  grid_row_offsets = np.random.rand(_GRID_ROWS,1)*10+10
  grid_col_starts = np.hstack((grid_x,grid_col_offsets))
  grid_row_starts = np.hstack((grid_row_offsets,grid_y))
  grid_starts = np.vstack((grid_col_starts,grid_row_starts))
  grid_col_ends = np.hstack((grid_x,480-grid_col_offsets))
  grid_row_ends = np.hstack((640-grid_row_offsets,grid_y))
  grid_ends = np.vstack((grid_col_ends,grid_row_ends))
  grid_distances = grid_ends-grid_starts
  grid_periods = np.random.rand(_GRID_COLS+_GRID_ROWS,1)*4.5+0.5

  # Spawn particles
  scene = pm.Scene([],[],pm.Settings())
  for particle_i in range(_GRID_COLS+_GRID_ROWS):
    particle = pm.Particle(True, False, 1, 2, 1.0, 1.0,
        grid_starts[particle_i], np.zeros(2), 10)
    scene.particles.append(particle)

  while window.running:
    new_time = time.perf_counter()
    dt = new_time-prev_time
    window.process_events()
    if not window.running:
      break
    pm.update_particle_physics(scene, dt)

    # Update particle grid
    particle_phase = (2*np.pi*new_time/grid_periods)
    particle_pos = (grid_distances/2.0)*(np.sin(particle_phase)+1)+grid_starts
    for particle_i, particle in enumerate(scene.particles):
      particle.x = particle_pos[particle_i]

    window.render_scene(scene)
    prev_time = new_time

if __name__ == '__main__':
  main()
