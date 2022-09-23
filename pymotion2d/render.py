"""Utilities to render 2D objects in scene to pygame window"""
import pygame
import numpy as np

from pymotion2d.typing import ivec2
from pymotion2d import scene
from pymotion2d import colors

class Window(object):
  """Pygame window management class.

  Usage:
    window = Window((640, 480))
    while window.running:
      window.process_events()
      if not window.running:
        break
      window.render_scene()
  """
  def __init__(self, dim: ivec2):
    pygame.init()
    self.surface = pygame.display.set_mode(dim)
    self.dim = dim
    self.running = True

  def process_events(self):
    """Process all window events and update running flag"""
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        self.running = False
        pygame.quit()
      if event.type == pygame.KEYUP:
        if event.key == pygame.K_ESCAPE:
          self.running = False
          pygame.quit()

  def render_scene(self, sc: scene.Scene):
    """Render all scene objects"""
    self.surface.fill(colors.LIGHT_GRAY)
    for particle in sc.particles:
      self._render_particle(particle)
    for rigidbody in sc.rigidbodies:
      self._render_rigidbody(rigidbody)
    pygame.display.update()

  def _render_particle(self, particle: scene.Particle):
    """Draw a single particle on screen"""
    particle_coord = np.copy(particle.x)
    particle_coord[1] = self.dim[1]-particle_coord[1]
    p_color = colors.DARK_BLUE
    if particle.colliding:
      p_color = colors.DARK_ORANGE
    pygame.draw.circle(self.surface, p_color,
      particle_coord, particle.r, 0)
    pygame.draw.circle(self.surface, colors.DARK_GRAY,
      particle_coord, particle.r, 2)

  def _render_rigidbody(self, rigidbody: scene.Rigidbody):
    """Draw a single rigidbody rectangle collider on screen"""
    p_color = colors.DARK_BLUE
    if rigidbody.colliding:
      p_color = colors.DARK_ORANGE
    vert_pos = scene.rigidbody_vertices(rigidbody)
    vert_pos[:,1] = self.dim[1]-vert_pos[:,1]
    pygame.draw.polygon(self.surface, p_color, vert_pos, 0)
    pygame.draw.polygon(self.surface, colors.DARK_GRAY, vert_pos, 2)
