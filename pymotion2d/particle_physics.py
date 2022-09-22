"""Particle (Point mass) Physics engine implementation"""
from typing import List, Optional
from dataclasses import dataclass

import numpy as np

from pymotion2d.typing import vec3
from pymotion2d import scene

_GRAVITATION = 500.0

@dataclass
class ParticleContact(object):
  a: scene.Particle
  b: Optional[scene.Particle]
  n: vec3
  penetration: float

def _unsafe_normalize(x: vec3) -> vec3:
  x_norm = np.linalg.norm(x)
  return x/x_norm

def _find_particle_contacts(sc: scene.Scene) -> List[ParticleContact]:
  # Generate contacts
  # Particle-Particle contacts
  contacts: List[ParticleContact] = []
  for particle_a_i, particle_a in enumerate(sc.particles):
    if particle_a.collision_layer == 0:
      continue
    for particle_b in sc.particles[particle_a_i+1:]:
      if particle_b.collision_layer == 0:
        continue
      if (particle_a.collision_mask>>particle_b.collision_layer) & 1 == 0:
        continue
      disp = particle_b.x - particle_a.x
      dist = np.linalg.norm(disp)
      pen = particle_a.r+particle_b.r-dist
      if pen>=0:
        contact = ParticleContact(particle_a, particle_b, disp/dist, pen)
        contacts.append(contact)

  # Particle-Floor contacts
  if sc.settings.floor:
    for particle_a_i, particle_a in enumerate(sc.particles):
      if particle_a.collision_layer == 0:
        continue
      if particle_a.collision_mask & 1 == 0:
        continue
      pen = particle_a.r-particle_a.x[1]
      if pen>=0:
        contact = ParticleContact(particle_a, None, np.array([0.0, -1.0]), pen)
        contacts.append(contact)
  return contacts

def update_particle_physics(sc: scene.Scene, dt: float):
  """Move forward by one physics step for each particle"""
  # Iterate position and velocity
  for particle in sc.particles:
    if particle.static:
      continue
    particle.x = particle.x+particle.v*dt
    particle.v[1] = particle.v[1]-_GRAVITATION*dt

  contacts = _find_particle_contacts(sc)

  # Particle Collision Resolution
  # For non-static particles, apply a restitution-adjusted impulse
  for contact in contacts:
    p_a = contact.a
    p_b = contact.b
    if p_a.static and p_b.static:
      continue
    a_v = p_a.v
    b_v = np.zeros(2) if p_b is None else p_b.v
    restitution = p_a.elasticity*sc.settings.floor_elasticity
    if p_b is not None:
      restitution = p_a.elasticity*p_b.elasticity
    sep_v = np.dot(a_v-b_v,contact.n)
    new_sep_v = -sep_v*restitution
    acc_caused_v = np.zeros(2)
    if p_b is None:
      acc_caused_v = _GRAVITATION*np.array([0.0, -1.0])
    acc_caused_sep_v = np.dot(acc_caused_v,contact.n)*dt
    if acc_caused_sep_v < 0:
      new_sep_v += restitution*acc_caused_sep_v
      new_sep_v = max(new_sep_v, 0.0)
    delta_v = new_sep_v-sep_v
    a_im = 1/p_a.m if not p_a.static else 0.0
    b_im = 1/p_b.m if p_b is not None and not p_b.static else 0.0
    ab_im = a_im+b_im
    ab_j = delta_v/ab_im
    if not p_a.static:
      p_a.v = a_v+contact.n*ab_j*a_im
    if p_b is not None and not p_b.static:
      p_b.v = b_v-contact.n*ab_j*b_im

  # For static particles, set colliding flag
  for particle in sc.particles:
    if particle.static:
      particle.colliding = False
  for contact in contacts:
    for particle in (contact.a, contact.b):
      if particle is not None and particle.static:
        particle.colliding = True

  # Interpenetration Resolution
  # Non-static particles only
  pen_eps = 1e-3
  pen_iters = 1
  for _ in range(pen_iters):
    for contact in contacts:
      p_a = contact.a
      p_b = contact.b
      a_m = p_a.m if not p_a.static else 0.0
      b_m = p_b.m if p_b is not None and not p_b.static else 0.0
      ab_m = a_m + b_m
      delta_dist = (contact.penetration+pen_eps)*ab_m
      if not p_a.static:
        p_a.x = p_a.x-contact.n*delta_dist/a_m
      if p_b is not None and not p_b.static:
        p_b.x = p_b.x+contact.n*delta_dist/b_m
    contacts = _find_particle_contacts(sc)
