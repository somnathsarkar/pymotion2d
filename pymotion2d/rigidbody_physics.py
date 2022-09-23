"""Physics engine implementation"""
from typing import List, Optional
from dataclasses import dataclass

import numpy as np

from pymotion2d.typing import vec3
from pymotion2d import scene

_GRAVITATION = 500.0

@dataclass
class RigidbodyContact(object):
  a: scene.Rigidbody
  b: scene.Rigidbody
  p: vec3
  n: vec3
  pen: float

def _generate_rigidbody_contact(a: scene.Rigidbody,
    b: scene.Rigidbody) -> Optional[RigidbodyContact]:
  b_x = np.array([np.cos(b.ang), np.sin(b.ang)])
  b_y = np.array([-np.sin(b.ang), np.cos(b.ang)])
  b_n = np.array([-b_x,b_x,-b_y,b_y])
  a_verts = scene.rigidbody_vertices(a)
  bx_comp = np.dot(a_verts, b_x)-np.dot(b_x, b.x)
  by_comp = np.dot(a_verts, b_y)-np.dot(b_y, b.x)
  bx0_pen = bx_comp+b.r[0]/2.0
  bx1_pen = b.r[0]/2.0-bx_comp
  by0_pen = by_comp+b.r[1]/2.0
  by1_pen = b.r[1]/2.0-by_comp
  vert_pen = np.vstack((bx0_pen,bx1_pen,by0_pen,by1_pen))
  vert_inside = np.all(vert_pen>=0,axis=0)
  if np.any(vert_inside):
    vert_idx = np.argmax(vert_inside)
    edge_idx = np.argmin(vert_pen[vert_idx])
    contact = RigidbodyContact(a, b, a_verts[vert_idx],
        b_n[edge_idx], vert_pen[edge_idx])
    return contact
  return None

def _find_rigidbody_contacts(sc: scene.Scene) -> List[RigidbodyContact]:
  contacts: List[RigidbodyContact] = []
  for a_i, rb_a in enumerate(sc.rigidbodies):
    if rb_a.collision_layer == 0:
      continue
    for rb_b in sc.rigidbodies[a_i+1:]:
      if rb_b.collision_layer == 0:
        continue
      if (rb_a.collision_mask>>rb_b.collision_layer)&1 == 0:
        continue
      contact = _generate_rigidbody_contact(rb_a,rb_b)
      if contact is not None:
        contacts.append(contact)
      else:
        contact = _generate_rigidbody_contact(rb_b,rb_a)
        if contact is not None:
          contacts.append(contact)
  return contacts

def update_rigidbody_physics(sc: scene.Scene, dt: float):
  for rigidbody in sc.rigidbodies:
    if rigidbody.static:
      continue
    rigidbody.x += rigidbody.v*dt
    rigidbody.v[1] -= _GRAVITATION*dt
    rigidbody.ang += rigidbody.w*dt

  # Rigidbody-Rigidbody contacts
  contacts = _find_rigidbody_contacts(sc)

  # Static rigidbodies - mark colliding
  for rigidbody in sc.rigidbodies:
    if rigidbody.static:
      rigidbody.colliding = False
  for contact in contacts:
    if contact.a.static:
      contact.a.colliding = True
    if contact.b.static:
      contact.b.colliding = True
