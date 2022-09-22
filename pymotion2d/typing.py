"""GLSL-style datatypes

Vector lengths are not enforced, merely used to provide clarity"""
import numpy as np
import numpy.typing as npt

vec2 = npt.NDArray[np.float32]
vec3 = npt.NDArray[np.float32]
ivec2 = npt.NDArray[np.int32]
