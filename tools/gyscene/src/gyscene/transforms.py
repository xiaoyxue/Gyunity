"""Row-major affine math used by external scene adapters."""

import math

from .model import validate_matrix


def multiply(a, b):
    return tuple(sum(a[4*r+k] * b[4*k+c] for k in range(4)) for r in range(4) for c in range(4))


def inverse(m):
    validate_matrix(m)
    a, b, c, _, d, e, f, _, g, h, i = m[:11]
    det = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g)
    r = ((e*i-f*h)/det, (c*h-b*i)/det, (b*f-c*e)/det,
         (f*g-d*i)/det, (a*i-c*g)/det, (c*d-a*f)/det,
         (d*h-e*g)/det, (b*g-a*h)/det, (a*e-b*d)/det)
    t = (m[3], m[7], m[11])
    return tuple(v for row in range(3) for v in (*r[3*row:3*row+3], -sum(r[3*row+k]*t[k] for k in range(3)))) + (0., 0., 0., 1.)


def normalize(v):
    length = math.sqrt(sum(x*x for x in v))
    if not math.isfinite(length) or length == 0:
        raise ValueError("Expected a finite nonzero direction")
    return tuple(x / length for x in v)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def look_at(eye, target, up):
    forward = normalize(tuple(b-a for a, b in zip(eye, target)))
    right = normalize(cross(normalize(up), forward))
    vertical = cross(forward, right)
    return tuple(v for row in zip(right, vertical, forward, eye) for v in row) + (0., 0., 0., 1.)


def rotation(angle, axis):
    x, y, z = normalize(axis)
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    t = 1-c
    return (t*x*x+c, t*x*y-s*z, t*x*z+s*y, 0.,
            t*x*y+s*z, t*y*y+c, t*y*z-s*x, 0.,
            t*x*z-s*y, t*y*z+s*x, t*z*z+c, 0.,
            0., 0., 0., 1.)
