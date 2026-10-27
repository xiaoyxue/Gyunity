"""Lossless text serialization of source triangle arrays for the native OBJ loader."""
import math

from .transforms import cross


def encode_obj(positions, indices, normals=(), uv=(), *, reverse=False) -> bytes:
    if len(positions) % 3 or len(indices) % 3 or not indices:
        raise ValueError("Triangle meshes need point triples and index triples")
    vertices = [positions[i:i+3] for i in range(0, len(positions), 3)]
    if (normals and len(normals) != len(positions)) or (uv and len(uv) != len(vertices) * 2):
        raise ValueError("Mesh normal/UV counts must match vertex count")
    if not all(math.isfinite(x) for values in (positions, normals, uv) for x in values):
        raise ValueError("Nonfinite mesh data")
    if any(i < 0 or i >= len(vertices) for i in indices):
        raise ValueError("Triangle index is outside the vertex array")
    lines = ["# Converted from source triangle arrays; geometry is not approximated."]
    lines.extend("v " + " ".join(format(x, ".17g") for x in vertex) for vertex in vertices)
    for i in range(0, len(normals), 3):
        n = normals[i:i+3]
        if sum(x*x for x in n) == 0:
            raise ValueError("Vertex normal cannot be zero")
        lines.append("vn " + " ".join(format(-x if reverse else x, ".17g") for x in n))
    for i in range(0, len(uv), 2):
        lines.append("vt " + " ".join(format(x, ".17g") for x in uv[i:i+2]))
    for i in range(0, len(indices), 3):
        face = list(indices[i:i+3])
        a, b, c = (vertices[index] for index in face)
        if sum(x*x for x in cross(tuple(y-x for x, y in zip(a, b)), tuple(y-x for x, y in zip(a, c)))) == 0:
            raise ValueError("Degenerate source triangle")
        if reverse:
            face.reverse()
        def index(n):
            n += 1
            return f"{n}/{n if uv else ''}/{n}" if normals else (f"{n}/{n}" if uv else str(n))
        lines.append("f " + " ".join(index(n) for n in face))
    return ("\n".join(lines) + "\n").encode("utf-8")
