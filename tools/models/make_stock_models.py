"""Writes the stock models of data/models/stock (BRIEFING D44) as binary glTF files.

They are made here, from boxes and prisms, so that their licence is the project's own. Run from
the repository's root:

    python tools/models/make_stock_models.py

Axes are glTF's, which a look's model is drawn in: x to the right of the panel, y up it, z out
of it towards whoever uses it; metres. A node called `moving` moves as the control does, about
its own origin; materials called `colour`, `accent`, `paint` or `lamp` take the look's colours
(data/models/README.md).
"""

import json
import math
import struct
from pathlib import Path

OUT = Path(__file__).resolve().parents[2] / "data" / "models" / "stock"


def srgb_to_linear(value):
    return value / 12.92 if value <= 0.04045 else ((value + 0.055) / 1.055) ** 2.4


class Mesh:
    """Triangles with flat normals: each face has its own vertices."""

    def __init__(self):
        self.positions = []
        self.normals = []
        self.indices = []

    def face(self, corners):
        """A flat polygon, counter-clockwise seen from outside."""
        a, b, c = corners[0], corners[1], corners[2]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(x * x for x in n)) or 1.0
        n = [x / length for x in n]
        first = len(self.positions)
        for corner in corners:
            self.positions.append(corner)
            self.normals.append(n)
        for k in range(1, len(corners) - 1):
            self.indices += [first, first + k, first + k + 1]

    def box(self, centre, size):
        cx, cy, cz = centre
        hx, hy, hz = (s / 2 for s in size)
        p = lambda sx, sy, sz: (cx + sx * hx, cy + sy * hy, cz + sz * hz)
        self.face([p(-1, -1, 1), p(1, -1, 1), p(1, 1, 1), p(-1, 1, 1)])      # out
        self.face([p(1, -1, -1), p(-1, -1, -1), p(-1, 1, -1), p(1, 1, -1)])  # in
        self.face([p(1, -1, 1), p(1, -1, -1), p(1, 1, -1), p(1, 1, 1)])      # right
        self.face([p(-1, -1, -1), p(-1, -1, 1), p(-1, 1, 1), p(-1, 1, -1)])  # left
        self.face([p(-1, 1, 1), p(1, 1, 1), p(1, 1, -1), p(-1, 1, -1)])      # top
        self.face([p(-1, -1, -1), p(1, -1, -1), p(1, -1, 1), p(-1, -1, 1)])  # bottom

    def prism(self, sides, radius_low, radius_high, z_low, z_high, turn=0.0):
        """A prism along z, wider or narrower at its top: a nut, a bat handle."""
        def ring(radius, z):
            return [(radius * math.cos(turn + 2 * math.pi * k / sides),
                     radius * math.sin(turn + 2 * math.pi * k / sides), z) for k in range(sides)]
        low, high = ring(radius_low, z_low), ring(radius_high, z_high)
        self.face(high)
        self.face(list(reversed(low)))
        for k in range(sides):
            j = (k + 1) % sides
            self.face([low[k], low[j], high[j], high[k]])


class Gltf:
    def __init__(self):
        self.doc = {"asset": {"version": "2.0", "generator": "tools/models/make_stock_models.py"},
                    "scene": 0, "scenes": [{"nodes": []}], "nodes": [], "meshes": [],
                    "materials": [], "accessors": [], "bufferViews": [], "buffers": []}
        self.binary = bytearray()
        self.material_index = {}

    def material(self, name, srgb):
        if name not in self.material_index:
            self.material_index[name] = len(self.doc["materials"])
            self.doc["materials"].append({"name": name, "pbrMetallicRoughness": {
                "baseColorFactor": [srgb_to_linear(c) for c in srgb] + [1.0],
                "metallicFactor": 0.0, "roughnessFactor": 0.7}})
        return self.material_index[name]

    def view(self, data, target):
        while len(self.binary) % 4:
            self.binary.append(0)
        self.doc["bufferViews"].append({"buffer": 0, "byteOffset": len(self.binary),
                                        "byteLength": len(data), "target": target})
        self.binary += data
        return len(self.doc["bufferViews"]) - 1

    def accessor(self, values, kind):
        if kind == "VEC3":
            data = b"".join(struct.pack("<3f", *v) for v in values)
            view = self.view(data, 34962)
            lows = [min(v[i] for v in values) for i in range(3)]
            highs = [max(v[i] for v in values) for i in range(3)]
            entry = {"bufferView": view, "componentType": 5126, "count": len(values), "type": "VEC3",
                     "min": lows, "max": highs}
        else:
            data = struct.pack(f"<{len(values)}I", *values)
            view = self.view(data, 34963)
            entry = {"bufferView": view, "componentType": 5125, "count": len(values), "type": "SCALAR"}
        self.doc["accessors"].append(entry)
        return len(self.doc["accessors"]) - 1

    def mesh(self, pieces):
        """pieces: (Mesh, material index) pairs, one primitive each."""
        primitives = []
        for mesh, material in pieces:
            primitives.append({"attributes": {"POSITION": self.accessor(mesh.positions, "VEC3"),
                                              "NORMAL": self.accessor(mesh.normals, "VEC3")},
                               "indices": self.accessor(mesh.indices, "SCALAR"), "material": material})
        self.doc["meshes"].append({"primitives": primitives})
        return len(self.doc["meshes"]) - 1

    def node(self, name, mesh=None, translation=None, parent=None):
        node = {"name": name}
        if mesh is not None:
            node["mesh"] = mesh
        if translation is not None:
            node["translation"] = list(translation)
        self.doc["nodes"].append(node)
        index = len(self.doc["nodes"]) - 1
        if parent is None:
            self.doc["scenes"][0]["nodes"].append(index)
        else:
            self.doc["nodes"][parent].setdefault("children", []).append(index)
        return index

    def write(self, path):
        while len(self.binary) % 4:
            self.binary.append(0)
        self.doc["buffers"] = [{"byteLength": len(self.binary)}]
        text = json.dumps(self.doc, separators=(",", ":")).encode()
        text += b" " * ((4 - len(text) % 4) % 4)
        total = 12 + 8 + len(text) + 8 + len(self.binary)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as out:
            out.write(struct.pack("<4sII", b"glTF", 2, total))
            out.write(struct.pack("<I4s", len(text), b"JSON") + text)
            out.write(struct.pack("<I4s", len(self.binary), b"BIN\0") + bytes(self.binary))


METAL = (0.62, 0.62, 0.60)
DARK = (0.08, 0.08, 0.09)


def toggle():
    """A bat-handle toggle on its plate: the plate takes the look's colour, the handle its accent."""
    gltf = Gltf()
    plate = Mesh()
    plate.box((0.0, 0.0, 0.0), (0.04, 0.064, 0.006))
    nut = Mesh()
    nut.prism(6, 0.0075, 0.0075, 0.003, 0.0065, turn=math.pi / 6)
    bushing = Mesh()
    bushing.prism(12, 0.0045, 0.0045, 0.0065, 0.009)
    body = gltf.mesh([(plate, gltf.material("colour", DARK)), (nut, gltf.material("nut", METAL)),
                      (bushing, gltf.material("nut", METAL))])
    root = gltf.node("toggle", mesh=body)
    handle = Mesh()
    handle.prism(8, 0.0022, 0.0038, 0.0, 0.024)
    tip = Mesh()
    tip.prism(8, 0.0038, 0.0026, 0.024, 0.0265)
    gltf.node("moving", mesh=gltf.mesh([(handle, gltf.material("accent", METAL)),
                                        (tip, gltf.material("accent", METAL))]),
              translation=(0.0, 0.0, 0.007), parent=root)
    gltf.write(OUT / "toggle.glb")


def pushbutton():
    """A square push-button: a dark bezel round a cap that is lit as the look's lamp."""
    gltf = Gltf()
    bezel = Mesh()
    for centre, size in (((0.0, 0.0235, 0.004), (0.05, 0.003, 0.008)),
                         ((0.0, -0.0235, 0.004), (0.05, 0.003, 0.008)),
                         ((0.0235, 0.0, 0.004), (0.003, 0.044, 0.008)),
                         ((-0.0235, 0.0, 0.004), (0.003, 0.044, 0.008))):
        bezel.box(centre, size)
    back = Mesh()
    back.box((0.0, 0.0, 0.001), (0.044, 0.044, 0.002))
    cap = Mesh()
    cap.box((0.0, 0.0, 0.007), (0.042, 0.042, 0.012))
    gltf.node("pushbutton", mesh=gltf.mesh([(bezel, gltf.material("bezel", DARK)),
                                            (back, gltf.material("bezel", DARK)),
                                            (cap, gltf.material("lamp", (1.0, 0.84, 0.43)))]))
    gltf.write(OUT / "pushbutton.glb")


if __name__ == "__main__":
    toggle()
    pushbutton()
    print(f"wrote {OUT}")
