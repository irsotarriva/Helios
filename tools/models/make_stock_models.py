"""Writes the stock models of data/models/stock (BRIEFING D44, D48) as binary glTF files.

They are made here, from boxes and prisms, so that their licence is the project's own. Run from
the repository's root:

    python tools/models/make_stock_models.py

Axes are glTF's, which a look's model is drawn in: x to the right of the panel, y up it, z out
of it towards whoever uses it; metres. A node called `moving` moves as the control does, about
its own origin; materials called `colour`, `accent`, `paint` or `lamp` take the look's colours
(data/models/README.md). A panel's picture is drawn here too, and kept in its file.
"""

import json
import math
import random
import struct
import zlib
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
        self.uvs = None  # surface coordinates, for a mesh with a picture

    def face(self, corners, uvs=None):
        """A flat polygon, counter-clockwise seen from outside; `uvs`, a picture's coordinates
        at its corners (u to the right, v down from the picture's top)."""
        a, b, c = corners[0], corners[1], corners[2]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(x * x for x in n)) or 1.0
        n = [x / length for x in n]
        first = len(self.positions)
        if uvs is not None and self.uvs is None:
            self.uvs = [(0.0, 0.0)] * first
        if self.uvs is not None:
            self.uvs += list(uvs) if uvs is not None else [(0.0, 0.0)] * len(corners)
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

    def material(self, name, srgb, texture=None):
        key = (name, texture)  # one name may be painted both with a picture and without
        if key not in self.material_index:
            self.material_index[key] = len(self.doc["materials"])
            pbr = {"baseColorFactor": [srgb_to_linear(c) for c in srgb] + [1.0],
                   "metallicFactor": 0.0, "roughnessFactor": 0.7}
            if texture is not None:
                pbr["baseColorTexture"] = {"index": texture}
            self.doc["materials"].append({"name": name, "pbrMetallicRoughness": pbr})
        return self.material_index[key]

    def texture(self, png):
        """A PNG kept in the file, smooth between its pixels and its levels, not repeated."""
        self.doc.setdefault("images", []).append({"bufferView": self.view(png, None),
                                                  "mimeType": "image/png"})
        self.doc.setdefault("samplers", [{"magFilter": 9729, "minFilter": 9987,
                                          "wrapS": 33071, "wrapT": 33071}])
        self.doc.setdefault("textures", []).append({"source": len(self.doc["images"]) - 1,
                                                    "sampler": 0})
        return len(self.doc["textures"]) - 1

    def view(self, data, target):
        while len(self.binary) % 4:
            self.binary.append(0)
        view = {"buffer": 0, "byteOffset": len(self.binary), "byteLength": len(data)}
        if target is not None:
            view["target"] = target
        self.doc["bufferViews"].append(view)
        self.binary += data
        return len(self.doc["bufferViews"]) - 1

    def accessor(self, values, kind):
        if kind == "VEC2":
            data = b"".join(struct.pack("<2f", *v) for v in values)
            entry = {"bufferView": self.view(data, 34962), "componentType": 5126,
                     "count": len(values), "type": "VEC2"}
        elif kind == "VEC3":
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
            attributes = {"POSITION": self.accessor(mesh.positions, "VEC3"),
                          "NORMAL": self.accessor(mesh.normals, "VEC3")}
            if mesh.uvs is not None:
                attributes["TEXCOORD_0"] = self.accessor(mesh.uvs, "VEC2")
            primitives.append({"attributes": attributes,
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


def png(width, height, pixel):
    """An 8-bit RGB PNG; `pixel(x, y)` gives each pixel's (r, g, b) from 0 to 255, from the top."""
    rows = bytearray()
    for y in range(height):
        rows.append(0)  # no filter
        for x in range(width):
            rows += bytes(pixel(x, y))
    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(bytes(rows), 9)) + chunk(b"IEND", b""))


def panel_picture(width_px, height_px, width_m, height_m):
    """Painted metal that has been used: grain along the brushing, a darker margin where hands
    and dust have been, fine scratches, and a line engraved round the face. Near white, so that
    the box's colour (the material `colour`) shows through as the paint's."""
    rng = random.Random(1969)
    grain = [rng.uniform(-1.0, 1.0) for _ in range(height_px)]
    shade = [[0.93 + 0.025 * grain[y] + rng.uniform(-0.02, 0.02) for _ in range(width_px)]
             for y in range(height_px)]
    px_per_m = width_px / width_m
    # Grime towards the edges and the bottom, where the panel is touched.
    for y in range(height_px):
        for x in range(width_px):
            edge_m = min(x, width_px - 1 - x, y, height_px - 1 - y) / px_per_m
            shade[y][x] *= 1.0 - 0.10 * math.exp(-edge_m / 0.02) - 0.05 * (y / height_px) ** 3
    # Scratches: short, thin, mostly across, a little lighter than the paint.
    for _ in range(140):
        x, y = rng.uniform(0, width_px), rng.uniform(0, height_px)
        angle = rng.gauss(0.0, 0.5)
        length = rng.uniform(4, 40)
        for step in range(int(length)):
            sx, sy = int(x + step * math.cos(angle)), int(y + step * math.sin(angle))
            if 0 <= sx < width_px and 0 <= sy < height_px:
                shade[sy][sx] = min(1.0, shade[sy][sx] + 0.06)
    # The engraved line, 12 mm in from the edge, two pixels wide.
    inset = round(0.012 * px_per_m)
    for y in range(height_px):
        for x in range(width_px):
            on_side = inset <= y < height_px - inset and (abs(x - inset) < 1 or abs(x - (width_px - 1 - inset)) < 1)
            on_end = inset <= x < width_px - inset and (abs(y - inset) < 1 or abs(y - (height_px - 1 - inset)) < 1)
            if on_side or on_end:
                shade[y][x] *= 0.55
    return png(width_px, height_px,
               lambda x, y: (round(255 * shade[y][x]),) * 3)


def instrument_panel():
    """The lander's instrument panel, 1.2 m by 0.4 m and 4 cm thick, about the centre of the box
    it stands for: a painted face with its picture, screwed to a frame round its edge."""
    gltf = Gltf()
    width, height, depth = 1.2, 0.4, 0.04
    hx, hy, hz = width / 2, height / 2, depth / 2
    paint = gltf.material("colour", (1.0, 1.0, 1.0), texture=gltf.texture(
        panel_picture(768, 256, width, height)))
    face = Mesh()
    face.face([(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)],
              uvs=[(0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)])
    body = Mesh()
    body.face([(hx, -hy, -hz), (-hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz)])  # back
    body.face([(hx, -hy, hz), (hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz)])      # right
    body.face([(-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz)])  # left
    body.face([(-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz), (-hx, hy, -hz)])      # top
    body.face([(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)])  # bottom
    frame = Mesh()
    rim, proud = 0.008, 0.003
    for centre, size in (((0.0, hy - rim / 2, hz + proud / 2), (width, rim, proud)),
                         ((0.0, -hy + rim / 2, hz + proud / 2), (width, rim, proud)),
                         ((hx - rim / 2, 0.0, hz + proud / 2), (rim, height - 2 * rim, proud)),
                         ((-hx + rim / 2, 0.0, hz + proud / 2), (rim, height - 2 * rim, proud))):
        frame.box(centre, size)
    screws = Mesh()
    for x in (-hx + 0.006, -hx / 3, hx / 3, hx - 0.006):
        for y in (-hy + 0.0045, hy - 0.0045):
            heads = Mesh()
            heads.prism(10, 0.0032, 0.0026, 0.0, 0.0016)
            screws.positions += [(px + x, py + y, pz + hz + proud) for px, py, pz in heads.positions]
            first = len(screws.normals)
            screws.normals += heads.normals
            screws.indices += [first + i for i in heads.indices]
    gltf.node("instrument_panel", mesh=gltf.mesh([
        (face, paint), (body, gltf.material("colour", (1.0, 1.0, 1.0))),
        (frame, gltf.material("frame", (0.16, 0.16, 0.17))), (screws, gltf.material("screw", METAL))]))
    gltf.write(OUT / "instrument_panel.glb")


if __name__ == "__main__":
    toggle()
    pushbutton()
    instrument_panel()
    print(f"wrote {OUT}")
