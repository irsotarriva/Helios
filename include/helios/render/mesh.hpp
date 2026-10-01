#ifndef HELIOS_RENDER_MESH_HPP
#define HELIOS_RENDER_MESH_HPP

#include "helios/render/camera.hpp"

#include <cstdint>
#include <vector>

namespace helios::render {

// A unit sphere; each vertex is also its own normal.
struct SphereMesh {
    std::vector<Float3> vertices;
    std::vector<std::uint16_t> indices; // counter-clockwise triangles seen from outside
};

// Icosphere: an icosahedron with each triangle split `subdivisions` times (4^n × 20 triangles),
// vertices projected onto the sphere. Nearly uniform triangles, unlike a UV sphere, and no
// seam or pole singularity. At most 5 subdivisions (16-bit indices).
[[nodiscard]] SphereMesh make_icosphere(int subdivisions);

struct MeshVertex {
    Float3 position;
    Float3 normal;
    float u = 0.0F; // surface coordinates, for whatever pattern the shader draws
    float v = 0.0F;
};

// Triangles are counter-clockwise seen from outside, as for the sphere.
struct Mesh {
    std::vector<MeshVertex> vertices;
    std::vector<std::uint16_t> indices;
};

// A cube with edges of 1 centred on the origin, with flat faces (four vertices each).
[[nodiscard]] Mesh make_box();

// A cylinder of radius 1 and length 1 along x, centred on the origin, with flat end caps.
[[nodiscard]] Mesh make_cylinder(int segments);

} // namespace helios::render

#endif // HELIOS_RENDER_MESH_HPP
