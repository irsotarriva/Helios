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

} // namespace helios::render

#endif // HELIOS_RENDER_MESH_HPP
