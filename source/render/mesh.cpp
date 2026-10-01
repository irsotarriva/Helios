#include "helios/render/mesh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <utility>

namespace helios::render {

namespace {

[[nodiscard]] Float3 normalised(float x, float y, float z) noexcept {
    const float length = std::sqrt((x * x) + (y * y) + (z * z));
    return {x / length, y / length, z / length};
}

} // namespace

SphereMesh make_icosphere(int subdivisions) {
    subdivisions = std::clamp(subdivisions, 0, 5);
    constexpr float k_golden = std::numbers::phi_v<float>;
    SphereMesh mesh;
    mesh.vertices = {normalised(-1, k_golden, 0),  normalised(1, k_golden, 0),   normalised(-1, -k_golden, 0),
                     normalised(1, -k_golden, 0),  normalised(0, -1, k_golden),  normalised(0, 1, k_golden),
                     normalised(0, -1, -k_golden), normalised(0, 1, -k_golden),  normalised(k_golden, 0, -1),
                     normalised(k_golden, 0, 1),   normalised(-k_golden, 0, -1), normalised(-k_golden, 0, 1)};
    mesh.indices = {0, 11, 5,  0, 5,  1, 0, 1, 7, 0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11,
                    4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3,  9,  4, 3,  4,  2, 3, 2, 6, 3,
                    6, 8,  3,  8, 9,  4, 9, 5, 2, 4, 11, 6,  2, 10, 8,  6, 7, 9, 8, 1};

    for (int level = 0; level < subdivisions; ++level) {
        std::map<std::pair<std::uint16_t, std::uint16_t>, std::uint16_t> midpoints;
        const auto midpoint = [&](std::uint16_t first, std::uint16_t second) {
            const auto key = std::minmax(first, second);
            if (const auto found = midpoints.find(key); found != midpoints.end()) {
                return found->second;
            }
            const Float3& a = mesh.vertices[first];
            const Float3& b = mesh.vertices[second];
            mesh.vertices.push_back(normalised(a.x + b.x, a.y + b.y, a.z + b.z));
            const auto index = static_cast<std::uint16_t>(mesh.vertices.size() - 1);
            midpoints.emplace(key, index);
            return index;
        };
        std::vector<std::uint16_t> refined;
        refined.reserve(mesh.indices.size() * 4);
        for (std::size_t triangle = 0; triangle < mesh.indices.size(); triangle += 3) {
            const std::uint16_t a = mesh.indices[triangle];
            const std::uint16_t b = mesh.indices[triangle + 1];
            const std::uint16_t c = mesh.indices[triangle + 2];
            const std::uint16_t ab = midpoint(a, b);
            const std::uint16_t bc = midpoint(b, c);
            const std::uint16_t ca = midpoint(c, a);
            refined.insert(refined.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
        }
        mesh.indices = std::move(refined);
    }
    return mesh;
}

Mesh make_box() {
    struct Face {
        Float3 normal;
        Float3 across; // across × along = normal
        Float3 along;
    };
    constexpr std::array<Face, 6> k_faces{{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                                           {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}},
                                           {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}},
                                           {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
                                           {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},
                                           {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}}};
    constexpr std::array<std::array<float, 2>, 4> k_corners{
        {{-0.5F, -0.5F}, {0.5F, -0.5F}, {0.5F, 0.5F}, {-0.5F, 0.5F}}};
    Mesh mesh;
    for (const Face& face : k_faces) {
        const auto first = static_cast<std::uint16_t>(mesh.vertices.size());
        for (const auto& corner : k_corners) {
            mesh.vertices.push_back(MeshVertex{
                .position =
                    {(0.5F * face.normal.x) + (corner[0] * face.across.x) + (corner[1] * face.along.x),
                     (0.5F * face.normal.y) + (corner[0] * face.across.y) + (corner[1] * face.along.y),
                     (0.5F * face.normal.z) + (corner[0] * face.across.z) + (corner[1] * face.along.z)},
                .normal = face.normal});
        }
        for (const int corner : {0, 1, 2, 0, 2, 3}) {
            mesh.indices.push_back(static_cast<std::uint16_t>(first + corner));
        }
    }
    return mesh;
}

Mesh make_cylinder(int segments) {
    segments = std::clamp(segments, 3, 256);
    Mesh mesh;
    const auto count = static_cast<std::uint16_t>(segments);
    const auto around = [&](int step) {
        const float angle = 2.0F * std::numbers::pi_v<float>
                            * static_cast<float>(step % segments) / static_cast<float>(segments);
        return Float3{0.0F, std::cos(angle), std::sin(angle)};
    };
    // The side: a pair of vertices (back, front) at each step around, the seam doubled.
    for (int step = 0; step <= segments; ++step) {
        const Float3 normal = around(step);
        mesh.vertices.push_back(MeshVertex{.position = {-0.5F, normal.y, normal.z}, .normal = normal});
        mesh.vertices.push_back(MeshVertex{.position = {0.5F, normal.y, normal.z}, .normal = normal});
    }
    for (std::uint16_t step = 0; step < count; ++step) {
        const auto back = static_cast<std::uint16_t>(2 * step);
        const auto front = static_cast<std::uint16_t>(back + 1);
        const auto next_back = static_cast<std::uint16_t>(back + 2);
        const auto next_front = static_cast<std::uint16_t>(back + 3);
        mesh.indices.insert(mesh.indices.end(), {back, next_back, next_front, back, next_front, front});
    }
    // The caps: a centre and a ring each.
    for (const float side : {1.0F, -1.0F}) {
        const Float3 normal{side, 0.0F, 0.0F};
        const auto centre = static_cast<std::uint16_t>(mesh.vertices.size());
        mesh.vertices.push_back(MeshVertex{.position = {0.5F * side, 0.0F, 0.0F}, .normal = normal});
        for (int step = 0; step < segments; ++step) {
            const Float3 rim = around(step);
            mesh.vertices.push_back(MeshVertex{.position = {0.5F * side, rim.y, rim.z}, .normal = normal});
        }
        for (std::uint16_t step = 0; step < count; ++step) {
            const auto here = static_cast<std::uint16_t>(centre + 1 + step);
            const auto next = static_cast<std::uint16_t>(centre + 1 + ((step + 1) % count));
            if (side > 0.0F) {
                mesh.indices.insert(mesh.indices.end(), {centre, here, next});
            } else {
                mesh.indices.insert(mesh.indices.end(), {centre, next, here});
            }
        }
    }
    return mesh;
}

} // namespace helios::render
