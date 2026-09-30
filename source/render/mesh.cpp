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

} // namespace helios::render
