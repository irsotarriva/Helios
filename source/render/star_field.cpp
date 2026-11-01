#include "helios/render/star_field.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace helios::render {

namespace {

using math::Vector3;

// Rationale: a generator of our own rather than <random>'s distributions, whose results differ
// between standard libraries; the sky has to be the same everywhere.
class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) noexcept : state_(seed) {}

    [[nodiscard]] std::uint64_t next() noexcept {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t mixed = state_;
        mixed = (mixed ^ (mixed >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        mixed = (mixed ^ (mixed >> 27U)) * 0x94D049BB133111EBULL;
        return mixed ^ (mixed >> 31U);
    }
    // In [0, 1).
    [[nodiscard]] double unit() noexcept {
        constexpr double k_scale = 0x1.0p-53;
        return static_cast<double>(next() >> 11U) * k_scale;
    }

private:
    std::uint64_t state_;
};

// The north pole of the galaxy (12h 51.4m, +27.13°) in the J2000 ecliptic axes: latitude
// 29.81°, longitude 180.02°.
constexpr Vector3 k_galactic_pole{-0.8677, -0.0003, 0.4970};
// Of the stars this many lie along the Milky Way, within about this galactic latitude.
constexpr double k_milky_way_fraction = 0.45;
constexpr double k_milky_way_width_rad = 0.25;
// Magnitudes: as many stars as the sky has down to the naked eye's limit (the number brighter
// than m grows as 10^(m/2)), none brighter than the brightest.
constexpr double k_faintest_magnitude = 6.5;
constexpr double k_brightest_magnitude = -1.5;
// Brightness on the screen: a first-magnitude star is full white, and the scale is squeezed
// (half the magnitudes' steps) so that the faint ones are not lost; none fainter than the floor.
constexpr double k_full_magnitude = 1.0;
constexpr double k_least_brightness = 0.3;

struct Tint {
    double share = 0.0;
    std::array<double, 3> rgb{};
};
// Hot stars blue-white, the Sun's kind yellowish, cool giants orange.
constexpr std::array<Tint, 4> k_tints{{{.share = 0.25, .rgb = {0.80, 0.87, 1.00}},
                                       {.share = 0.35, .rgb = {1.00, 1.00, 1.00}},
                                       {.share = 0.25, .rgb = {1.00, 0.95, 0.85}},
                                       {.share = 0.15, .rgb = {1.00, 0.80, 0.62}}}};

[[nodiscard]] Vector3 uniform_direction(SplitMix64& random) noexcept {
    const double z = (2.0 * random.unit()) - 1.0;
    const double longitude_rad = 2.0 * std::numbers::pi * random.unit();
    const double ring = std::sqrt(std::max(0.0, 1.0 - (z * z)));
    return {ring * std::cos(longitude_rad), ring * std::sin(longitude_rad), z};
}

[[nodiscard]] Vector3 milky_way_direction(SplitMix64& random) noexcept {
    const Vector3 pole = k_galactic_pole / math::norm(k_galactic_pole);
    const Vector3 node = math::cross(pole, Vector3{0.0, 0.0, 1.0});
    const Vector3 first = node / math::norm(node);
    const Vector3 second = math::cross(pole, first);
    // Box–Muller: a normal latitude about the galactic plane.
    const double radius = std::sqrt(-2.0 * std::log(1.0 - random.unit()));
    const double angle_rad = 2.0 * std::numbers::pi * random.unit();
    const double latitude_rad = std::clamp(k_milky_way_width_rad * radius * std::cos(angle_rad),
                                           -0.5 * std::numbers::pi, 0.5 * std::numbers::pi);
    const double longitude_rad = 2.0 * std::numbers::pi * random.unit();
    return (std::cos(latitude_rad) * std::cos(longitude_rad)) * first
           + (std::cos(latitude_rad) * std::sin(longitude_rad)) * second + std::sin(latitude_rad) * pole;
}

[[nodiscard]] std::uint8_t channel(double value) noexcept {
    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
}

} // namespace

std::vector<Star> make_star_field(std::size_t count, std::uint64_t seed) {
    SplitMix64 random(seed);
    std::vector<Star> stars;
    stars.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        Vector3 direction =
            random.unit() < k_milky_way_fraction ? milky_way_direction(random) : uniform_direction(random);
        direction = direction / math::norm(direction);
        const double magnitude =
            std::max(k_brightest_magnitude, k_faintest_magnitude + (2.0 * std::log10(1.0 - random.unit())));
        const double brightness =
            std::clamp(std::pow(10.0, -0.2 * (magnitude - k_full_magnitude)), k_least_brightness, 1.0);
        double pick = random.unit();
        const Tint* tint = &k_tints.back();
        for (const Tint& candidate : k_tints) {
            if (pick < candidate.share) {
                tint = &candidate;
                break;
            }
            pick -= candidate.share;
        }
        stars.push_back(
            Star{.direction = direction,
                 .brightness = static_cast<float>(brightness),
                 .abgr = rgba(channel(tint->rgb[0] * brightness), channel(tint->rgb[1] * brightness),
                              channel(tint->rgb[2] * brightness))});
    }
    return stars;
}

std::vector<LineVertex> star_lines(std::span<const Star> stars) {
    std::vector<LineVertex> lines;
    lines.reserve(2 * stars.size());
    const double half_m = 0.5 * k_star_length_rad * k_star_distance_m;
    for (const Star& star : stars) {
        const Vector3& direction = star.direction;
        const Vector3 any = std::abs(direction.z) < 0.9 ? Vector3{0.0, 0.0, 1.0} : Vector3{1.0, 0.0, 0.0};
        const Vector3 across = math::cross(direction, any) / math::norm(math::cross(direction, any));
        const Vector3 centre_m = k_star_distance_m * direction;
        for (const Vector3& end_m : {centre_m - half_m * across, centre_m + half_m * across}) {
            lines.push_back(LineVertex{.position = {static_cast<float>(end_m.x), static_cast<float>(end_m.y),
                                                    static_cast<float>(end_m.z)},
                                       .abgr = star.abgr});
        }
    }
    return lines;
}

} // namespace helios::render
