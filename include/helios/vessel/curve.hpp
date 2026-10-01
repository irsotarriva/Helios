#ifndef HELIOS_VESSEL_CURVE_HPP
#define HELIOS_VESSEL_CURVE_HPP

#include "helios/core/error.hpp"

#include <utility>
#include <vector>

namespace helios::vessel {

// A characterised curve y(x) of a datasheet (BRIEFING §10): linear between its points and
// constant beyond the first and the last one.
class Curve {
public:
    struct Point {
        double x = 0.0;
        double y = 0.0;
    };

    // y = value everywhere.
    [[nodiscard]] static Curve constant(double value);

    // Needs at least one point, finite values and strictly increasing x.
    [[nodiscard]] static core::Result<Curve> make(std::vector<Point> points);

    [[nodiscard]] double operator()(double x) const noexcept;

private:
    explicit Curve(std::vector<Point> points) noexcept : points_(std::move(points)) {}

    std::vector<Point> points_;
};

} // namespace helios::vessel

#endif // HELIOS_VESSEL_CURVE_HPP
