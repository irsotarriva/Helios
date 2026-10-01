#include "helios/vessel/curve.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

namespace helios::vessel {

Curve Curve::constant(double value) {
    return Curve({Point{.x = 0.0, .y = value}});
}

core::Result<Curve> Curve::make(std::vector<Point> points) {
    if (points.empty()) {
        return core::fail(core::ErrorCode::InvalidArgument, "a curve needs at least one point");
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (!std::isfinite(points[index].x) || !std::isfinite(points[index].y)) {
            return core::fail(core::ErrorCode::NotFinite, "a curve point is not finite");
        }
        if (index > 0 && !(points[index].x > points[index - 1].x)) {
            return core::fail(core::ErrorCode::InvalidArgument, "curve points must have increasing x");
        }
    }
    return Curve(std::move(points));
}

double Curve::operator()(double x) const noexcept {
    if (!(x > points_.front().x)) { // also a NaN
        return points_.front().y;
    }
    if (x >= points_.back().x) {
        return points_.back().y;
    }
    const auto upper = std::ranges::upper_bound(points_, x, std::less{}, &Point::x);
    const Point& before = *(upper - 1);
    const Point& after = *upper;
    return std::lerp(before.y, after.y, (x - before.x) / (after.x - before.x));
}

} // namespace helios::vessel
