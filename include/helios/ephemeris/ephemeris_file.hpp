#ifndef HELIOS_EPHEMERIS_EPHEMERIS_FILE_HPP
#define HELIOS_EPHEMERIS_EPHEMERIS_FILE_HPP

#include "helios/core/error.hpp"
#include "helios/ephemeris/chebyshev_ephemeris.hpp"
#include "helios/ephemeris/keplerian_ephemeris.hpp"

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace helios::ephemeris {

// One body's motion relative to a named centre (the parent frame).
struct NamedChebyshevSegment {
    std::string target_name;
    std::string center_name;
    ChebyshevEphemeris ephemeris;
};

struct NamedSecularElements {
    std::string target_name;
    std::string center_name;
    KeplerianEphemeris ephemeris;
};

// Helios Chebyshev ephemeris file (".hce"), all fields little-endian:
//
//   char[8]  magic "HLSCHEB1"
//   uint32   segment_count
//   segment_count × {
//     uint32 target_name_length, char[] target_name (UTF-8)
//     uint32 center_name_length, char[] center_name (UTF-8)
//     int64  start_whole_seconds     (TDB since the universe reference epoch)
//     double start_fraction_s        ([0, 1))
//     double record_duration_s
//     uint32 record_count
//     uint32 coefficients_per_axis
//     double coefficients_m[record_count × 3 × coefficients_per_axis]
//   }
//
// Produced by tools/ephemeris/export_de_chebyshev.py. Segments are listed parent-first.
[[nodiscard]] core::Result<std::vector<NamedChebyshevSegment>>
parse_chebyshev_file(std::span<const std::byte> file_bytes) noexcept;

[[nodiscard]] core::Result<std::vector<NamedChebyshevSegment>>
load_chebyshev_file(const std::filesystem::path& path) noexcept;

// Mean-element table (CSV, '#' comments, one header row). Columns, in order:
//   target, center, reference_epoch_tdb_s,
//   semi_major_axis_m, semi_major_axis_rate_m_s, eccentricity, eccentricity_rate_per_s,
//   inclination_rad, inclination_rate_rad_s, longitude_of_ascending_node_rad,
//   longitude_of_ascending_node_rate_rad_s, longitude_of_periapsis_rad,
//   longitude_of_periapsis_rate_rad_s, mean_longitude_rad, mean_longitude_rate_rad_s
[[nodiscard]] core::Result<std::vector<NamedSecularElements>>
parse_secular_elements_csv(std::string_view csv_text) noexcept;

[[nodiscard]] core::Result<std::vector<NamedSecularElements>>
load_secular_elements_csv(const std::filesystem::path& path) noexcept;

} // namespace helios::ephemeris

#endif // HELIOS_EPHEMERIS_EPHEMERIS_FILE_HPP
