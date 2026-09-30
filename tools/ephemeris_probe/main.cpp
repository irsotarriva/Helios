// helios_ephemeris_probe — evaluates the Helios frame tree at given epochs, for validation.
//
//   helios_ephemeris_probe <ephemeris.hce> <mean_elements.csv | -> <pairs.csv> <epochs.csv> <output.csv>
//
// pairs.csv:  one "target,observer" frame-name pair per line.
// epochs.csv: one "whole_seconds,fraction_s" TDB epoch (since J2000) per line.
// Mean-element bodies are added as "<target> (mean elements)" under their centre frame.
// Output columns: target,observer,whole_s,fraction_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,ax_m_s2,ay_m_s2,az_m_s2

#include "helios/core/error.hpp"
#include "helios/core/parse.hpp"
#include "helios/ephemeris/ephemeris_file.hpp"
#include "helios/frames/frame_tree.hpp"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using helios::core::ErrorCode;
using helios::core::Result;
using helios::core::VoidResult;
using helios::frames::FrameId;
using helios::frames::FrameTree;
using helios::time::Epoch;

[[nodiscard]] Result<std::vector<std::string>> read_lines(const std::filesystem::path& path) {
    return helios::core::try_call(ErrorCode::IoFailure, "reading input",
                                  [&]() -> Result<std::vector<std::string>> {
                                      std::ifstream stream(path);
                                      if (!stream) {
                                          return helios::core::fail(
                                              ErrorCode::FileNotFound,
                                              std::format("cannot open '{}'", path.string()));
                                      }
                                      std::vector<std::string> lines;
                                      for (std::string line; std::getline(stream, line);) {
                                          if (!line.empty() && line.back() == '\r') {
                                              line.pop_back();
                                          }
                                          if (!line.empty() && line.front() != '#') {
                                              lines.push_back(std::move(line));
                                          }
                                      }
                                      return lines;
                                  })
        .and_then([](Result<std::vector<std::string>> lines) { return lines; });
}

[[nodiscard]] Result<std::pair<std::string_view, std::string_view>> split_pair(std::string_view line) {
    const std::size_t comma = line.find(',');
    if (comma == std::string_view::npos) {
        return helios::core::fail(ErrorCode::ParseFailure,
                                  std::format("expected two comma-separated fields: '{}'", line));
    }
    return std::pair{line.substr(0, comma), line.substr(comma + 1)};
}

[[nodiscard]] Result<Epoch> parse_epoch(std::string_view line) {
    return split_pair(line).and_then(
        [&](std::pair<std::string_view, std::string_view> fields) -> Result<Epoch> {
            const auto whole_seconds = helios::core::parse_int64(fields.first);
            const auto fraction_s = helios::core::parse_double(fields.second);
            if (!whole_seconds || !fraction_s) {
                return helios::core::fail(ErrorCode::ParseFailure, std::format("bad epoch line '{}'", line));
            }
            return Epoch::from_parts(*whole_seconds, *fraction_s);
        });
}

[[nodiscard]] Result<FrameTree> build_tree(const std::filesystem::path& chebyshev_path,
                                           const std::filesystem::path& mean_elements_path) {
    auto segments = helios::ephemeris::load_chebyshev_file(chebyshev_path);
    if (!segments) {
        return std::unexpected(segments.error());
    }
    if (segments->empty()) {
        return helios::core::fail(ErrorCode::InvalidArgument, "ephemeris file has no segments");
    }
    FrameTree tree(segments->front().center_name);
    for (auto& segment : *segments) {
        auto added = tree.find(segment.center_name).and_then([&](FrameId center) {
            return tree.add_frame(segment.target_name, center, std::move(segment.ephemeris));
        });
        if (!added) {
            return std::unexpected(added.error());
        }
    }
    if (mean_elements_path != "-") {
        auto bodies = helios::ephemeris::load_secular_elements_csv(mean_elements_path);
        if (!bodies) {
            return std::unexpected(bodies.error());
        }
        for (auto& body : *bodies) {
            auto added = tree.find(body.center_name).and_then([&](FrameId center) {
                return tree.add_frame(body.target_name + " (mean elements)", center, body.ephemeris);
            });
            if (!added) {
                return std::unexpected(added.error());
            }
        }
    }
    return tree;
}

[[nodiscard]] VoidResult run(std::span<char*> arguments) {
    if (arguments.size() != 6) {
        return helios::core::fail(ErrorCode::InvalidArgument,
                                  "usage: helios_ephemeris_probe <ephemeris.hce> <mean_elements.csv|-> "
                                  "<pairs.csv> <epochs.csv> <output.csv>");
    }
    auto tree = build_tree(arguments[1], arguments[2]);
    auto pair_lines = tree.and_then([&](const FrameTree&) { return read_lines(arguments[3]); });
    auto epoch_lines = pair_lines.and_then([&](const auto&) { return read_lines(arguments[4]); });
    if (!epoch_lines) {
        return std::unexpected(epoch_lines.error());
    }

    std::vector<Epoch> epochs;
    epochs.reserve(epoch_lines->size());
    for (const std::string& line : *epoch_lines) {
        auto epoch = parse_epoch(line);
        if (!epoch) {
            return std::unexpected(epoch.error());
        }
        epochs.push_back(*epoch);
    }

    return helios::core::try_call(
               ErrorCode::IoFailure, "writing output",
               [&]() -> VoidResult {
                   std::ofstream output(arguments[5]);
                   if (!output) {
                       return helios::core::fail(ErrorCode::IoFailure, "cannot open the output file");
                   }
                   output << "target,observer,whole_s,fraction_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,ax_m_s2,ay_"
                             "m_s2,az_m_s2\n";
                   for (const std::string& line : *pair_lines) {
                       auto names = split_pair(line);
                       if (!names) {
                           return std::unexpected(names.error());
                       }
                       auto target = tree->find(names->first);
                       auto observer = target.and_then([&](FrameId) { return tree->find(names->second); });
                       if (!observer) {
                           return std::unexpected(observer.error());
                       }
                       for (const Epoch& epoch : epochs) {
                           auto state = tree->relative_state(*target, *observer, epoch);
                           auto acceleration = state.and_then([&](const auto&) {
                               return tree->relative_acceleration_m_s2(*target, *observer, epoch);
                           });
                           if (!acceleration) {
                               return std::unexpected(acceleration.error());
                           }
                           output << std::format(
                               "{},{},{},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},"
                               "{:.17g},{:.17g},{:.17g}\n",
                               names->first, names->second, epoch.whole_seconds(), epoch.fraction(),
                               state->position_m.x, state->position_m.y, state->position_m.z,
                               state->velocity_m_s.x, state->velocity_m_s.y, state->velocity_m_s.z,
                               acceleration->x, acceleration->y, acceleration->z);
                       }
                   }
                   return {};
               })
        .and_then([](VoidResult result) { return result; });
}

} // namespace

int main(int argc, char** argv) {
    const VoidResult result = run(std::span<char*>{argv, static_cast<std::size_t>(argc)});
    if (!result) {
        std::fputs(
            std::format("helios_ephemeris_probe: {}\n", helios::core::describe(result.error())).c_str(),
            stderr);
        return 1;
    }
    return 0;
}
