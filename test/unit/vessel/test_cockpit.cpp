#include "helios/vessel/cockpit.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <string>

#include "support.hpp"

namespace {

using helios::core::ErrorCode;
using helios::math::Vector3;
using helios::vessel::Cockpit;
using helios::vessel::Instrument;
using helios::vessel::PartCatalog;

[[nodiscard]] helios::core::Result<Cockpit> parse(const std::string& cockpit_toml) {
    const auto document = helios::core::parse_toml("[cockpit]\n" + cockpit_toml);
    EXPECT_TRUE(document.has_value());
    return document->at("cockpit").and_then(helios::vessel::parse_cockpit);
}

TEST(Cockpit, TheStockCrewedPartsHaveASeatAndInstruments) {
    const PartCatalog catalog = helios::test::load_stock_parts();
    const helios::vessel::PartDatasheet& pod = **catalog.part("core.command_pod");
    ASSERT_TRUE(pod.cockpit.has_value());
    EXPECT_FALSE(pod.cockpit->boxes.empty());
    const auto throttle = std::ranges::find(pod.cockpit->instruments, "engine/throttle", &Instrument::signal);
    ASSERT_NE(throttle, pod.cockpit->instruments.end());
    EXPECT_EQ(throttle->kind, Instrument::Kind::Lever);
    // On the console: it faces up, and is pushed forward for more.
    EXPECT_EQ(throttle->facing, (Vector3{0.0, 0.0, 1.0}));
    EXPECT_EQ(throttle->up, (Vector3{1.0, 0.0, 0.0}));
    const auto stage = std::ranges::find(pod.cockpit->instruments, "STAGE", &Instrument::label);
    ASSERT_NE(stage, pod.cockpit->instruments.end());
    EXPECT_EQ(stage->kind, Instrument::Kind::Button);
    EXPECT_EQ(stage->step, 1.0);

    // The lander's pilot faces sideways (the part's +z) with the nose overhead.
    const helios::vessel::PartDatasheet& cabin = **catalog.part("core.lander_cabin");
    ASSERT_TRUE(cabin.cockpit.has_value());
    const helios::math::Matrix3 seat = cabin.cockpit->seat_to_part();
    EXPECT_EQ(seat * (Vector3{1.0, 0.0, 0.0}), (Vector3{0.0, 0.0, 1.0}));
    EXPECT_EQ(seat * (Vector3{0.0, 0.0, 1.0}), (Vector3{1.0, 0.0, 0.0}));
    EXPECT_EQ(seat * (Vector3{0.0, 1.0, 0.0}), (Vector3{0.0, -1.0, 0.0})); // left = up × forward

    // The habitat is a cabin to move about in, with windows that stop a body; the cabins of
    // the pod and the lander are no bigger than their seats.
    const helios::vessel::PartDatasheet& habitat = **catalog.part("core.habitat");
    ASSERT_TRUE(habitat.cockpit.has_value());
    EXPECT_TRUE(habitat.cockpit->walkable);
    EXPECT_EQ(std::ranges::count(habitat.cockpit->boxes, true, &helios::vessel::CockpitBox::glass), 3);
    EXPECT_FALSE(cabin.cockpit->walkable);
    EXPECT_FALSE(pod.cockpit->walkable);
    // Loose things in it: a crate and a toolbox.
    ASSERT_EQ(habitat.cockpit->items.size(), 2U);
    EXPECT_EQ(habitat.cockpit->items[0].name, "crate");
    EXPECT_EQ(habitat.cockpit->items[0].mass_kg, 10.0);
    EXPECT_EQ(habitat.cockpit->items[1].size_m, (Vector3{0.35, 0.16, 0.16}));
    EXPECT_TRUE(pod.cockpit->items.empty());

    // A part without a seat has none.
    EXPECT_FALSE((**catalog.part("core.probe_core")).cockpit.has_value());
}

TEST(Cockpit, DirectionsAreNormalisedAndMadePerpendicular) {
    const auto cockpit = parse("forward = [2.0, 0.0, 0.0]\n"
                               "up = [1.0, 0.0, 3.0]\n"
                               "[[cockpit.instrument]]\n"
                               "kind = \"dial\"\n"
                               "signal = \"vessel/mass_kg\"\n"
                               "facing = [-4.0, 0.0, 0.0]\n"
                               "up = [0.5, 0.0, 0.5]\n"
                               "maximum = 10.0\n");
    ASSERT_TRUE(cockpit.has_value()) << helios::core::describe(cockpit.error());
    EXPECT_EQ(cockpit->forward, (Vector3{1.0, 0.0, 0.0}));
    EXPECT_EQ(cockpit->up, (Vector3{0.0, 0.0, 1.0}));
    ASSERT_EQ(cockpit->instruments.size(), 1U);
    EXPECT_EQ(cockpit->instruments[0].facing, (Vector3{-1.0, 0.0, 0.0}));
    EXPECT_EQ(cockpit->instruments[0].up, (Vector3{0.0, 0.0, 1.0}));
}

TEST(Cockpit, LooseItemsNeedACabinAndAMass) {
    const auto with_item = [](const std::string& head, const std::string& body) {
        return parse(head + "[[cockpit.item]]\n" + body);
    };
    const std::string block = "name = \"block\"\nsize_m = [0.2, 0.2, 0.2]\n";
    const auto cabin =
        with_item("walkable = true\n", block + "position_m = [0.5, 0.0, 0.0]\nmass_kg = 8.0\n");
    ASSERT_TRUE(cabin.has_value()) << helios::core::describe(cabin.error());
    ASSERT_EQ(cabin->items.size(), 1U);
    EXPECT_EQ(cabin->items[0].mass_kg, 8.0);
    EXPECT_EQ(cabin->items[0].position_m, (Vector3{0.5, 0.0, 0.0}));
    // Not in a cockpit nobody can move about in.
    EXPECT_EQ(with_item("", block).error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(with_item("walkable = true\n", block + "mass_kg = 0.0\n").error().code,
              ErrorCode::ParseFailure);
    EXPECT_EQ(with_item("walkable = true\n", "name = \"flat\"\nsize_m = [0.2, 0.0, 0.2]\n").error().code,
              ErrorCode::ParseFailure);
    EXPECT_EQ(with_item("walkable = true\n", "size_m = [0.2, 0.2, 0.2]\n").error().code,
              ErrorCode::ParseFailure); // no name
    // Two of a name could not be told apart in a prompt.
    EXPECT_EQ(with_item("walkable = true\n", block + "[[cockpit.item]]\n" + block).error().code,
              ErrorCode::ParseFailure);
}

TEST(Cockpit, RejectsWhatItCannotDraw) {
    // No direction left for `up`.
    EXPECT_EQ(parse("forward = [1.0, 0.0, 0.0]\nup = [3.0, 0.0, 0.0]\n").error().code,
              ErrorCode::ParseFailure);
    // A box with no thickness.
    EXPECT_EQ(parse("boxes = [{ centre_m = [0.0, 0.0, 0.0], size_m = [1.0, 0.0, 1.0] }]\n").error().code,
              ErrorCode::ParseFailure);
    const auto with_instrument = [](const std::string& body) {
        return parse("[[cockpit.instrument]]\n" + body);
    };
    EXPECT_EQ(with_instrument("kind = \"wheel\"\nsignal = \"a/b\"\n").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(with_instrument("kind = \"lever\"\n").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(with_instrument("kind = \"button\"\nlabel = \"GO\"\n").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(
        with_instrument("kind = \"dial\"\nsignal = \"a/b\"\nminimum = 5.0\nmaximum = 5.0\n").error().code,
        ErrorCode::ParseFailure);
    // A misspelt key is an error, not a default.
    EXPECT_EQ(with_instrument("kind = \"lever\"\nsignal = \"a/b\"\nsize = 0.2\n").error().code,
              ErrorCode::ParseFailure);
    // A button may step a signal, or publish commands, or both.
    EXPECT_TRUE(with_instrument("kind = \"button\"\nsignal = \"staging/stage\"\nstep = 1.0\n").has_value());
    EXPECT_TRUE(with_instrument("kind = \"button\"\ncommands = [{ signal = \"guidance/hold\", value = 1 }]\n")
                    .has_value());
}

} // namespace
