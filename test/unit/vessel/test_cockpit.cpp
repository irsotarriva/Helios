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
using helios::vessel::Control;
using helios::vessel::Indicator;
using helios::vessel::PartCatalog;

[[nodiscard]] helios::core::Result<Cockpit> parse(const std::string& cockpit_toml) {
    const auto document = helios::core::parse_toml("[cockpit]\n" + cockpit_toml);
    EXPECT_TRUE(document.has_value());
    return document->at("cockpit").and_then(helios::vessel::parse_cockpit);
}

TEST(Cockpit, TheStockCrewedPartsHaveASeatControlsAndIndicators) {
    const PartCatalog catalog = helios::test::load_stock_parts();
    const helios::vessel::PartDatasheet& pod = **catalog.part("core.command_pod");
    ASSERT_TRUE(pod.cockpit.has_value());
    EXPECT_FALSE(pod.cockpit->boxes.empty());
    EXPECT_EQ(pod.cockpit->style, "stock");
    const auto throttle = std::ranges::find(pod.cockpit->controls, "engine/throttle", &Control::signal);
    ASSERT_NE(throttle, pod.cockpit->controls.end());
    EXPECT_EQ(throttle->type, Control::Type::Axis);
    EXPECT_EQ(throttle->look.shape, "lever") << "from the stock style";
    // On the console: it faces up, and is pushed forward for more.
    EXPECT_EQ(throttle->place.facing, (Vector3{0.0, 0.0, 1.0}));
    EXPECT_EQ(throttle->place.up, (Vector3{1.0, 0.0, 0.0}));
    const auto stage = std::ranges::find(pod.cockpit->controls, "stage", &Control::id);
    ASSERT_NE(stage, pod.cockpit->controls.end());
    EXPECT_EQ(stage->type, Control::Type::Momentary);
    EXPECT_EQ(stage->step, 1.0);
    ASSERT_TRUE(stage->guard.has_value()) << "under a cover";
    EXPECT_EQ(stage->guard->look.shape, "cover");
    EXPECT_EQ(stage->look.size_m, 0.06) << "its own look over the style";
    EXPECT_EQ(stage->look.colour, (Vector3{0.62, 0.12, 0.10})) << "the stock style's red class";
    const auto ignition = std::ranges::find(pod.cockpit->controls, "ign", &Control::id);
    ASSERT_NE(ignition, pod.cockpit->controls.end());
    EXPECT_EQ(ignition->positions.size(), 2U);
    EXPECT_EQ(ignition->guard->closes_to, 0U);
    const auto lamp = std::ranges::find(pod.cockpit->indicators, "engine_lamp", &Indicator::id);
    ASSERT_NE(lamp, pod.cockpit->indicators.end());
    EXPECT_EQ(lamp->type, Indicator::Type::State);
    EXPECT_EQ(lamp->look.shape, "lamp");

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
                               "[[cockpit.indicator]]\n"
                               "id = \"mass\"\n"
                               "type = \"gauge\"\n"
                               "signal = \"vessel/mass_kg\"\n"
                               "facing = [-4.0, 0.0, 0.0]\n"
                               "up = [0.5, 0.0, 0.5]\n"
                               "maximum = 10.0\n");
    ASSERT_TRUE(cockpit.has_value()) << helios::core::describe(cockpit.error());
    EXPECT_EQ(cockpit->forward, (Vector3{1.0, 0.0, 0.0}));
    EXPECT_EQ(cockpit->up, (Vector3{0.0, 0.0, 1.0}));
    ASSERT_EQ(cockpit->indicators.size(), 1U);
    EXPECT_EQ(cockpit->indicators[0].place.facing, (Vector3{-1.0, 0.0, 0.0}));
    EXPECT_EQ(cockpit->indicators[0].place.up, (Vector3{0.0, 0.0, 1.0}));
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
    const auto control = [](const std::string& body) {
        return parse("[[cockpit.control]]\nid = \"c\"\n" + body);
    };
    const auto fails = [](const helios::core::Result<Cockpit>& parsed) {
        return !parsed.has_value() && parsed.error().code == ErrorCode::ParseFailure;
    };
    EXPECT_TRUE(fails(control("type = \"wheel\"\nsignal = \"a/b\"\n"))) << "not a type";
    EXPECT_TRUE(fails(control("type = \"axis\"\n"))) << "no signal";
    EXPECT_TRUE(fails(control("type = \"momentary\"\nlabel = \"GO\"\n"))) << "does nothing";
    EXPECT_TRUE(fails(control("type = \"axis\"\nsignal = \"a/b\"\nsize = 0.2\n"))) << "a misspelt key";
    // A momentary control may step a signal, or set some, or both.
    EXPECT_TRUE(control("type = \"momentary\"\nsignal = \"staging/stage\"\nstep = 1.0\n").has_value());
    EXPECT_TRUE(control("type = \"momentary\"\ncommands = [{ signal = \"guidance/hold\", value = 1 }]\n")
                    .has_value());
    // A switch: two positions or more, its rest one that is not sprung, its guard closing to one.
    const std::string a_switch = "type = \"switch\"\nsignal = \"a/b\"\n";
    EXPECT_TRUE(fails(control(a_switch + "positions = [1]\n")));
    EXPECT_TRUE(fails(control(a_switch + "positions = [0, { value = 1, spring = true }]\nrest = 1\n")));
    EXPECT_TRUE(fails(control(a_switch + "positions = [0, 1]\nguard = { closes_to = 2 }\n")));
    const auto sprung = control(a_switch
                                + "positions = [0, 1, { value = 2, spring = true, commands = [{ signal = "
                                  "\"c/d\", value = 1 }] }]\nrest = 1\nlocked = true\n");
    ASSERT_TRUE(sprung.has_value()) << helios::core::describe(sprung.error());
    const Control& three = sprung->controls[0];
    ASSERT_EQ(three.positions.size(), 3U);
    EXPECT_TRUE(three.positions[2].spring);
    ASSERT_EQ(three.positions[2].settings.size(), 2U) << "its value and its commands";
    EXPECT_EQ(three.positions[2].settings[0].value, 2.0);
    EXPECT_TRUE(three.locked);
    // What belongs to one type is refused on another.
    EXPECT_TRUE(fails(control("type = \"axis\"\nsignal = \"a/b\"\nlocked = true\n")));
    EXPECT_TRUE(fails(control("type = \"stick\"\nsignals = [\"a/b\"]\n"))) << "two signals or three";
    EXPECT_TRUE(fails(control("type = \"encoder\"\nsignal = \"a/b\"\n"))) << "no step";
    // Indicators: a gauge needs a range; thresholds are a state's, in order.
    const auto indicator = [](const std::string& body) {
        return parse("[[cockpit.indicator]]\nid = \"i\"\nsignal = \"a/b\"\n" + body);
    };
    EXPECT_TRUE(fails(indicator("type = \"gauge\"\nminimum = 5.0\nmaximum = 5.0\n")));
    EXPECT_TRUE(fails(indicator("type = \"state\"\nthresholds = [2.0, 1.0]\n")));
    EXPECT_TRUE(fails(indicator("type = \"gauge\"\nthresholds = [1.0]\n")));
    EXPECT_EQ(indicator("type = \"state\"\n").value().indicators[0].thresholds, std::vector<double>{0.5})
        << "a lamp, lit from one half";
    // Ids are shared by controls and indicators.
    EXPECT_TRUE(fails(parse("[[cockpit.control]]\nid = \"i\"\ntype = \"axis\"\nsignal = \"a/b\"\n"
                            "[[cockpit.indicator]]\nid = \"i\"\ntype = \"gauge\"\nsignal = \"a/b\"\n")));
}

TEST(Cockpit, ACabinMayHaveAnAirlock) {
    const auto cabin = parse("walkable = true\nairlock = { inside_m = [0, 1, 0], outside_m = [0, 3, 0] }\n");
    ASSERT_TRUE(cabin.has_value()) << helios::core::describe(cabin.error());
    ASSERT_TRUE(cabin->airlock.has_value());
    EXPECT_EQ(cabin->airlock->outside_m.y, 3.0);
    EXPECT_EQ(cabin->airlock->cycle_s, 20.0);
    EXPECT_FALSE(parse("walkable = true\n").value().airlock.has_value());
    // Not without a cabin, and it has to lead somewhere.
    EXPECT_FALSE(parse("airlock = { inside_m = [0, 1, 0], outside_m = [0, 3, 0] }\n").has_value());
    EXPECT_FALSE(
        parse("walkable = true\nairlock = { inside_m = [0, 1, 0], outside_m = [0, 1, 0] }\n").has_value());
    EXPECT_FALSE(
        parse("walkable = true\nairlock = { inside_m = [0, 1, 0], outside_m = [0, 3, 0], cycle_s = -1 }\n")
            .has_value());
}

// Looks come from style sheets as in CSS (BRIEFING D43): the element's rule, then its classes,
// then the control's own look; a sheet may extend another.
TEST(Cockpit, LooksCascadeFromTheStyleSheets) {
    const auto sheet = [](std::string name, const std::string& text) {
        return helios::vessel::parse_style_sheet(std::move(name), helios::core::parse_toml(text).value())
            .value();
    };
    const std::vector<helios::vessel::StyleSheet> sheets{
        sheet("base", "[switch]\nshape = \"rocker\"\nsize_m = 0.05\n[switch.red]\naccent = [1, 0, 0]\n"),
        sheet("replica", "[style]\nextends = \"base\"\n[switch]\nsize_m = 0.03\n"
                         "[switch.big]\nsize_m = 0.08\n[guard]\ncolour = [0, 0, 1]\n"),
        sheet("loop", "[style]\nextends = \"loop\"\n"),
    };
    auto cockpit =
        parse("style = \"replica\"\n"
              "[[cockpit.control]]\nid = \"a\"\ntype = \"switch\"\nsignal = \"a/b\"\npositions = [0, 1]\n"
              "class = \"big red\"\nguard = true\n"
              "[[cockpit.control]]\nid = \"b\"\ntype = \"switch\"\nsignal = \"a/b\"\npositions = [0, 1]\n"
              "look = { shape = \"rotary\" }\n"
              "[[cockpit.indicator]]\nid = \"c\"\ntype = \"state\"\nsignal = \"a/b\"\n");
    ASSERT_TRUE(cockpit.has_value()) << helios::core::describe(cockpit.error());
    ASSERT_TRUE(helios::vessel::resolve_looks(*cockpit, sheets).has_value());
    const helios::vessel::Look& a = cockpit->controls[0].look;
    EXPECT_EQ(a.shape, "rocker") << "from the sheet it extends";
    EXPECT_EQ(a.size_m, 0.08) << "a class is above the element's rule, in any sheet";
    EXPECT_EQ(a.accent, (Vector3{1.0, 0.0, 0.0}));
    EXPECT_EQ(cockpit->controls[0].guard->look.colour, (Vector3{0.0, 0.0, 1.0}));
    const helios::vessel::Look& b = cockpit->controls[1].look;
    EXPECT_EQ(b.shape, "rotary") << "its own look above all";
    EXPECT_EQ(b.size_m, 0.03);
    EXPECT_EQ(b.travel, 1.57) << "the rotary's own measure, which no rule gives";
    EXPECT_EQ(cockpit->indicators[0].look.shape, "lamp") << "built in";

    cockpit->style = "nowhere";
    EXPECT_FALSE(helios::vessel::resolve_looks(*cockpit, sheets).has_value());
    cockpit->style = "loop";
    EXPECT_FALSE(helios::vessel::resolve_looks(*cockpit, sheets).has_value());
    cockpit->style.clear();
    cockpit->controls[1].own_look.shape = "lever";
    EXPECT_FALSE(helios::vessel::resolve_looks(*cockpit, sheets).has_value()) << "not a switch's shape";
    // A sheet with an unknown element, or a model outside data/models, is refused.
    EXPECT_FALSE(helios::vessel::parse_style_sheet("x", helios::core::parse_toml("[dial]\n").value()));
    EXPECT_FALSE(helios::vessel::parse_style_sheet(
        "x", helios::core::parse_toml("[switch]\nmodel = \"../a.glb\"\n").value()));
    const auto modelled = helios::vessel::parse_style_sheet(
        "x", helios::core::parse_toml("[switch]\nmodel = \"a/b.glb\"\n").value());
    ASSERT_TRUE(modelled.has_value());
    EXPECT_EQ(modelled->rules.front().look.model, "a/b.glb");
}

} // namespace
