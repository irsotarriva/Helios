#include "helios/core/toml.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <string_view>

#include "key_bindings.hpp"

namespace {

using helios::app::Action;
using helios::app::action_from_name;
using helios::app::action_name;
using helios::app::bound_control;
using helios::app::ControlKey;
using helios::app::k_action_count;
using helios::app::key_code;
using helios::app::key_labels;
using helios::app::key_name;
using helios::app::KeyBindings;
using helios::app::KeyCode;
using helios::app::parse_key_bindings;

[[nodiscard]] helios::core::Result<KeyBindings> parse(std::string_view text) {
    auto document = helios::core::parse_toml(text);
    if (!document) {
        return std::unexpected(document.error());
    }
    return parse_key_bindings(*document);
}

TEST(KeyBindings, KeysAreNamedByPosition) {
    // SDL's scancodes, which are the USB HID usages.
    EXPECT_EQ(key_code("A"), KeyCode{4});
    EXPECT_EQ(key_code("left shift"), KeyCode{225});
    EXPECT_EQ(key_code("KEYPAD 7"), KeyCode{95});
    EXPECT_EQ(key_code("Up"), KeyCode{82});
    EXPECT_FALSE(key_code("Shift").has_value());
    EXPECT_EQ(key_name(44), "Space");
    EXPECT_EQ(key_name(0), "");
}

TEST(KeyBindings, EveryActionHasAName) {
    for (std::size_t index = 0; index < k_action_count; ++index) {
        const auto action = static_cast<Action>(index);
        const std::string_view name = action_name(action);
        ASSERT_FALSE(name.empty()) << index;
        EXPECT_EQ(action_from_name(name), action) << name;
    }
    EXPECT_FALSE(action_from_name("fly").has_value());
}

TEST(KeyBindings, TheStockFileBindsEveryAction) {
    const auto bindings =
        helios::app::load_key_bindings(std::filesystem::path{HELIOS_DATA_DIR} / "input" / "bindings.toml");
    ASSERT_TRUE(bindings.has_value()) << helios::core::describe(bindings.error());
    for (std::size_t index = 0; index < k_action_count; ++index) {
        EXPECT_FALSE(bindings->keys_of(static_cast<Action>(index)).empty())
            << action_name(static_cast<Action>(index));
    }
    // The space bar both jumps and pauses: on foot it is the jump's.
    EXPECT_TRUE(bindings->binds(*key_code("Space"), Action::Jump));
    EXPECT_TRUE(bindings->binds(*key_code("Space"), Action::Pause));
    const auto labels = key_labels(*bindings);
    EXPECT_EQ(labels.interact, "F");
    EXPECT_EQ(labels.move, "arrows");
    EXPECT_EQ(labels.turn, "W S A D Q E");
    EXPECT_EQ(labels.up_down, "Right Shift / Right Ctrl");
    const std::string help = helios::app::help_text(*bindings);
    EXPECT_NE(help.find("F: whatever the prompt"), std::string::npos) << help;
    EXPECT_NE(help.find("I: ign"), std::string::npos) << help;
}

TEST(KeyBindings, ActionsTakeOneKeyOrSeveral) {
    const auto bindings = parse(R"([actions]
interact = "G"
pause = ["P", "Keypad 0"]
forward = "I"
back = "K"
left = "J"
right = "L"
)");
    ASSERT_TRUE(bindings.has_value()) << helios::core::describe(bindings.error());
    EXPECT_EQ(bindings->keys_of(Action::Interact), std::vector<KeyCode>{*key_code("G")});
    EXPECT_EQ(bindings->keys_of(Action::Pause).size(), 2U);
    EXPECT_TRUE(bindings->keys_of(Action::Quit).empty()); // left out: no key
    const auto labels = key_labels(*bindings);
    EXPECT_EQ(labels.interact, "G");
    EXPECT_EQ(labels.move, "I K J L");
    EXPECT_EQ(labels.jump, "(unbound)");
}

TEST(KeyBindings, MistakesAreRefused) {
    EXPECT_FALSE(parse("[actions]\nfly = \"F\"\n").has_value());            // no such action
    EXPECT_FALSE(parse("[actions]\ninteract = \"Shift\"\n").has_value());   // no such key
    EXPECT_FALSE(parse("[actions]\npause = [\"P\", \"p\"]\n").has_value()); // the same key twice
    EXPECT_FALSE(parse("[actions]\npause = 3\n").has_value());              // not a key
    EXPECT_FALSE(parse("[keys]\n").has_value());                            // no such table
    EXPECT_FALSE(parse("[[control]]\nkey = \"G\"\n").has_value());          // no control
    EXPECT_FALSE(parse("[[control]]\nkey = \"G\"\ncontrol = \"\"\n").has_value());
    EXPECT_FALSE(parse("[[control]]\nkey = \"G\"\ncontrol = \"gear\"\nway = 2\n").has_value());
    EXPECT_FALSE(parse("[[control]]\nkey = \"G\"\ncontrol = \"gear\"\nrate = 0\n").has_value());
    EXPECT_FALSE(parse("[[control]]\nkey = \"G\"\ncontrol = \"gear\"\nwhy = 1\n").has_value());
    // A key is an action's or a control's, not both.
    EXPECT_FALSE(
        parse("[actions]\ninteract = \"G\"\n[[control]]\nkey = \"G\"\ncontrol = \"gear\"\n").has_value());
}

TEST(KeyBindings, AControlKeyFindsItsControlInAnyCockpitThatHasIt) {
    const auto bindings = parse(R"([[control]]
key = "G"
control = "gear"
way = -1

[[control]]
key = "H"
control = "gear"
part = "core.lander_cabin"
rate = 0.25
)");
    ASSERT_TRUE(bindings.has_value()) << helios::core::describe(bindings.error());
    ASSERT_EQ(bindings->controls.size(), 2U);
    const ControlKey& any = bindings->controls[0];
    const ControlKey& lander = bindings->controls[1];
    EXPECT_EQ(any.way, -1);
    EXPECT_DOUBLE_EQ(lander.rate_per_s, 0.25);

    helios::vessel::Cockpit cockpit;
    cockpit.controls.resize(2);
    cockpit.controls[0].id = "hold";
    cockpit.controls[1].id = "gear";
    EXPECT_EQ(bound_control(cockpit, "core.command_pod", any), std::size_t{1});
    EXPECT_FALSE(bound_control(cockpit, "core.command_pod", lander).has_value());
    EXPECT_EQ(bound_control(cockpit, "core.lander_cabin", lander), std::size_t{1});
    cockpit.controls[1].id = "lights";
    EXPECT_FALSE(bound_control(cockpit, "core.command_pod", any).has_value());
}

} // namespace
