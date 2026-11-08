#include "key_bindings.hpp"

#include "helios/core/parse.hpp"

#include <algorithm>
#include <array>
#include <format>
#include <initializer_list>
#include <iterator>
#include <span>
#include <utility>

namespace helios::app {

namespace {

using core::ErrorCode;

struct NamedKey {
    std::string_view name;
    KeyCode key;
};

// USB HID usages of the keyboard page (SDL's scancodes), with SDL's names for them.
constexpr std::array k_keys{
    NamedKey{"A", 4},
    NamedKey{"B", 5},
    NamedKey{"C", 6},
    NamedKey{"D", 7},
    NamedKey{"E", 8},
    NamedKey{"F", 9},
    NamedKey{"G", 10},
    NamedKey{"H", 11},
    NamedKey{"I", 12},
    NamedKey{"J", 13},
    NamedKey{"K", 14},
    NamedKey{"L", 15},
    NamedKey{"M", 16},
    NamedKey{"N", 17},
    NamedKey{"O", 18},
    NamedKey{"P", 19},
    NamedKey{"Q", 20},
    NamedKey{"R", 21},
    NamedKey{"S", 22},
    NamedKey{"T", 23},
    NamedKey{"U", 24},
    NamedKey{"V", 25},
    NamedKey{"W", 26},
    NamedKey{"X", 27},
    NamedKey{"Y", 28},
    NamedKey{"Z", 29},
    NamedKey{"1", 30},
    NamedKey{"2", 31},
    NamedKey{"3", 32},
    NamedKey{"4", 33},
    NamedKey{"5", 34},
    NamedKey{"6", 35},
    NamedKey{"7", 36},
    NamedKey{"8", 37},
    NamedKey{"9", 38},
    NamedKey{"0", 39},
    NamedKey{"Return", 40},
    NamedKey{"Escape", 41},
    NamedKey{"Backspace", 42},
    NamedKey{"Tab", 43},
    NamedKey{"Space", 44},
    NamedKey{"-", 45},
    NamedKey{"=", 46},
    NamedKey{"[", 47},
    NamedKey{"]", 48},
    NamedKey{"\\", 49},
    NamedKey{";", 51},
    NamedKey{"'", 52},
    NamedKey{"`", 53},
    NamedKey{",", 54},
    NamedKey{".", 55},
    NamedKey{"/", 56},
    NamedKey{"CapsLock", 57},
    NamedKey{"F1", 58},
    NamedKey{"F2", 59},
    NamedKey{"F3", 60},
    NamedKey{"F4", 61},
    NamedKey{"F5", 62},
    NamedKey{"F6", 63},
    NamedKey{"F7", 64},
    NamedKey{"F8", 65},
    NamedKey{"F9", 66},
    NamedKey{"F10", 67},
    NamedKey{"F11", 68},
    NamedKey{"F12", 69},
    NamedKey{"Insert", 73},
    NamedKey{"Home", 74},
    NamedKey{"PageUp", 75},
    NamedKey{"Delete", 76},
    NamedKey{"End", 77},
    NamedKey{"PageDown", 78},
    NamedKey{"Right", 79},
    NamedKey{"Left", 80},
    NamedKey{"Down", 81},
    NamedKey{"Up", 82},
    NamedKey{"Keypad /", 84},
    NamedKey{"Keypad *", 85},
    NamedKey{"Keypad -", 86},
    NamedKey{"Keypad +", 87},
    NamedKey{"Keypad Enter", 88},
    NamedKey{"Keypad 1", 89},
    NamedKey{"Keypad 2", 90},
    NamedKey{"Keypad 3", 91},
    NamedKey{"Keypad 4", 92},
    NamedKey{"Keypad 5", 93},
    NamedKey{"Keypad 6", 94},
    NamedKey{"Keypad 7", 95},
    NamedKey{"Keypad 8", 96},
    NamedKey{"Keypad 9", 97},
    NamedKey{"Keypad 0", 98},
    NamedKey{"Keypad .", 99},
    NamedKey{"Left Ctrl", 224},
    NamedKey{"Left Shift", 225},
    NamedKey{"Left Alt", 226},
    NamedKey{"Left GUI", 227},
    NamedKey{"Right Ctrl", 228},
    NamedKey{"Right Shift", 229},
    NamedKey{"Right Alt", 230},
    NamedKey{"Right GUI", 231},
};

struct NamedAction {
    std::string_view name;
    Action action;
};

constexpr std::array k_actions{
    NamedAction{"throttle_up", Action::ThrottleUp},
    NamedAction{"throttle_down", Action::ThrottleDown},
    NamedAction{"throttle_full", Action::ThrottleFull},
    NamedAction{"throttle_cut", Action::ThrottleCut},
    NamedAction{"pitch_up", Action::PitchUp},
    NamedAction{"pitch_down", Action::PitchDown},
    NamedAction{"yaw_left", Action::YawLeft},
    NamedAction{"yaw_right", Action::YawRight},
    NamedAction{"roll_left", Action::RollLeft},
    NamedAction{"roll_right", Action::RollRight},
    NamedAction{"stage", Action::Stage},
    NamedAction{"attitude_hold", Action::ToggleAttitudeHold},
    NamedAction{"forward", Action::TranslateForward},
    NamedAction{"back", Action::TranslateBack},
    NamedAction{"left", Action::TranslateLeft},
    NamedAction{"right", Action::TranslateRight},
    NamedAction{"up", Action::TranslateUp},
    NamedAction{"down", Action::TranslateDown},
    NamedAction{"interact", Action::Interact},
    NamedAction{"jump", Action::Jump},
    NamedAction{"pause", Action::Pause},
    NamedAction{"warp_faster", Action::WarpFaster},
    NamedAction{"warp_slower", Action::WarpSlower},
    NamedAction{"map_view", Action::MapView},
    NamedAction{"flight_view", Action::FlightView},
    NamedAction{"next_focus", Action::NextFocus},
    NamedAction{"help", Action::Help},
    NamedAction{"panels", Action::Panels},
    NamedAction{"quit", Action::Quit},
};
static_assert(k_actions.size() == k_action_count, "every action has a name");

[[nodiscard]] constexpr char lower(char letter) noexcept {
    return letter >= 'A' && letter <= 'Z' ? static_cast<char>(letter - 'A' + 'a') : letter;
}

[[nodiscard]] bool same_name(std::string_view a, std::string_view b) noexcept {
    return std::ranges::equal(a, b, [](char x, char y) { return lower(x) == lower(y); });
}

// One key or an array of them.
[[nodiscard]] core::Result<std::vector<KeyCode>> keys_at(const core::TomlValue& value, std::string_view key) {
    std::vector<std::string> names;
    if (value.kind == core::TomlValue::Kind::String) {
        names.push_back(value.text);
    } else if (value.kind == core::TomlValue::Kind::Array) {
        for (const core::TomlValue& item : value.items) {
            if (item.kind != core::TomlValue::Kind::String) {
                return core::fail(ErrorCode::ParseFailure,
                                  std::format("line {}: '{}' takes key names", value.line, key));
            }
            names.push_back(item.text);
        }
    } else {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: '{}' takes a key name or an array of them", value.line, key));
    }
    std::vector<KeyCode> keys;
    for (const std::string& name : names) {
        const auto code = key_code(name);
        if (!code.has_value()) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: '{}' is not a key", value.line, name));
        }
        if (std::ranges::find(keys, *code) != keys.end()) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: '{}' is given twice for '{}'", value.line, name, key));
        }
        keys.push_back(*code);
    }
    return keys;
}

[[nodiscard]] core::Result<ControlKey> parse_control_key(const core::TomlValue& table) {
    auto key_text = table.string_at("key");
    auto control = table.string_at("control");
    auto part = table.string_or("part", "");
    auto way = table.number_or("way", 0.0);
    auto rate = table.number_or("rate", 0.5);
    if (const auto error = core::first_error(table.expect_keys({"key", "control", "part", "way", "rate"}),
                                             key_text, control, part, way, rate)) {
        return std::unexpected(*error);
    }
    const auto code = key_code(*key_text);
    if (!code.has_value()) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: '{}' is not a key", table.line, *key_text));
    }
    if (control->empty()) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: a control's id is empty", table.line));
    }
    if (*way != -1.0 && *way != 0.0 && *way != 1.0) {
        return core::fail(ErrorCode::ParseFailure, std::format("line {}: 'way' is -1, 0 or 1", table.line));
    }
    if (!(*rate > 0.0)) {
        return core::fail(ErrorCode::ParseFailure,
                          std::format("line {}: 'rate' must be above 0", table.line));
    }
    return ControlKey{.control = std::move(*control),
                      .part = std::move(*part),
                      .rate_per_s = *rate,
                      .way = static_cast<int>(*way),
                      .key = *code};
}

// "A / B", or "unbound".
[[nodiscard]] std::string label_of(const KeyBindings& bindings, Action action) {
    std::string label;
    for (const KeyCode key : bindings.keys_of(action)) {
        label += label.empty() ? "" : " / ";
        label += key_name(key);
    }
    return label.empty() ? std::string{"(unbound)"} : label;
}

// Each action's first key, one after the other.
[[nodiscard]] std::string first_keys(const KeyBindings& bindings, std::initializer_list<Action> actions) {
    std::string label;
    for (const Action action : actions) {
        const std::vector<KeyCode> keys = bindings.keys_of(action);
        label += label.empty() ? "" : " ";
        label += keys.empty() ? std::string_view{"-"} : key_name(keys.front());
    }
    return label;
}

} // namespace

std::optional<KeyCode> key_code(std::string_view name) noexcept {
    const std::span keys{k_keys};
    const auto match =
        std::ranges::find_if(keys, [&](const NamedKey& key) { return same_name(key.name, name); });
    return match != keys.end() ? std::optional(match->key) : std::nullopt;
}

std::string_view key_name(KeyCode key) noexcept {
    const std::span keys{k_keys};
    const auto match = std::ranges::find(keys, key, &NamedKey::key);
    return match != keys.end() ? match->name : std::string_view{};
}

std::string_view action_name(Action action) noexcept {
    const std::span actions{k_actions};
    const auto match = std::ranges::find(actions, action, &NamedAction::action);
    return match != actions.end() ? match->name : std::string_view{};
}

std::optional<Action> action_from_name(std::string_view name) noexcept {
    const std::span actions{k_actions};
    const auto match = std::ranges::find(actions, name, &NamedAction::name);
    return match != actions.end() ? std::optional(match->action) : std::nullopt;
}

std::vector<KeyCode> KeyBindings::keys_of(Action action) const {
    std::vector<KeyCode> keys;
    for (const ActionKey& binding : actions) {
        if (binding.action == action) {
            keys.push_back(binding.key);
        }
    }
    return keys;
}

bool KeyBindings::binds(KeyCode key, Action action) const noexcept {
    return std::ranges::any_of(
        actions, [&](const ActionKey& binding) { return binding.key == key && binding.action == action; });
}

core::Result<KeyBindings> parse_key_bindings(const core::TomlValue& document) {
    if (core::VoidResult keys = document.expect_keys({"actions", "control"}); !keys) {
        return std::unexpected(keys.error());
    }
    KeyBindings bindings;
    if (document.contains("actions")) {
        const core::TomlValue& table = document.at("actions")->get();
        if (table.kind != core::TomlValue::Kind::Table) {
            return core::fail(ErrorCode::ParseFailure,
                              std::format("line {}: 'actions' is a table", table.line));
        }
        for (std::size_t index = 0; index < table.keys.size(); ++index) {
            const std::string& name = table.keys[index];
            const auto action = action_from_name(name);
            if (!action.has_value()) {
                return core::fail(ErrorCode::ParseFailure, std::format("line {}: '{}' is not an action",
                                                                       table.items[index].line, name));
            }
            auto keys = keys_at(table.items[index], name);
            if (!keys) {
                return std::unexpected(keys.error());
            }
            for (const KeyCode key : *keys) {
                bindings.actions.push_back({.key = key, .action = *action});
            }
        }
    }
    auto controls = document.array_or_empty("control");
    if (!controls) {
        return std::unexpected(controls.error());
    }
    for (const core::TomlValue& table : *controls) {
        auto control = parse_control_key(table);
        if (!control) {
            return std::unexpected(control.error());
        }
        if (std::ranges::find(bindings.actions, control->key, &ActionKey::key) != bindings.actions.end()) {
            return core::fail(ErrorCode::ParseFailure, std::format("line {}: '{}' is already an action's key",
                                                                   table.line, key_name(control->key)));
        }
        bindings.controls.push_back(std::move(*control));
    }
    return bindings;
}

core::Result<KeyBindings> load_key_bindings(const std::filesystem::path& file) {
    auto bindings =
        core::read_text_file(file)
            .and_then([](const std::string& text) { return core::parse_toml(text); })
            .and_then([](const core::TomlValue& document) { return parse_key_bindings(document); });
    if (!bindings) {
        return std::unexpected(
            core::make_error(bindings.error().code,
                             std::format("key bindings {}: {}", file.string(), bindings.error().context)));
    }
    return bindings;
}

std::optional<std::size_t> bound_control(const vessel::Cockpit& cockpit, std::string_view part_id,
                                         const ControlKey& binding) noexcept {
    if (!binding.part.empty() && binding.part != part_id) {
        return std::nullopt;
    }
    const auto match = std::ranges::find(cockpit.controls, binding.control, &vessel::Control::id);
    return match != cockpit.controls.end()
               ? std::optional(static_cast<std::size_t>(std::distance(cockpit.controls.begin(), match)))
               : std::nullopt;
}

KeyLabels key_labels(const KeyBindings& bindings) {
    const auto only = [&](Action action, std::string_view name) {
        const std::vector<KeyCode> keys = bindings.keys_of(action);
        return keys.size() == 1 && keys.front() == key_code(name);
    };
    const bool arrows = only(Action::TranslateForward, "Up") && only(Action::TranslateBack, "Down")
                        && only(Action::TranslateLeft, "Left") && only(Action::TranslateRight, "Right");
    return {.interact = label_of(bindings, Action::Interact),
            .jump = label_of(bindings, Action::Jump),
            .move = arrows ? std::string{"arrows"}
                           : first_keys(bindings, {Action::TranslateForward, Action::TranslateBack,
                                                   Action::TranslateLeft, Action::TranslateRight}),
            .forward = label_of(bindings, Action::TranslateForward),
            .back = label_of(bindings, Action::TranslateBack),
            .up_down = std::format("{} / {}", label_of(bindings, Action::TranslateUp),
                                   label_of(bindings, Action::TranslateDown)),
            .turn = first_keys(bindings, {Action::PitchDown, Action::PitchUp, Action::YawLeft,
                                          Action::YawRight, Action::RollLeft, Action::RollRight}),
            .flight_view = label_of(bindings, Action::FlightView)};
}

std::string help_text(const KeyBindings& bindings) {
    const auto key = [&](Action action) { return label_of(bindings, action); };
    const KeyLabels labels = key_labels(bindings);
    std::string text = std::format(
        "{}: map / flight view   {}: cockpit / outside\n"
        "drag: turn the view   wheel: zoom\n"
        "cockpit: click switches and buttons, drag levers\n"
        "{} / {}: throttle up / down   {}: full   {}: cut\n"
        "{}: pitch, yaw, roll (vessel, pilot)\n"
        "{}: stage   {}: attitude hold\n"
        "{}: whatever the prompt in the picture says\n"
        "  (leave the seat, sit down, the airlock, the ladder)\n"
        "{}, {}: move (vessel, pilot)\n"
        "  afoot: the mouse is the hand, {} turn\n"
        "  hold the left button on a wall or an item and drag:\n"
        "    push off, pull in, throw (let go while dragging)\n"
        "  right button: the same, but stays anchored after\n"
        "  left button on what it holds: let go of it\n"
        "  standing: {} walk   {}: jump\n"
        "{} / {}: slower / faster warp   {}: pause\n"
        "{}: next focus   {}: this help   {}: panels\n"
        "{}: quit",
        key(Action::MapView), key(Action::FlightView), key(Action::ThrottleUp), key(Action::ThrottleDown),
        key(Action::ThrottleFull), key(Action::ThrottleCut), labels.turn, key(Action::Stage),
        key(Action::ToggleAttitudeHold), key(Action::Interact), labels.move, labels.up_down, labels.turn,
        labels.move, labels.jump, key(Action::WarpSlower), key(Action::WarpFaster), key(Action::Pause),
        key(Action::NextFocus), key(Action::Help), key(Action::Panels), key(Action::Quit));
    for (const ControlKey& control : bindings.controls) {
        std::string_view way;
        if (control.way != 0) {
            way = control.way > 0 ? " up" : " down";
        }
        text += std::format("\n{}: {}{}{}", key_name(control.key), control.control, way,
                            control.part.empty() ? std::string{} : std::format(" ({})", control.part));
    }
    return text;
}

} // namespace helios::app
