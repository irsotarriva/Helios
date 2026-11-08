#include "helios/render/flight_geometry.hpp"
#include "helios/render/model.hpp"
#include "helios/vessel/controls.hpp"
#include "helios/vessel/part_datasheet.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using helios::math::Vector3;
using helios::render::Model;
using helios::render::ModelLibrary;
using helios::render::ModelPaint;
using helios::render::ModelPiece;
using helios::vessel::Control;

const std::filesystem::path k_models = std::filesystem::path{HELIOS_DATA_DIR} / "models";
constexpr std::string_view k_toggle = "stock/toggle.glb";

[[nodiscard]] ModelLibrary stock_models() {
    auto library = ModelLibrary::load(k_models);
    EXPECT_TRUE(library.has_value()) << (library ? "" : helios::core::describe(library.error()));
    return library ? std::move(*library) : ModelLibrary{};
}

[[nodiscard]] Vector3 apply(const helios::render::Matrix4f& matrix, const Vector3& point) {
    const auto at = [&](std::size_t index) { return static_cast<double>(matrix.at(index)); };
    return {(at(0) * point.x) + (at(4) * point.y) + (at(8) * point.z) + at(12),
            (at(1) * point.x) + (at(5) * point.y) + (at(9) * point.z) + at(13),
            (at(2) * point.x) + (at(6) * point.y) + (at(10) * point.z) + at(14)};
}

TEST(Models, TheStockToggleHasAPlateAndAHandleThatMoves) {
    const ModelLibrary library = stock_models();
    const Model* toggle = library.find(k_toggle);
    ASSERT_NE(toggle, nullptr);
    EXPECT_EQ(library.find("stock/none.glb"), nullptr);
    const auto has = [&](ModelPaint paint, bool moving) {
        return std::ranges::any_of(toggle->pieces, [&](const ModelPiece& piece) {
            return piece.paint == paint && piece.moving == moving;
        });
    };
    EXPECT_TRUE(has(ModelPaint::Colour, false)); // the plate
    EXPECT_TRUE(has(ModelPaint::Own, false));    // the nut
    EXPECT_TRUE(has(ModelPaint::Accent, true));  // the handle
    EXPECT_NEAR(toggle->pivot_m.z, 0.007, 1e-6);
    // The nut's own colour, as displayed, is the grey it was made in.
    for (const ModelPiece& piece : toggle->pieces) {
        if (piece.paint == ModelPaint::Own) {
            EXPECT_NEAR(piece.colour.x, 0.62, 0.01);
        }
    }
    // The handle stands out of the panel: its mesh is placed above the plate.
    for (const ModelPiece& piece : toggle->pieces) {
        ASSERT_LT(piece.mesh, library.meshes().size());
        const helios::render::ModelMesh& mesh = library.meshes()[piece.mesh];
        EXPECT_EQ(mesh.indices.size() % 3, 0U);
        if (piece.moving) {
            for (const helios::render::MeshVertex& vertex : mesh.vertices) {
                EXPECT_GE(vertex.position.z, 0.007F - 1e-6F);
            }
        }
    }
    const Model* button = library.find("stock/pushbutton.glb");
    ASSERT_NE(button, nullptr);
    EXPECT_TRUE(std::ranges::any_of(button->pieces,
                                    [](const ModelPiece& piece) { return piece.paint == ModelPaint::Lamp; }));
}

// The licences (D44): no model is shipped without one.
TEST(Models, EveryModelInTheDataIsListedWithItsLicence) {
    const ModelLibrary library = stock_models();
    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(k_models)) {
        const std::filesystem::path extension = entry.path().extension();
        if (extension == ".glb" || extension == ".gltf") {
            files.push_back(entry.path().lexically_relative(k_models).generic_string());
        }
    }
    EXPECT_FALSE(files.empty());
    for (const std::string& file : files) {
        EXPECT_NE(library.find(file), nullptr) << file << " is not in LICENCES.toml";
    }
    for (const helios::render::ModelLicence& licence : library.licences()) {
        EXPECT_FALSE(licence.licence.empty());
        EXPECT_FALSE(licence.author.empty());
    }
}

TEST(Models, EveryModelTheStockLooksNameIsThere) {
    const ModelLibrary library = stock_models();
    helios::vessel::PartCatalog catalog;
    ASSERT_TRUE(catalog.load_style_sheets(std::filesystem::path{HELIOS_DATA_DIR} / "styles").has_value());
    ASSERT_TRUE(catalog.load_directory(std::filesystem::path{HELIOS_DATA_DIR} / "parts").has_value());
    std::size_t named = 0;
    for (const helios::vessel::StyleSheet& sheet : catalog.style_sheets()) {
        for (const helios::vessel::StyleSheet::Rule& rule : sheet.rules) {
            if (rule.look.model.has_value() && !rule.look.model->empty()) {
                ++named;
                EXPECT_NE(library.find(*rule.look.model), nullptr) << sheet.name << ": " << *rule.look.model;
            }
        }
    }
    EXPECT_GT(named, 0U);
}

class ModelDirectory : public ::testing::Test {
protected:
    void SetUp() override {
        directory_ = std::filesystem::temp_directory_path()
                     / ("helios_models_"
                        + std::string{::testing::UnitTest::GetInstance()->current_test_info()->name()});
        std::filesystem::remove_all(directory_);
        std::filesystem::create_directories(directory_ / "stock");
        std::filesystem::copy_file(k_models / k_toggle, directory_ / k_toggle);
    }
    void TearDown() override { std::filesystem::remove_all(directory_); }

    void write_licences(std::string_view text) const {
        std::ofstream out(directory_ / "LICENCES.toml");
        out << text;
    }

    [[nodiscard]] const std::filesystem::path& directory() const noexcept { return directory_; }

private:
    std::filesystem::path directory_;
};

TEST_F(ModelDirectory, AModelIsReadOnlyWithItsLicence) {
    write_licences("");
    const auto unlisted = ModelLibrary::load(directory());
    ASSERT_TRUE(unlisted.has_value());
    EXPECT_EQ(unlisted->find(k_toggle), nullptr);

    write_licences("[[model]]\nfile = \"stock/toggle.glb\"\ntitle = \"T\"\nauthor = \"A\"\nsource = \"S\"\n");
    EXPECT_FALSE(ModelLibrary::load(directory()).has_value()); // no licence

    write_licences("[[model]]\nfile = \"stock/toggle.glb\"\ntitle = \"T\"\nauthor = \"A\"\nlicence = \"\"\n"
                   "source = \"S\"\n");
    EXPECT_FALSE(ModelLibrary::load(directory()).has_value()); // an empty one

    write_licences(
        "[[model]]\nfile = \"../toggle.glb\"\ntitle = \"T\"\nauthor = \"A\"\nlicence = \"CC0-1.0\"\n"
        "source = \"S\"\n");
    EXPECT_FALSE(ModelLibrary::load(directory()).has_value()); // outside the directory

    write_licences(
        "[[model]]\nfile = \"stock/gone.glb\"\ntitle = \"T\"\nauthor = \"A\"\nlicence = \"CC0-1.0\"\n"
        "source = \"S\"\n");
    EXPECT_FALSE(ModelLibrary::load(directory()).has_value()); // listed, but not there

    write_licences(
        "[[model]]\nfile = \"stock/toggle.glb\"\ntitle = \"T\"\nauthor = \"A\"\nlicence = \"CC0-1.0\"\n"
        "source = \"S\"\n");
    const auto listed = ModelLibrary::load(directory());
    ASSERT_TRUE(listed.has_value()) << helios::core::describe(listed.error());
    EXPECT_NE(listed->find(k_toggle), nullptr);
}

TEST_F(ModelDirectory, AFileThatIsNotGltfIsRefused) {
    {
        std::ofstream out(directory() / "stock" / "broken.glb", std::ios::binary);
        out << "not a model";
    }
    std::vector<helios::render::ModelMesh> meshes;
    const auto model =
        helios::render::load_model(directory() / "stock" / "broken.glb", "stock/broken.glb", meshes);
    EXPECT_FALSE(model.has_value());
    EXPECT_TRUE(meshes.empty());
}

TEST(Models, AModelPathStaysInsideTheModels) {
    using helios::vessel::is_model_path;
    EXPECT_TRUE(is_model_path("stock/toggle.glb"));
    EXPECT_TRUE(is_model_path("apollo/panel_1.gltf"));
    EXPECT_FALSE(is_model_path("toggle.obj"));
    EXPECT_FALSE(is_model_path("../toggle.glb"));
    EXPECT_FALSE(is_model_path("stock/../../toggle.glb"));
    EXPECT_FALSE(is_model_path("/stock/toggle.glb"));
    EXPECT_FALSE(is_model_path("stock\\toggle.glb"));
    EXPECT_FALSE(is_model_path("C:/toggle.glb"));
    EXPECT_FALSE(is_model_path("stock//toggle.glb"));
}

// A rule that changes the shape drops a model made for the shape before (data/styles/README.md).
TEST(Models, AModelIsDroppedWithTheShapeItWasMadeFor) {
    helios::vessel::LookPatch patch{.shape = "toggle", .model = std::string{k_toggle}};
    patch.apply(helios::vessel::LookPatch{.shape = "toggle"});
    EXPECT_FALSE(patch.model.has_value());
    patch.apply(helios::vessel::LookPatch{.model = std::string{k_toggle}});
    patch.apply(helios::vessel::LookPatch{.accent = Vector3{1.0, 0.0, 0.0}});
    EXPECT_EQ(patch.model, std::string{k_toggle});
}

// A switch drawn with the stock toggle: the plate stays, the handle leans to its position.
[[nodiscard]] helios::sim::SceneSnapshot snapshot_with_toggle(double value, bool modelled) {
    helios::vessel::PartDatasheet pod;
    pod.id = "test.pod";
    helios::vessel::Cockpit cockpit;
    cockpit.eye_m = {0.0, 0.0, 0.0};
    Control toggle{.id = "pump",
                   .type = Control::Type::Switch,
                   .label = "PUMP",
                   .place = {.position_m = {0.5, 0.0, 0.0}},
                   .signal = "pump",
                   .positions = {{.settings = {{.signal = "pump", .value = 0.0}}},
                                 {.settings = {{.signal = "pump", .value = 1.0}}}}};
    if (modelled) {
        toggle.own_look.model = std::string{k_toggle};
    }
    cockpit.controls.push_back(std::move(toggle));
    EXPECT_TRUE(helios::vessel::resolve_looks(cockpit, {}).has_value());
    pod.cockpit = std::move(cockpit);
    helios::sim::SceneSnapshot snapshot;
    helios::sim::VesselView vessel;
    vessel.id = {0};
    vessel.name = "Test";
    vessel.domain = {7}; // no body: no ground
    vessel.signals.push_back(
        {.name = "pump", .command = true, .value = value, .minimum = 0.0, .maximum = 1.0});
    vessel.parts.resize(1);
    vessel.parts[0].name = "pod";
    vessel.parts[0].datasheet = std::make_shared<const helios::vessel::PartDatasheet>(std::move(pod));
    snapshot.vessels.push_back(std::move(vessel));
    return snapshot;
}

TEST(Models, AControlDrawnWithAModelMovesItsMovingPart) {
    const ModelLibrary library = stock_models();
    const Model* toggle = library.find(k_toggle);
    ASSERT_NE(toggle, nullptr);
    const auto geometry_at = [&](double value, bool modelled, const ModelLibrary* models) {
        const helios::sim::SceneSnapshot snapshot = snapshot_with_toggle(value, modelled);
        const auto seat =
            helios::render::seat_view_point(snapshot.vessels.front(), helios::render::HeadPose{}).value();
        return helios::render::build_flight_geometry(snapshot, 0, seat, true, models);
    };
    const helios::render::FlightGeometry built_in = geometry_at(0.0, false, &library);
    const helios::render::FlightGeometry off = geometry_at(0.0, true, &library);
    const helios::render::FlightGeometry on = geometry_at(1.0, true, &library);
    EXPECT_TRUE(built_in.models.empty());
    ASSERT_EQ(off.models.size(), toggle->pieces.size());
    // The model stands in for the shape's solids; the handle and the label stay.
    EXPECT_LT(off.solids.size(), built_in.solids.size());
    ASSERT_EQ(off.controls.size(), 1U);
    EXPECT_FALSE(off.text.empty());
    // Without the library, or with a model it lacks, the built-in shape.
    EXPECT_EQ(geometry_at(0.0, true, nullptr).solids.size(), built_in.solids.size());

    for (std::size_t index = 0; index < toggle->pieces.size(); ++index) {
        const Vector3 tip_m = toggle->pivot_m + Vector3{0.0, 0.0, 0.02};
        const Vector3 off_m = apply(off.models[index].model, tip_m);
        const Vector3 on_m = apply(on.models[index].model, tip_m);
        if (toggle->pieces[index].moving) {
            // Thrown to its upper position, the handle leans up the panel (the seat's up).
            EXPECT_GT(on_m.z - off_m.z, 0.01);
        } else {
            EXPECT_NEAR(norm(on_m - off_m), 0.0, 1e-6);
        }
    }
}

} // namespace
