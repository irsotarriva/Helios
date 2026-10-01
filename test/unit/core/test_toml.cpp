#include "helios/core/toml.hpp"

#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

using helios::core::ErrorCode;
using helios::core::parse_toml;

TEST(Toml, ScalarsStringsAndComments) {
    const auto document = parse_toml("# a comment\n"
                                     "name = \"Kestrel \\\"A\\\"\"  # trailing comment\n"
                                     "literal = 'C:\\path'\n"
                                     "mass_kg = 1_250.5\n"
                                     "count = +3\n"
                                     "rate = -2.5e-3\n"
                                     "enabled = true\r\n"
                                     "\"quoted key\" = false\n");
    ASSERT_TRUE(document.has_value()) << helios::core::describe(document.error());
    EXPECT_EQ(document->string_at("name").value(), "Kestrel \"A\"");
    EXPECT_EQ(document->string_at("literal").value(), "C:\\path");
    EXPECT_EQ(document->number_at("mass_kg").value(), 1250.5);
    EXPECT_EQ(document->number_at("count").value(), 3.0);
    EXPECT_EQ(document->number_at("rate").value(), -2.5e-3);
    EXPECT_TRUE(document->boolean_or("enabled", false).value());
    EXPECT_FALSE(document->boolean_or("quoted key", true).value());
    EXPECT_EQ(document->number_or("absent", 7.0).value(), 7.0);
    EXPECT_EQ(document->string_or("absent", "fallback").value(), "fallback");
}

TEST(Toml, TablesArraysOfTablesAndInlineTables) {
    const auto document =
        parse_toml("[[part]]\n"
                   "id = \"tank\"\n"
                   "position_m = [\n"
                   "  1.0, 2.0,  # spans lines\n"
                   "  3.0,\n"
                   "]\n"
                   "stores = [{ resource = \"lox\", capacity = 10 }, { resource = \"rp1\" }]\n"
                   "\n"
                   "[part.shape]\n"
                   "kind = \"cylinder\"\n"
                   "\n"
                   "[[part.process]]\n"
                   "name = \"burn\"\n"
                   "\n"
                   "[[part]]\n"
                   "id = \"engine\"\n"
                   "tags = [\"a\", \"b\"]\n"
                   "\n"
                   "[settings.display]\n"
                   "units = \"si\"\n");
    ASSERT_TRUE(document.has_value()) << helios::core::describe(document.error());
    const auto parts = document->array_or_empty("part").value();
    ASSERT_EQ(parts.size(), 2U);
    EXPECT_EQ(parts[0].string_at("id").value(), "tank");
    EXPECT_EQ(parts[0].numbers_or_empty("position_m").value(), (std::vector<double>{1.0, 2.0, 3.0}));
    const auto stores = parts[0].array_or_empty("stores").value();
    ASSERT_EQ(stores.size(), 2U);
    EXPECT_EQ(stores[0].number_at("capacity").value(), 10.0);
    EXPECT_EQ(stores[1].string_at("resource").value(), "rp1");
    EXPECT_EQ(parts[0].at("shape").value().get().string_at("kind").value(), "cylinder");
    EXPECT_EQ(parts[0].array_or_empty("process").value().size(), 1U);
    EXPECT_EQ(parts[1].strings_or_empty("tags").value(), (std::vector<std::string>{"a", "b"}));
    EXPECT_TRUE(parts[1].array_or_empty("process").value().empty());
    EXPECT_EQ(document->at("settings").value().get().at("display").value().get().string_at("units").value(),
              "si");
    EXPECT_EQ(parts[1].line, 15U);
}

TEST(Toml, ErrorsNameTheLine) {
    const auto unterminated = parse_toml("a = 1\nb = \"open\n");
    ASSERT_FALSE(unterminated.has_value());
    EXPECT_EQ(unterminated.error().code, ErrorCode::ParseFailure);
    EXPECT_NE(unterminated.error().context.find("line 2"), std::string::npos);

    for (const char* text :
         {"a = 1\na = 2\n",                                                   // a key set twice
          "a 1\n",                                                            // no '='
          "a = 1 2\n",                                                        // junk after the value
          "a = [1, 2\n",                                                      // unterminated array
          "a = 1979-05-27\n",                                                 // dates are not supported
          "a = \"\"\"text\"\"\"\n",                                           // nor multi-line strings
          "a.b = 1\n",                                                        // nor dotted keys
          "a = nan\n", "a = 0x1p4\n", "a = .5\n", "a = 1.5.2\n", "a = inf\n", // values must be finite
          "a = 0x10\n",                                                       // decimal only
          "[t]\nx = 1\n[t]\ny = 2\n",                                         // a table defined twice
          "a = 1\n[a]\nb = 2\n",                                              // a value reopened as a table
          "a = 1\n[[a]]\nb = 2\n",                                            // or as an array of tables
          "[t\nx = 1\n"}) {                                                   // unterminated header
        const auto document = parse_toml(text);
        ASSERT_FALSE(document.has_value()) << text;
        EXPECT_EQ(document.error().code, ErrorCode::ParseFailure) << text;
    }
}

TEST(Toml, TypedLookupsRejectTheWrongKindAndUnknownKeys) {
    const auto document = parse_toml("name = \"x\"\nmass_kg = 2\nlist = [1, \"two\"]\n");
    ASSERT_TRUE(document.has_value());
    EXPECT_EQ(document->number_at("name").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(document->string_at("mass_kg").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(document->boolean_or("mass_kg", true).error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(document->array_or_empty("name").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(document->numbers_or_empty("list").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(document->strings_or_empty("list").error().code, ErrorCode::ParseFailure);
    EXPECT_EQ(document->number_at("absent").error().code, ErrorCode::ParseFailure);
    EXPECT_TRUE(document->expect_keys({"name", "mass_kg", "list"}).has_value());
    const auto unknown = document->expect_keys({"name", "list"});
    ASSERT_FALSE(unknown.has_value());
    EXPECT_NE(unknown.error().context.find("mass_kg"), std::string::npos);
}

} // namespace
