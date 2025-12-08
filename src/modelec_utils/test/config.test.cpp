#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include "modelec_utils/config.hpp"

TEST(ConfigTest, LoadValidXML)
{
    // Create temporary XML file
    const std::string filepath = "test_config.xml";

    std::ofstream file(filepath);
    file <<
        "<root attr1=\"A\" attr2=\"B\">"
        "   <child1>123</child1>"
        "   <child2 flag=\"true\">3.14</child2>"
        "   <nested>"
        "       <deep>hello</deep>"
        "   </nested>"
        "</root>";
    file.close();

    EXPECT_TRUE(Modelec::Config::load(filepath));

    // Root attributes
    EXPECT_EQ(Modelec::Config::get<std::string>("root@attr1", ""), "A");
    EXPECT_EQ(Modelec::Config::get<std::string>("root@attr2", ""), "B");

    // Simple child element
    EXPECT_EQ(Modelec::Config::get<int>("root.child1", 0), 123);

    // Child with attribute
    EXPECT_DOUBLE_EQ(Modelec::Config::get<double>("root.child2", 0.0), 3.14);
    EXPECT_TRUE(Modelec::Config::get<bool>("root.child2@flag", false));

    // Deep nested node
    EXPECT_EQ(Modelec::Config::get<std::string>("root.nested.deep", ""), "hello");
}

TEST(ConfigTest, LoadInvalidXML)
{
    EXPECT_FALSE(Modelec::Config::load("no_such_file.xml"));
}

TEST(ConfigTest, DefaultValues)
{
    // Key does not exist → default returned
    EXPECT_EQ(Modelec::Config::get<int>("missing.int", 42), 42);
    EXPECT_EQ(Modelec::Config::get<std::string>("missing.string", "default"), "default");
    EXPECT_FALSE(Modelec::Config::get<bool>("missing.bool", false));
}
