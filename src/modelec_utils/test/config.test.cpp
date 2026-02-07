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

TEST(ConfigTest, CountArray)
{
    // Create temporary XML file
    const std::string filepath = "test_array_config.xml";

    std::ofstream file(filepath);
    file <<
        "<root>"
        "   <item>1</item>"
        "   <item>2</item>"
        "   <item>3</item>"
        "</root>";
    file.close();

    EXPECT_TRUE(Modelec::Config::load(filepath));

    EXPECT_EQ(Modelec::Config::count("root.item"), 3);
}

TEST(ConfigTest, GetArray)
{
    // Create temporary XML file
    const std::string filepath = "test_array_config.xml";

    std::ofstream file(filepath);
    file <<
        "<root>"
        "   <item a=\"b\">1</item>"
        "   <item a=\"c\">2</item>"
        "   <item a=\"d\">3</item>"
        "</root>";
    file.close();

    struct Item
    {
        int value;
        std::string attr;
    };

    EXPECT_TRUE(Modelec::Config::load(filepath));

    auto array = Modelec::Config::getArray<int>("root.item");

    EXPECT_EQ(array.size(), 3);
    EXPECT_EQ(array[0], 1);
    EXPECT_EQ(array[1], 2);
    EXPECT_EQ(array[2], 3);

    auto array2 = Modelec::Config::getArray<Item>("root.item",
        [](const std::string& base)
        {
            return Item{
                Modelec::Config::get<int>(base, 0),
                Modelec::Config::get<std::string>(base + "@a", "")
            };
        });

    EXPECT_EQ(array2.size(), 3);
    EXPECT_EQ(array2[0].value, 1);
    EXPECT_EQ(array2[0].attr, "b");
    EXPECT_EQ(array2[1].value, 2);
    EXPECT_EQ(array2[1].attr, "c");
    EXPECT_EQ(array2[2].value, 3);
    EXPECT_EQ(array2[2].attr, "d");
}