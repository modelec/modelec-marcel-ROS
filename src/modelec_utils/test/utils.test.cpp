#include <gtest/gtest.h>
#include <modelec_utils/utils.hpp>

TEST(UtilsTest, Split) {
    std::string s = "a,b,c";
    std::vector<std::string> result = Modelec::split(s, ',');
    ASSERT_EQ(result.size(), 3);
    EXPECT_EQ(result[0], "a");
    EXPECT_EQ(result[1], "b");
    EXPECT_EQ(result[2], "c");
}

TEST(UtilsTest, Join) {
    std::vector<std::string> v{"a", "b", "c"};
    std::string result = Modelec::join(v, ",");
    EXPECT_EQ(result, "a,b,c");
}

TEST(UtilsTest, StartsWith) {
    EXPECT_TRUE(Modelec::startsWith("hello world", "hello"));
    EXPECT_FALSE(Modelec::startsWith("hello world", "world"));
}

TEST(UtilsTest, EndsWith) {
    EXPECT_TRUE(Modelec::endsWith("hello world", "world"));
    EXPECT_FALSE(Modelec::endsWith("hello world", "hello"));
}

TEST(UtilsTest, Contains) {
    EXPECT_TRUE(Modelec::contains("hello world", "lo wo"));
    EXPECT_FALSE(Modelec::contains("hello world", "bye"));
}

TEST(UtilsTest, Trim) {
    EXPECT_EQ(Modelec::trim("   hello   "), "hello");
    EXPECT_EQ(Modelec::trim("\t\n hello \r\n"), "hello");
    EXPECT_EQ(Modelec::trim(""), "");
    EXPECT_EQ(Modelec::trim("   "), "");
}

TEST(UtilsTest, MapValueInt) {
    int result = Modelec::mapValue<int>(5, 0, 10, 0, 100);
    EXPECT_EQ(result, 50);
}

TEST(UtilsTest, MapValueDouble) {
    auto result = Modelec::mapValue<double>(2.5, 0.0, 5.0, 0.0, 10.0);
    EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST(UtilsTest, MapValueNegative) {
    auto result = Modelec::mapValue<double>(-5.0, -10.0, 0.0, 0.0, 100.0);
    EXPECT_DOUBLE_EQ(result, 50.0);
}