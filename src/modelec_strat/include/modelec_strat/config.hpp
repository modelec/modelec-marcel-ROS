#pragma once
#include <map>
#include <string>
#include <tinyxml2.h>

#include "obstacle.hpp"

namespace Modelec
{
    class Config
    {

    public:
        static bool load(const std::string& filepath);

        template<typename T>
        static T get(const std::string& key, const T& default_value = T());

        static void printAll();

    private:
        static void parseNode(tinyxml2::XMLElement* element, const std::string& prefix);

        static inline std::unordered_map<std::string, std::string> values_;
    };

    inline bool Config::load(const std::string& filepath)
    {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("Config"), "Failed to load XML file: %s", doc.ErrorName());
            return false;
        }

        auto* root = doc.RootElement();
        if (!root)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Config"), "No root element found in XML file");
            return false;
        }

        parseNode(root, "");
        return true;
    }

    inline void Config::parseNode(tinyxml2::XMLElement* element, const std::string& prefix) {
        std::string key_prefix = prefix.empty() ? element->Name() : prefix + "." + element->Name();

        const char* text = element->GetText();
        if (text && std::string(text).find_first_not_of(" \n\t") != std::string::npos) {
            values_[key_prefix] = text;
        }

        const tinyxml2::XMLAttribute* attr = element->FirstAttribute();
        while (attr) {
            std::string attr_key = key_prefix + "@" + attr->Name();
            values_[attr_key] = attr->Value();
            attr = attr->Next();
        }

        for (tinyxml2::XMLElement* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
            parseNode(child, key_prefix);
        }
    }

    inline void Config::printAll()
    {
        RCLCPP_INFO(rclcpp::get_logger("Config"), "Loaded %zu configuration values", values_.size());

        for (const auto& pair : values_) {
            RCLCPP_INFO(rclcpp::get_logger("Config"), "%s: %s", pair.first.c_str(), pair.second.c_str());
        }
    }

    template<typename T>
    T Config::get(const std::string& key, const T& default_value) {
        auto it = values_.find(key);
        if (it == values_.end()) return default_value;

        std::istringstream iss(it->second);
        T result;
        if (!(iss >> result)) return default_value;
        return result;
    }

    template<>
    inline std::string Config::get<std::string>(const std::string& key, const std::string& default_value) {
        auto it = values_.find(key);
        return it != values_.end() ? it->second : default_value;
    }

    template<>
    inline bool Config::get<bool>(const std::string& key, const bool& default_value) {
        auto str = get<std::string>(key, default_value ? "true" : "false");
        return str == "true" || str == "1";
    }
}
