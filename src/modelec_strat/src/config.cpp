#include <modelec_strat/config.hpp>

namespace Modelec
{
    bool Config::load(const std::string& filepath)
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

    void Config::parseNode(tinyxml2::XMLElement* element, const std::string& prefix) {
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

    void Config::printAll()
    {
        RCLCPP_INFO(rclcpp::get_logger("Config"), "Loaded %zu configuration values", values_.size());

        for (const auto& pair : values_) {
            RCLCPP_INFO(rclcpp::get_logger("Config"), "%s: %s", pair.first.c_str(), pair.second.c_str());
        }
    }
}
