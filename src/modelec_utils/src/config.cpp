#include <iostream>
#include <modelec_utils/config.hpp>

namespace Modelec
{
    bool Config::load(const std::string& filepath)
    {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
            return false;
        }

        auto* root = doc.RootElement();
        if (!root)
        {
            return false;
        }

        parseNode(root, root->Name());
        return true;
    }

    size_t Config::count(const std::string& prefix)
    {
        size_t max_index = 0;
        bool found = false;

        for (const auto& [key, _] : values_)
        {
            if (key.rfind(prefix + "[", 0) == 0)
            {
                auto start = key.find('[', prefix.size());
                auto end = key.find(']', start);
                if (start != std::string::npos && end != std::string::npos)
                {
                    size_t index = std::stoul(key.substr(start + 1, end - start - 1));
                    max_index = std::max(max_index, index);
                    found = true;
                }
            }
        }

        return found ? max_index + 1 : 0;
    }

    void Config::printAll()
    {
        for (const auto& [key, value] : values_)
        {
            std::cout << key << " = " << value << std::endl;
        }
    }

    void Config::parseNode(tinyxml2::XMLElement* element, const std::string& key) {
        if (const char* text = element->GetText())
        {
            if (std::string(text).find_first_not_of(" \n\t") != std::string::npos)
            {
                values_[key] = text;
            }
        }

        // Store attributes
        for (auto* attr = element->FirstAttribute(); attr; attr = attr->Next())
        {
            values_[key + "@" + attr->Name()] = attr->Value();
        }

        // Count children by name
        std::unordered_map<std::string, int> child_count;
        for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
        {
            child_count[child->Name()]++;
        }

        // Index children
        std::unordered_map<std::string, int> child_index;

        for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
        {
            std::string child_key = key + "." + child->Name();

            if (child_count[child->Name()] > 1)
            {
                int index = child_index[child->Name()]++;
                child_key += "[" + std::to_string(index) + "]";
            }

            parseNode(child, child_key);
        }
    }
}