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
