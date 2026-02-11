#pragma once

#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <functional>
#include <iostream>

namespace Modelec
{
    class Config
    {

    public:

        template<typename T>
        using BuilderFunc = std::function<T(const std::string& base_key)>;

        static bool load(const std::string& filepath);

        template<typename T>
        static T get(const std::string& key, const T& default_value = T());

        template<typename T>
        static std::vector<T> getArray(const std::string& prefix,
                                    BuilderFunc<T> builder = [](const std::string& base) { return get<T>(base); });

        static size_t count(const std::string& prefix);

        static void clear(const std::string& prefix);

        static bool has(const std::string& prefix);

        static void printAll();

    private:
        static void parseNode(tinyxml2::XMLElement* element, const std::string& key);

        static inline std::unordered_map<std::string, std::string> values_;
    };

    template<typename T>
    T Config::get(const std::string& key, const T& default_value) {
        auto it = values_.find(key);
        if (it == values_.end())
        {
            std::cerr << "Config key not found: " << key << std::endl;
            return default_value;
        }

        std::istringstream iss(it->second);
        T result;
        if (!(iss >> result))
        {
            std::cerr << "Config key has invalid format: " << key << " = " << it->second << std::endl;
            return default_value;
        }
        return result;
    }

    template<>
    inline std::string Config::get<std::string>(const std::string& key, const std::string& default_value) {
        auto it = values_.find(key);
        if (it == values_.end()) std::cerr << "Config key not found: " << key << std::endl;
        return it != values_.end() ? it->second : default_value;
    }

    template<>
    inline bool Config::get<bool>(const std::string& key, const bool& default_value) {
        auto str = get<std::string>(key, default_value ? "true" : "false");
        return str == "true" || str == "1";
    }

    template<typename T>
    std::vector<T> Config::getArray(const std::string& prefix,
                                BuilderFunc<T> builder)
    {
        std::vector<T> result;

        size_t n = count(prefix);
        result.reserve(n);

        if (n > 0)
        {
            if (n == 1 && !has(prefix + "[0]"))
            {
                result.emplace_back(builder(prefix));
            } else
            {
                for (size_t i = 0; i < n; ++i)
                {
                    std::string base = prefix + "[" + std::to_string(i) + "]";
                    result.emplace_back(builder(base));
                }
            }
        }

        return result;
    }
}
