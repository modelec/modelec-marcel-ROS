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

        template<typename K, typename V>
        using MapBuilderFunc = std::function<std::pair<K, V>(const std::string& base_key)>;

        static bool load(const std::string& filepath);

        template<typename T>
        static T get(const std::string& key, const T& default_value = T(), bool is_optional = false);

        template<typename T>
        static std::vector<T> getArray(const std::string& prefix,
                                    BuilderFunc<T> builder = [](const std::string& base) { return get<T>(base); });

        template<typename K, typename V>
        static std::unordered_map<K, V> getMap(
            const std::string& prefix,
            MapBuilderFunc<K, V> builder);

        static size_t count(const std::string& prefix);

        static void clear(const std::string& prefix);

        static bool has(const std::string& prefix);

        static void printAll();

        static void print(const std::string& prefix);

    private:
        static void parseNode(tinyxml2::XMLElement* element, const std::string& key);

        static inline std::unordered_map<std::string, std::string> values_;
    };

    template<typename T>
    T Config::get(const std::string& key, const T& default_value, bool is_optional) {
        auto it = values_.find(key);
        if (it == values_.end())
        {
            if (!is_optional)
            {
                std::cerr << "Config key not found: " << key << std::endl;
            }
            return default_value;
        }

        std::istringstream iss(it->second);
        T result;
        if (!(iss >> result))
        {
            if (!is_optional)
            {
                std::cerr << "Config key has invalid format: " << key << " = " << it->second << std::endl;
            }
            return default_value;
        }
        return result;
    }

    template<>
    inline std::string Config::get<std::string>(const std::string& key, const std::string& default_value, bool is_optional) {
        auto it = values_.find(key);
        if (it == values_.end())
        {
            if (is_optional)
            {
                std::cerr << "Config key not found: " << key << std::endl;
            }
            return default_value;
        }
        return it->second;
    }

    template<>
    inline bool Config::get<bool>(const std::string& key, const bool& default_value, bool is_optional) {
        auto str = get<std::string>(key, default_value ? "true" : "false", is_optional);
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
        } else
        {
            std::cerr << "Config array not found: " << prefix << std::endl;
        }

        return result;
    }

    template<typename K, typename V>
    std::unordered_map<K, V> Config::getMap(
        const std::string& prefix,
        MapBuilderFunc<K, V> builder)
    {
        std::unordered_map<K, V> result;

        size_t n = count(prefix);
        if (n == 0)
        {
            std::cerr << "Config map not found: " << prefix << std::endl;
            return result;
        }

        for (size_t i = 0; i < n; ++i)
        {
            std::string base = prefix + "[" + std::to_string(i) + "]";
            auto [key, value] = builder(base);
            result.emplace(key, value);
        }

        return result;
    }
}
