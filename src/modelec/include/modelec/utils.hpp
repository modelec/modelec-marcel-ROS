#pragma once

#include <string>
#include <vector>
#include <sstream>

namespace Modelec {
    #define PI 3.14159265358979323846
    
    inline std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> result;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delim)) {
            result.push_back(token);
        }
        return result;
    }

    inline std::string join(const std::vector<std::string> &v, const std::string &delim) {
        std::string result;
        for (size_t i = 0; i < v.size(); ++i) {
            if (i != 0) {
                result += delim;
            }
            result += v[i];
        }
        return result;
    }

    inline bool startsWith(const std::string &s, const std::string &prefix) {
        return s.rfind(prefix, 0) == 0;
    }

    inline bool endsWith(const std::string &s, const std::string &suffix) {
        return s.size() >= suffix.size() && s.rfind(suffix) == s.size() - suffix.size();
    }

    inline bool contains(const std::string &s, const std::string &substring) {
        return s.find(substring) != std::string::npos;
    }

    template <typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
    T mapValue(T v, T v_min, T v_max, T v_min_prime, T v_max_prime) {
        return v_min_prime + (((v - v_min) * (v_max_prime - v_min_prime)) / (v_max - v_min));
    }
}
