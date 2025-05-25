#pragma once

#include <string>
#include <vector>
#include <sstream>

namespace Modelec {
    #define PI 3.14159265358979323846
    
    std::vector<std::string> split(const std::string &s, char delim);

    std::string join(const std::vector<std::string> &v, const std::string &delim);

    bool startsWith(const std::string &s, const std::string &prefix);

    bool endsWith(const std::string &s, const std::string &suffix);

    bool contains(const std::string &s, const std::string &substring);

    template <typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
    T mapValue(T v, T v_min, T v_max, T v_min_prime, T v_max_prime) {
        return v_min_prime + (((v - v_min) * (v_max_prime - v_min_prime)) / (v_max - v_min));
    }

    std::string trim(const std::string &s);
}
