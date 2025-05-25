#include <modelec_utils/utils.hpp>

namespace Modelec {

    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> result;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delim)) {
            result.push_back(token);
        }
        return result;
    }

    std::string join(const std::vector<std::string>& v, const std::string& delim)
    {
        std::string result;
        for (size_t i = 0; i < v.size(); ++i) {
            if (i != 0) {
                result += delim;
            }
            result += v[i];
        }
        return result;
    }

    bool startsWith(const std::string& s, const std::string& prefix)
    {
        return s.rfind(prefix, 0) == 0;
    }

    bool endsWith(const std::string& s, const std::string& suffix)
    {
        return s.size() >= suffix.size() && s.rfind(suffix) == s.size() - suffix.size();
    }

    bool contains(const std::string& s, const std::string& substring)
    {
        return s.find(substring) != std::string::npos;
    }

    std::string trim(const std::string& s)
    {
        size_t start = s.find_first_not_of(" \t\n\r\f\v");
        if (start == std::string::npos)
            return "";

        size_t end = s.find_last_not_of(" \t\n\r\f\v");
        return s.substr(start, end - start + 1);
    }
};