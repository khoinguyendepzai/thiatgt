#pragma once

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace utils {

// Bỏ khoảng trắng (" \t\r\n") ở đầu/cuối chuỗi
inline std::string strip(const std::string& text) {
    const char* ws    = " \t\r\n";
    const auto  first = text.find_first_not_of(ws);
    if (first == std::string::npos) return {};
    const auto last = text.find_last_not_of(ws);
    return text.substr(first, last - first + 1);
}

// Tách chuỗi theo separator, luôn trả về ít nhất một phần tử
inline std::vector<std::string> split(const std::string& text, char separator) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true) {
        const auto pos = text.find(separator, start);
        if (pos == std::string::npos) {
            fields.push_back(text.substr(start));
            return fields;
        }
        fields.push_back(text.substr(start, pos - start));
        start = pos + 1;
    }
}

// Kiểm tra có ít nhất count field (thừa thì bỏ qua), thiếu thì ném std::invalid_argument
inline void require_fields(const std::vector<std::string>& fields, std::size_t count) {
    if (fields.size() < count) {
        throw std::invalid_argument("expected at least " + std::to_string(count) +
                                    " fields, got " + std::to_string(fields.size()));
    }
}

// Chữ thường
inline std::string to_lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

// Chữ hoa
inline std::string to_upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

// Mọi byte đều < 128
inline bool is_ascii(const std::string& text) {
    for (unsigned char c : text) {
        if (c >= 128) return false;
    }
    return true;
}

// Cả chuỗi (bỏ khoảng trắng đầu/cuối) phải là số thực hữu hạn, không thì ném std::invalid_argument
inline float to_float(const std::string& text) {
    const std::string s = strip(text);
    char* end = nullptr;
    const float value = std::strtof(s.c_str(), &end);
    if (s.empty() || end != s.c_str() + s.size() || !std::isfinite(value)) {
        throw std::invalid_argument("not a float: '" + text + "'");
    }
    return value;
}

// Cả chuỗi phải là số nguyên vừa kiểu int, không thì ném std::invalid_argument
inline int to_int(const std::string& text) {
    const std::string s = strip(text);
    char* end = nullptr;
    errno = 0;
    const long long value = std::strtoll(s.c_str(), &end, 10);
    if (s.empty() || end != s.c_str() + s.size() || errno == ERANGE ||
        value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("not an int: '" + text + "'");
    }
    return static_cast<int>(value);
}

} // namespace utils
