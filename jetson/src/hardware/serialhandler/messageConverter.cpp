#include <hardware/serialhandler/messageConverter.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace hardware::serialhandler {

namespace {

// Khác network/messageConverter.cpp ở 2 hằng dưới đây và bảng commands()
constexpr char kPrefix    = '#';                     // lệnh xuống Nucleo: "#action:v1;v2;;\r\n"
constexpr char kLogName[] = "[Message Converter]";

// Đổi giá trị sang chuỗi theo kiểu trong spec (int được nhận cho "float"). nullopt nếu sai kiểu hoặc NaN/inf.
std::optional<std::string> format_value(const CommandValue& value, const std::string& type) {
    if (type == "int") {
        if (const auto* p = std::get_if<int>(&value)) return std::to_string(*p);
        return std::nullopt;
    }
    if (type == "float") {
        float number;
        if (const auto* pf = std::get_if<float>(&value))    number = *pf;
        else if (const auto* pi = std::get_if<int>(&value)) number = static_cast<float>(*pi);
        else return std::nullopt;

        if (!std::isfinite(number)) return std::nullopt;
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(number));   // chỉnh số chữ số thập phân ở đây
        return std::string(buffer);
    }
    if (type == "string") {
        if (const auto* p = std::get_if<std::string>(&value)) return *p;
        return std::nullopt;
    }
    return std::nullopt;   // spec khai báo kiểu lạ
}

} // namespace

const std::unordered_map<std::string, std::vector<CommandSpec>>& MessageConverter::commands() {
    static const std::unordered_map<std::string, std::vector<CommandSpec>> table = {
        {"speed", {{"speed", "float"}}},
        {"steer", {{"steer", "float"}}},
        {"dir",   {{"dir",   "int"}}}
    };
    return table;
}

MessageConverter::MessageConverter(std::ostream& logger) : logger_(logger) {}

bool MessageConverter::verify_command(const std::string& action,
                                      const std::vector<CommandArg>& args) const {
    const auto& table = commands();
    const auto  it    = table.find(action);
    if (it == table.end()) {
        logger_ << kLogName << " unknown action: " << action << '\n';
        return false;
    }
    const std::vector<CommandSpec>& spec = it->second;

    if (args.size() != spec.size()) {
        logger_ << kLogName << " Number of arguments does not match "
                << args.size() << " vs " << spec.size() << '\n';
        return false;
    }

    for (std::size_t i = 0; i < args.size(); ++i) {
        const CommandArg& a = args[i];

        // Tên phải có trong spec
        const auto pos = std::find_if(spec.begin(), spec.end(),
            [&](const CommandSpec& s) { return s.name == a.name; });
        if (pos == spec.end()) {
            logger_ << kLogName << ' ' << action
                    << " should not contain key: " << a.name << '\n';
            return false;
        }

        // Không trùng tên (nếu thiếu bước này, size bằng nhau nhưng vẫn có thể thiếu key)
        for (std::size_t j = 0; j < i; ++j) {
            if (args[j].name == a.name) {
                logger_ << kLogName << ' ' << action
                        << " has duplicate key: " << a.name << '\n';
                return false;
            }
        }

        // Kiểu phải khớp
        if (!format_value(a.value, pos->type)) {
            logger_ << kLogName << ' ' << action << '.' << a.name
                    << " expects " << pos->type << " (wrong type or invalid value)\n";
            return false;
        }
    }
    return true;
}

std::optional<std::string> MessageConverter::get_command(const std::string& action,
                                                         const std::vector<CommandArg>& args) const {
    if (!verify_command(action, args)) return std::nullopt;

    const auto& spec = commands().at(action);
    std::string command(1, kPrefix);
    command += action;
    command += ':';

    // Xuất theo thứ tự của spec, không theo thứ tự người gọi truyền vào
    for (const CommandSpec& s : spec) {
        const auto it = std::find_if(args.begin(), args.end(),
            [&](const CommandArg& a) { return a.name == s.name; });
        command += *format_value(it->value, s.type);   // verify đã đảm bảo hợp lệ
        command += ';';
    }

    command += ";\r\n";
    return command;
}

} // namespace hardware::serialhandler
