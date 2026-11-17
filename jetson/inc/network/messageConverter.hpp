#pragma once

#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace network {

struct CommandSpec {
    std::string name;
    std::string type;   // "int", "float", "string"
};

using CommandValue = std::variant<int, float, std::string>;

struct CommandArg {
    std::string  name;
    CommandValue value;
};

// Kiểm tra và định dạng text gửi ra UDP: "@action:v1;v2;;\r\n".
// Giống hệt hardware::serialhandler::MessageConverter trừ ký tự đầu, tên log và bảng commands(): sửa một bên thì sửa
// cả hai.
class MessageConverter {
public:
    explicit MessageConverter(std::ostream& logger = std::cerr);

    std::optional<std::string> get_command(const std::string& action,
                                           const std::vector<CommandArg>& args) const;

    // Kiểm tra: action tồn tại, đủ tham số, đúng tên, không trùng tên, đúng kiểu
    bool verify_command(const std::string& action,
                        const std::vector<CommandArg>& args) const;

    static const std::unordered_map<std::string, std::vector<CommandSpec>>& commands();

private:
    std::ostream& logger_;
};

} // namespace network
