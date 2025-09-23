#include <fmt/core.h>
#include <fmt/format.h>

int main() {
    fmt::print("Hello, {}!\n", "fmt library");
    std::string info = fmt::format("{} version: {}\n", "fmt", "9.1.0");
    fmt::print(info);
    fmt::print("Integer: {}\n", 123);
    fmt::print("Float: {:.3f}\n", 3.1415926535);
    return 0;
}
