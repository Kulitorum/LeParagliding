#include "engine_paths.h"

#include <array>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

extern "C" int MAIN__();
extern "C" void f_exit();

namespace {

constexpr std::array<std::string_view, 4> outputFiles{
    "leparagliding.dxf",
    "lep-3d.dxf",
    "lep-out.txt",
    "lines.txt",
};

void printUsage()
{
    std::cout
        << "LEparagliding C++ engine 3.17\n"
        << "Usage: leparagliding-engine <design-file> <output-directory>\n"
        << "\n"
        << "Relative airfoil paths are resolved from the design file's directory.\n";
}

std::string pathToUtf8(const std::filesystem::path &path)
{
    const auto encoded = path.u8string();
    return {reinterpret_cast<const char *>(encoded.data()), encoded.size()};
}

int runEngine(const std::filesystem::path &inputArgument,
              const std::filesystem::path &outputArgument)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    try {
        const auto input = std::filesystem::absolute(inputArgument).lexically_normal();
        const auto output = std::filesystem::absolute(outputArgument).lexically_normal();

        if (!std::filesystem::is_regular_file(input)) {
            std::cerr << "Input file does not exist: " << pathToUtf8(input) << '\n';
            return 2;
        }

        std::filesystem::create_directories(output);
        if (!std::filesystem::is_directory(output)) {
            std::cerr << "Output path is not a directory: " << pathToUtf8(output) << '\n';
            return 2;
        }

        for (const auto fileName : outputFiles) {
            std::error_code error;
            std::filesystem::remove(output / fileName, error);
            if (error) {
                std::cerr << "Cannot replace output file "
                          << pathToUtf8(output / fileName) << ": " << error.message() << '\n';
                return 2;
            }
        }

        const std::string inputUtf8 = pathToUtf8(input);
        const std::string outputUtf8 = pathToUtf8(output);
        lep_configure_paths(inputUtf8.c_str(), outputUtf8.c_str());
        std::filesystem::current_path(input.parent_path());

        const int result = MAIN__();
        f_exit();
        return result;
    } catch (const std::exception &exception) {
        std::cerr << "Engine error: " << exception.what() << '\n';
        return 2;
    }
}

} // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t *argv[])
{
    if (argc == 2
        && (std::wstring_view(argv[1]) == L"--help"
            || std::wstring_view(argv[1]) == L"-h")) {
        printUsage();
        return 0;
    }
    if (argc != 3) {
        printUsage();
        return 2;
    }
    return runEngine(std::filesystem::path(argv[1]), std::filesystem::path(argv[2]));
}
#else
int main(int argc, char *argv[])
{
    if (argc == 2
        && (std::string_view(argv[1]) == "--help"
            || std::string_view(argv[1]) == "-h")) {
        printUsage();
        return 0;
    }
    if (argc != 3) {
        printUsage();
        return 2;
    }
    return runEngine(std::filesystem::u8path(argv[1]), std::filesystem::u8path(argv[2]));
}
#endif
