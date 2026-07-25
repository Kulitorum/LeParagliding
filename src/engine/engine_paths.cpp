#include "engine_paths.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace {

std::string inputPath;
std::filesystem::path outputDirectory;
std::unordered_map<std::string, std::string> outputPaths;

std::string pathToUtf8(const std::filesystem::path &path)
{
    const auto encoded = path.u8string();
    return {reinterpret_cast<const char *>(encoded.data()), encoded.size()};
}

std::string trimFortranString(const char *text, int length)
{
    if (text == nullptr || length <= 0) {
        return {};
    }

    std::string result(text, static_cast<std::size_t>(length));
    while (!result.empty()
           && (result.back() == ' ' || result.back() == '\0'
               || result.back() == '\r' || result.back() == '\n')) {
        result.pop_back();
    }
    return result;
}

std::filesystem::path safeRelativeOutputPath(std::string name)
{
    std::replace(name.begin(), name.end(), '\\', '/');

    std::filesystem::path result;
    for (const auto &component : std::filesystem::u8path(name)) {
        if (component.empty() || component == "." || component == ".."
            || component == component.root_name()
            || component == component.root_directory()) {
            continue;
        }
        result /= component;
    }
    return result;
}

char *cacheOutputPath(const std::filesystem::path &relativePath)
{
    const std::filesystem::path fullPath = outputDirectory / relativePath;
    if (fullPath.has_parent_path()) {
        std::filesystem::create_directories(fullPath.parent_path());
    }

    const std::string key = pathToUtf8(relativePath);
    const auto [iterator, inserted] =
        outputPaths.try_emplace(key, pathToUtf8(fullPath));
    (void)inserted;
    return iterator->second.data();
}

#ifdef _WIN32
std::wstring utf8ToWide(const char *text)
{
    if (text == nullptr || *text == '\0') {
        return {};
    }

    const int size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text, -1, nullptr, 0);
    if (size <= 0) {
        return {};
    }

    std::wstring result(static_cast<std::size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text, -1, result.data(), size);
    result.pop_back();
    return result;
}
#endif

} // namespace

extern "C" void lep_configure_paths(const char *input, const char *output)
{
    inputPath = input == nullptr
                    ? std::string{}
                    : pathToUtf8(
                          std::filesystem::absolute(std::filesystem::u8path(input))
                              .lexically_normal());
    outputDirectory = output == nullptr
                          ? std::filesystem::path{}
                          : std::filesystem::absolute(std::filesystem::u8path(output))
                                .lexically_normal();
    outputPaths.clear();
}

extern "C" char *lep_input_path()
{
    return inputPath.data();
}

extern "C" char *lep_output_path(const char *fileName)
{
    if (fileName == nullptr) {
        return nullptr;
    }

    return cacheOutputPath(safeRelativeOutputPath(fileName));
}

extern "C" char *lep_output_path_fortran(const char *fileName, int length)
{
    const std::string name = trimFortranString(fileName, length);
    if (name.empty()) {
        return nullptr;
    }
    return cacheOutputPath(safeRelativeOutputPath(name));
}

extern "C" void lep_ensure_output_subdirectory(const char *directoryName)
{
    if (directoryName == nullptr) {
        return;
    }
    std::filesystem::create_directories(
        outputDirectory / safeRelativeOutputPath(directoryName));
}

extern "C" int lep_path_length(const char *path)
{
    if (path == nullptr) {
        return 0;
    }

    const auto length = std::strlen(path);
    return static_cast<int>(
        std::min(length, static_cast<std::size_t>(std::numeric_limits<int>::max())));
}

extern "C" int lep_count_fields(const char *text, int length)
{
    if (text == nullptr || length <= 0) {
        return 0;
    }

    int count = 0;
    bool inField = false;
    for (int index = 0; index < length; ++index) {
        const unsigned char character = static_cast<unsigned char>(text[index]);
        const bool whitespace =
            character == '\0' || character == ' ' || character == '\t'
            || character == '\r' || character == '\n' || character == '\f';
        if (whitespace) {
            inField = false;
        } else if (!inField) {
            ++count;
            inField = true;
        }
    }
    return count;
}

extern "C" int lep_parse_planform_row(
    const char *text,
    int length,
    double *ribNumber,
    double *xRib,
    double *leadingEdge,
    double *trailingEdge,
    double *xPrime,
    double *z,
    double *beta,
    double *rotationPoint,
    double *washin,
    double *tipAngle1,
    double *tipAngle2)
{
    if (text == nullptr || length <= 0) {
        return 0;
    }

    std::array<double *, 11> destinations{
        ribNumber,
        xRib,
        leadingEdge,
        trailingEdge,
        xPrime,
        z,
        beta,
        rotationPoint,
        washin,
        tipAngle1,
        tipAngle2,
    };

    std::istringstream input(std::string(text, static_cast<std::size_t>(length)));
    int count = 0;
    for (double *destination : destinations) {
        double value = 0.0;
        if (!(input >> value)) {
            break;
        }
        if (destination != nullptr) {
            *destination = value;
        }
        ++count;
    }

    if (count == 9) {
        if (tipAngle1 != nullptr) {
            *tipAngle1 = 0.0;
        }
        if (tipAngle2 != nullptr) {
            *tipAngle2 = 0.0;
        }
    }
    return count;
}

#ifdef _WIN32
extern "C" FILE *lep_fopen(const char *path, const char *mode)
{
    const std::wstring widePath = utf8ToWide(path);
    std::wstring wideMode = utf8ToWide(mode);
    if (widePath.empty() || wideMode.empty()) {
        return nullptr;
    }

    // libf2c opens existing formatted input as "r+" (or "r" as a
    // read-only fallback). MSVC text streams cannot reliably seek through
    // Unix-LF files using positions returned by ftell(), which breaks the
    // Fortran BACKSPACE statements used by the design parser. Binary input
    // preserves byte positions; lread.c accepts both LF and CRLF records.
    if (wideMode == L"r+") {
        wideMode = L"r+b";
    } else if (wideMode == L"r") {
        wideMode = L"rb";
    }

    return _wfopen(widePath.c_str(), wideMode.c_str());
}

extern "C" FILE *lep_freopen(const char *path, const char *mode, FILE *stream)
{
    const std::wstring widePath = utf8ToWide(path);
    const std::wstring wideMode = utf8ToWide(mode);
    if (widePath.empty() || wideMode.empty()) {
        return nullptr;
    }
    return _wfreopen(widePath.c_str(), wideMode.c_str(), stream);
}
#endif

// LEN_TRIM is a Fortran 90 intrinsic not supplied by the F77 runtime.
extern "C" long len_trim__(char *text, long length)
{
    if (text == nullptr || length <= 0) {
        return 0;
    }

    while (length > 0 && (text[length - 1] == ' ' || text[length - 1] == '\0')) {
        --length;
    }
    return length;
}
