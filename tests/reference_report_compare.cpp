#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr double absoluteTolerance = 0.00015;

const std::regex &numberPattern()
{
    static const std::regex pattern(
        R"([-+]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[EeDd][-+]?[0-9]+)?)");
    return pattern;
}

std::string readFile(const char *path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error(std::string("Could not read ") + path);
    }
    return {
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
}

std::vector<std::string> lines(std::string text)
{
    text.erase(
        std::remove(text.begin(), text.end(), '\r'),
        text.end());

    std::vector<std::string> result;
    std::size_t begin = 0;
    while (begin < text.size()) {
        const std::size_t end = text.find('\n', begin);
        result.emplace_back(
            text.substr(
                begin,
                (end == std::string::npos ? text.size() : end) - begin));
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return result;
}

std::string collapseWhitespace(std::string_view text)
{
    std::string result;
    bool pendingSpace = false;
    for (const unsigned char character : text) {
        if (character == ' ' || character == '\t') {
            pendingSpace = !result.empty();
        } else {
            if (pendingSpace) {
                result.push_back(' ');
            }
            result.push_back(static_cast<char>(character));
            pendingSpace = false;
        }
    }
    return result;
}

struct ParsedLine
{
    std::string structure;
    std::vector<double> values;
};

ParsedLine parseLine(const std::string &line)
{
    ParsedLine result;
    std::string structure;
    std::size_t previous = 0;

    for (std::sregex_iterator match(line.begin(), line.end(), numberPattern()), end;
         match != end;
         ++match) {
        const std::size_t position =
            static_cast<std::size_t>(match->position());
        structure.append(line, previous, position - previous);
        structure.append("{number}");

        std::string number = match->str();
        std::replace(number.begin(), number.end(), 'D', 'E');
        std::replace(number.begin(), number.end(), 'd', 'e');
        result.values.push_back(std::stod(number));
        previous = position + static_cast<std::size_t>(match->length());
    }
    structure.append(line, previous, std::string::npos);
    result.structure = collapseWhitespace(structure);
    return result;
}

bool compareReports(const char *referencePath, const char *actualPath)
{
    const auto referenceLines = lines(readFile(referencePath));
    const auto actualLines = lines(readFile(actualPath));
    if (referenceLines.size() != actualLines.size()) {
        std::cerr << "Line count differs: reference " << referenceLines.size()
                  << ", actual " << actualLines.size() << '\n';
        return false;
    }

    for (std::size_t index = 0; index < referenceLines.size(); ++index) {
        if (referenceLines[index] == actualLines[index]) {
            continue;
        }

        const ParsedLine reference = parseLine(referenceLines[index]);
        const ParsedLine actual = parseLine(actualLines[index]);
        if (reference.structure != actual.structure
            || reference.values.size() != actual.values.size()) {
            std::cerr << "Report structure differs at line " << index + 1
                      << "\nreference: " << referenceLines[index]
                      << "\nactual:    " << actualLines[index] << '\n';
            return false;
        }

        for (std::size_t value = 0; value < reference.values.size(); ++value) {
            if (std::abs(reference.values[value] - actual.values[value])
                > absoluteTolerance) {
                std::cerr << "Numeric value differs at line " << index + 1
                          << "\nreference: " << referenceLines[index]
                          << "\nactual:    " << actualLines[index] << '\n';
                return false;
            }
        }
    }
    return true;
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: reference-report-compare <reference> <actual>\n";
        return 2;
    }

    try {
        return compareReports(argv[1], argv[2]) ? 0 : 1;
    } catch (const std::exception &exception) {
        std::cerr << exception.what() << '\n';
        return 2;
    }
}
