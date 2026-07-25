#include "input_migration.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <fstream>
#include <iterator>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace {

constexpr std::array<std::string_view, 5> defaultSections{
    "*******************************************************\n"
    "*       33. DETAILED RISERS\n"
    "*******************************************************\n"
    "0\n",
    "*******************************************************\n"
    "*       34. LINES CHARACTERISTICS TABLE\n"
    "*******************************************************\n"
    "0\n",
    "*******************************************************\n"
    "*       35. SOLVE EQUILIBRIUM EQUATIONS\n"
    "*******************************************************\n"
    "0\n",
    "*******************************************************\n"
    "*       36. CREATE FILES FOR XFLR5 ANALYSIS\n"
    "*******************************************************\n"
    "0\n",
    "*******************************************************\n"
    "*       37. SOME SPECIAL PARAMETERS\n"
    "*******************************************************\n"
    "0\n",
};

constexpr std::string_view embeddedHistoryMarker =
    "* >>> LEPARAGLIDING STUDIO HISTORY V1 >>>";

bool stripEmbeddedHistory(std::string *text)
{
    std::size_t marker = text->find(embeddedHistoryMarker);
    while (marker != std::string::npos) {
        if (marker == 0 || (*text)[marker - 1] == '\n') {
            text->erase(marker);
            return true;
        }
        marker = text->find(embeddedHistoryMarker, marker + 1);
    }
    return false;
}

// Blank lines are never data in a 3.28 design, but the translated reader's
// comment skips are strictly positional and consume them in place of the
// comment they were meant to skip, silently derailing the parse. Dropping
// them makes the format tolerate blank lines anywhere.
bool stripBlankLines(std::string *text)
{
    std::string kept;
    kept.reserve(text->size());
    bool changed = false;
    std::size_t begin = 0;
    while (begin < text->size()) {
        std::size_t end = text->find('\n', begin);
        const bool lastLine = end == std::string::npos;
        if (lastLine) {
            end = text->size();
        }
        const std::string_view line(text->data() + begin, end - begin);
        const bool blank = line.find_first_not_of(" \t\r") == std::string_view::npos;
        if (blank) {
            changed = true;
        } else {
            kept.append(line);
            if (!lastLine) {
                kept.push_back('\n');
            }
        }
        begin = end + 1;
    }
    if (changed) {
        *text = std::move(kept);
    }
    return changed;
}

int sectionNumber(std::string_view line)
{
    const std::size_t marker = line.find('*');
    if (marker == std::string_view::npos) {
        return 0;
    }

    std::size_t begin = marker + 1;
    while (begin < line.size() && (line[begin] == ' ' || line[begin] == '\t')) {
        ++begin;
    }
    const std::size_t end = line.find('.', begin);
    if (end == std::string_view::npos || end == begin) {
        return 0;
    }

    int number = 0;
    const auto parsed =
        std::from_chars(line.data() + begin, line.data() + end, number);
    return parsed.ec == std::errc{} && parsed.ptr == line.data() + end
               ? number
               : 0;
}

std::array<bool, 38> findSections(const std::string &text)
{
    std::array<bool, 38> found{};
    std::size_t begin = 0;
    while (begin <= text.size()) {
        const std::size_t end = text.find('\n', begin);
        const std::string_view line(
            text.data() + begin,
            (end == std::string::npos ? text.size() : end) - begin);
        const int number = sectionNumber(line);
        if (number >= 1 && number < static_cast<int>(found.size())) {
            found[static_cast<std::size_t>(number)] = true;
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return found;
}

std::string readFile(const std::filesystem::path &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not read the design file for migration.");
    }
    return {
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
}

std::filesystem::path uniqueTemporaryPath(
    const std::filesystem::path &directory)
{
    const auto now = std::chrono::high_resolution_clock::now()
                         .time_since_epoch()
                         .count();
    std::random_device random;
    return directory
        / (".leparagliding-3.28-input-" + std::to_string(now) + "-"
           + std::to_string(random()) + ".tmp");
}

} // namespace

PreparedInput::~PreparedInput()
{
    if (!temporaryPath_.empty()) {
        std::error_code error;
        std::filesystem::remove(temporaryPath_, error);
    }
}

PreparedInput::PreparedInput(PreparedInput &&other) noexcept
    : path_(std::move(other.path_))
    , temporaryPath_(std::move(other.temporaryPath_))
    , addedVersion328Sections_(other.addedVersion328Sections_)
    , strippedEmbeddedHistory_(other.strippedEmbeddedHistory_)
    , strippedBlankLines_(other.strippedBlankLines_)
{
    other.temporaryPath_.clear();
    other.addedVersion328Sections_ = false;
    other.strippedEmbeddedHistory_ = false;
    other.strippedBlankLines_ = false;
}

PreparedInput &PreparedInput::operator=(PreparedInput &&other) noexcept
{
    if (this != &other) {
        if (!temporaryPath_.empty()) {
            std::error_code error;
            std::filesystem::remove(temporaryPath_, error);
        }
        path_ = std::move(other.path_);
        temporaryPath_ = std::move(other.temporaryPath_);
        addedVersion328Sections_ = other.addedVersion328Sections_;
        strippedEmbeddedHistory_ = other.strippedEmbeddedHistory_;
        strippedBlankLines_ = other.strippedBlankLines_;
        other.temporaryPath_.clear();
        other.addedVersion328Sections_ = false;
        other.strippedEmbeddedHistory_ = false;
        other.strippedBlankLines_ = false;
    }
    return *this;
}

PreparedInput PreparedInput::forVersion328(
    const std::filesystem::path &source,
    const std::filesystem::path &temporaryDirectory)
{
    PreparedInput result;
    result.path_ = source;

    std::string text = readFile(source);
    result.strippedEmbeddedHistory_ = stripEmbeddedHistory(&text);
    result.strippedBlankLines_ = stripBlankLines(&text);
    const auto found = findSections(text);

    bool seenMissing = false;
    int firstMissing = 38;
    for (int number = 33; number <= 37; ++number) {
        if (!found[static_cast<std::size_t>(number)]) {
            seenMissing = true;
            firstMissing = std::min(firstMissing, number);
        } else if (seenMissing) {
            throw std::runtime_error(
                "Sections 33 through 37 must be present in order without gaps.");
        }
    }

    if (firstMissing == 38 && !result.strippedEmbeddedHistory_
        && !result.strippedBlankLines_) {
        return result;
    }
    if (firstMissing != 38 && !found[32]) {
        throw std::runtime_error(
            "The design is missing section 32 and cannot be upgraded automatically.");
    }

    if (firstMissing != 38) {
        while (!text.empty()
               && (text.back() == '\r' || text.back() == '\n'
                   || text.back() == ' ' || text.back() == '\t')) {
            text.pop_back();
        }
        text.push_back('\n');
        for (int number = firstMissing; number <= 37; ++number) {
            text.append(defaultSections[static_cast<std::size_t>(number - 33)]);
        }
        result.addedVersion328Sections_ = true;
    }

    std::filesystem::create_directories(temporaryDirectory);
    result.temporaryPath_ = uniqueTemporaryPath(temporaryDirectory);
    std::ofstream output(result.temporaryPath_, std::ios::binary | std::ios::trunc);
    if (!output
        || !output.write(text.data(), static_cast<std::streamsize>(text.size()))) {
        throw std::runtime_error(
            "Could not create the temporary LEparagliding 3.28 input.");
    }
    output.close();
    result.path_ = result.temporaryPath_;
    return result;
}

const std::filesystem::path &PreparedInput::path() const
{
    return path_;
}

bool PreparedInput::wasMigrated() const
{
    return !temporaryPath_.empty();
}

bool PreparedInput::addedVersion328Sections() const
{
    return addedVersion328Sections_;
}

bool PreparedInput::strippedEmbeddedHistory() const
{
    return strippedEmbeddedHistory_;
}

bool PreparedInput::strippedBlankLines() const
{
    return strippedBlankLines_;
}
