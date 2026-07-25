#pragma once

#include <filesystem>

class PreparedInput
{
public:
    PreparedInput() = default;
    ~PreparedInput();

    PreparedInput(const PreparedInput &) = delete;
    PreparedInput &operator=(const PreparedInput &) = delete;
    PreparedInput(PreparedInput &&other) noexcept;
    PreparedInput &operator=(PreparedInput &&other) noexcept;

    static PreparedInput forVersion328(
        const std::filesystem::path &source,
        const std::filesystem::path &temporaryDirectory);

    const std::filesystem::path &path() const;
    bool wasMigrated() const;
    bool addedVersion328Sections() const;
    bool strippedEmbeddedHistory() const;
    bool strippedBlankLines() const;

private:
    std::filesystem::path path_;
    std::filesystem::path temporaryPath_;
    bool addedVersion328Sections_ = false;
    bool strippedEmbeddedHistory_ = false;
    bool strippedBlankLines_ = false;
};
