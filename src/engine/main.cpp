#include "engine_paths.h"
#include "input_migration.h"
#include "nurbs_model.h"

#include <array>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

extern "C" int MAIN__();
extern "C" void f_exit();

namespace {

constexpr std::array<std::string_view, 5> outputFiles{
    "leparagliding.dxf",
    "lep-3d.dxf",
    "lep-3d.step",
    "lep-out.txt",
    "lines.txt",
};

constexpr std::array<std::string_view, 7> additionalOutputFiles{
    "run-log.txt",
    "lep-3d-surfaces.dxf",
    "stl/lep-3d-surfaces.scad",
    "stl/lep-3d-surfaces.stl",
    "stl/Upper-surface.stl",
    "stl/Vents-surface.stl",
    "stl/Lower-surface.stl",
};

void printUsage()
{
    std::cout
        << "LEparagliding C++ engine 3.28\n"
        << "Usage: leparagliding-engine [--resource-dir <directory>] "
           "<design-file> <output-directory>\n"
        << "\n"
        << "Relative airfoil paths are resolved from the design file's directory,\n"
        << "or from --resource-dir when calculating a temporary design copy.\n";
}

std::string pathToUtf8(const std::filesystem::path &path)
{
    const auto encoded = path.u8string();
    return {reinterpret_cast<const char *>(encoded.data()), encoded.size()};
}

int runEngine(const std::filesystem::path &inputArgument,
              const std::filesystem::path &outputArgument,
              const std::filesystem::path &resourceArgument = {})
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::setvbuf(stderr, nullptr, _IONBF, 0);

    try {
        const auto input = std::filesystem::absolute(inputArgument).lexically_normal();
        const auto output = std::filesystem::absolute(outputArgument).lexically_normal();
        const auto resourceDirectory =
            resourceArgument.empty()
                ? input.parent_path()
                : std::filesystem::absolute(resourceArgument).lexically_normal();

        if (!std::filesystem::is_regular_file(input)) {
            std::cerr << "Input file does not exist: " << pathToUtf8(input) << '\n';
            return 2;
        }
        if (!std::filesystem::is_directory(resourceDirectory)) {
            std::cerr << "Resource path is not a directory: "
                      << pathToUtf8(resourceDirectory) << '\n';
            return 2;
        }

        std::filesystem::create_directories(output);
        if (!std::filesystem::is_directory(output)) {
            std::cerr << "Output path is not a directory: " << pathToUtf8(output) << '\n';
            return 2;
        }

        const auto removeOutput = [&output](std::string_view fileName) {
            std::error_code error;
            std::filesystem::remove(output / fileName, error);
            if (error) {
                std::cerr << "Cannot replace output file "
                          << pathToUtf8(output / fileName) << ": " << error.message() << '\n';
                return false;
            }
            return true;
        };
        for (const auto fileName : outputFiles) {
            if (!removeOutput(fileName)) {
                return 2;
            }
        }
        for (const auto fileName : additionalOutputFiles) {
            if (!removeOutput(fileName)) {
                return 2;
            }
        }

        PreparedInput preparedInput = PreparedInput::forVersion328(input, output);
        if (preparedInput.addedVersion328Sections()) {
            std::cout
                << "Compatibility: added disabled defaults for sections 33-37 "
                   "to a temporary LEparagliding 3.28 input.\n";
        }
        if (preparedInput.strippedEmbeddedHistory()) {
            std::cout
                << "Compatibility: excluded embedded Studio version history "
                   "from the calculation input.\n";
        }
        if (preparedInput.strippedBlankLines()) {
            std::cout
                << "Compatibility: removed blank lines from the calculation "
                   "input (the 3.28 reader cannot tolerate them).\n";
        }

        const std::string inputUtf8 = pathToUtf8(preparedInput.path());
        const std::string outputUtf8 = pathToUtf8(output);
        lep_configure_paths(inputUtf8.c_str(), outputUtf8.c_str());
        std::filesystem::current_path(resourceDirectory);

        lep::resetNurbsModel();
        const int result = MAIN__();
        f_exit();
        if (result != 0) {
            return result;
        }

        const lep::NurbsWriteResult step =
            lep::writeNurbsStep(output / "lep-3d.step");
        for (const std::string &warning : step.warnings) {
            std::cerr << "NURBS model warning: " << warning << '\n';
        }
        if (!step.success) {
            std::cerr << "NURBS model error: " << step.error << '\n';
            return 2;
        }
        std::cout
            << "OCCT NURBS model: "
            << step.surfaceCount << " surfaces, "
            << step.splineCount << " spline curves\n"
            << "Named assembly: "
            << step.partCount << " parts, "
            << step.ribCount << " ribs, "
            << step.lineCount << " labeled lines\n"
            << "Sewn topology: "
            << step.sewnEdgeCount << " shared edges, "
            << step.freeEdgeCount << " designed free edges\n"
            << "Maximum NURBS/source deviation: "
            << step.maximumSourceDeviationMillimetres << " mm\n"
            << "Maximum source/legacy-grid deviation: "
            << step.maximumLegacyAgreementMillimetres << " mm\n"
            << "STEP model: "
            << pathToUtf8(output / "lep-3d.step") << '\n';
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
    if (argc == 3) {
        return runEngine(
            std::filesystem::path(argv[1]),
            std::filesystem::path(argv[2]));
    }
    if (argc == 5 && std::wstring_view(argv[1]) == L"--resource-dir") {
        return runEngine(
            std::filesystem::path(argv[3]),
            std::filesystem::path(argv[4]),
            std::filesystem::path(argv[2]));
    }
    printUsage();
    return 2;
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
    if (argc == 3) {
        return runEngine(
            std::filesystem::u8path(argv[1]),
            std::filesystem::u8path(argv[2]));
    }
    if (argc == 5 && std::string_view(argv[1]) == "--resource-dir") {
        return runEngine(
            std::filesystem::u8path(argv[3]),
            std::filesystem::u8path(argv[4]),
            std::filesystem::u8path(argv[2]));
    }
    printUsage();
    return 2;
}
#endif
