#include "engine_paths.h"
#include "input_migration.h"
#include "nurbs_model.h"

#include <array>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

extern "C" int MAIN__();
extern "C" void f_exit();

namespace {

constexpr std::array<std::string_view, 6> outputFiles{
    "leparagliding.dxf",
    "lep-3d.dxf",
    "lep-3d.step",
    "lep-3d.xbf",
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
        << "Usage: leparagliding-engine [--preview] [--no-construction-curves]\n"
           "       [--resource-dir <directory>] <design-file> <output-directory>\n"
        << "\n"
        << "Relative airfoil paths are resolved from the design file's directory,\n"
        << "or from --resource-dir when calculating a temporary design copy.\n"
        << "--preview writes the 3D model as binary XCAF (lep-3d.xbf) instead\n"
        << "of STEP; the Studio preview loads it directly.\n"
        << "--no-construction-curves omits the interior surface-wireframe curve\n"
        << "groups from the model; suspension lines are always written.\n";
}

std::string pathToUtf8(const std::filesystem::path &path)
{
    const auto encoded = path.u8string();
    return {reinterpret_cast<const char *>(encoded.data()), encoded.size()};
}

int runEngine(const std::filesystem::path &inputArgument,
              const std::filesystem::path &outputArgument,
              const std::filesystem::path &resourceArgument,
              bool preview,
              bool includeConstructionCurves)
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

        const char *modelFileName = preview ? "lep-3d.xbf" : "lep-3d.step";
        const lep::NurbsWriteResult step = lep::writeNurbsStep(
            output / modelFileName, includeConstructionCurves);
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
            << (preview ? "Preview model: " : "STEP model: ")
            << pathToUtf8(output / modelFileName) << '\n';

        // Companion mesh for the Studio's Playground simulation; losing it
        // never fails the calculation.
        std::string simError;
        if (lep::writeSimMesh(output / "lep-sim.json", simError)) {
            std::cout << "Playground mesh: "
                      << pathToUtf8(output / "lep-sim.json") << '\n';
        } else {
            std::cerr << "Playground mesh warning: " << simError << '\n';
        }
        return result;
    } catch (const std::exception &exception) {
        std::cerr << "Engine error: " << exception.what() << '\n';
        return 2;
    }
}

// Flags are plain ASCII, so one comparison covers char and wchar_t argv.
template <typename StringView>
bool matchesFlag(StringView argument, std::string_view flag)
{
    if (argument.size() != flag.size()) {
        return false;
    }
    for (std::size_t index = 0; index < flag.size(); ++index) {
        if (argument[index]
            != static_cast<typename StringView::value_type>(flag[index])) {
            return false;
        }
    }
    return true;
}

std::filesystem::path argumentPath(std::wstring_view argument)
{
    return std::filesystem::path(argument);
}

std::filesystem::path argumentPath(std::string_view argument)
{
    return std::filesystem::u8path(std::string(argument));
}

template <typename StringView>
int runFromArguments(const std::vector<StringView> &arguments)
{
    bool preview = false;
    bool includeConstructionCurves = true;
    std::filesystem::path resourceDirectory;
    std::vector<std::filesystem::path> positional;
    for (std::size_t index = 0; index < arguments.size(); ++index) {
        const StringView argument = arguments[index];
        if (matchesFlag(argument, "--help") || matchesFlag(argument, "-h")) {
            printUsage();
            return arguments.size() == 1 ? 0 : 2;
        }
        if (matchesFlag(argument, "--preview")) {
            preview = true;
        } else if (matchesFlag(argument, "--no-construction-curves")) {
            includeConstructionCurves = false;
        } else if (matchesFlag(argument, "--resource-dir")) {
            if (index + 1 >= arguments.size()) {
                printUsage();
                return 2;
            }
            resourceDirectory = argumentPath(arguments[++index]);
        } else {
            positional.push_back(argumentPath(argument));
        }
    }
    if (positional.size() != 2) {
        printUsage();
        return 2;
    }
    return runEngine(positional[0],
                     positional[1],
                     resourceDirectory,
                     preview,
                     includeConstructionCurves);
}

} // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t *argv[])
{
    return runFromArguments(
        std::vector<std::wstring_view>(argv + 1, argv + argc));
}
#else
int main(int argc, char *argv[])
{
    return runFromArguments(
        std::vector<std::string_view>(argv + 1, argv + argc));
}
#endif
