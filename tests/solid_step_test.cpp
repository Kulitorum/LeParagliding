#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS.hxx>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

std::string pathToUtf8(const std::filesystem::path &path)
{
    const auto encoded = path.u8string();
    return {reinterpret_cast<const char *>(encoded.data()), encoded.size()};
}

void reportInvalidShapes(const BRepCheck_Analyzer &analyzer,
                         const TopoDS_Shape &solid)
{
    const auto report = [&analyzer](const TopoDS_Shape &shape,
                                    const char *kind) {
        const occ::handle<BRepCheck_Result> result = analyzer.Result(shape);
        if (result.IsNull()) {
            return;
        }
        for (const BRepCheck_Status status : result->Status()) {
            if (status != BRepCheck_NoError) {
                std::cerr << ' ' << kind << ':'
                          << static_cast<int>(status);
            }
        }
    };
    report(solid, "solid");
    int faceIndex = 0;
    for (TopExp_Explorer explorer(solid, TopAbs_FACE);
         explorer.More();
         explorer.Next()) {
        ++faceIndex;
        const occ::handle<BRepCheck_Result> result =
            analyzer.Result(explorer.Current());
        bool invalid = false;
        if (!result.IsNull()) {
            for (const BRepCheck_Status status : result->Status()) {
                invalid = invalid || status != BRepCheck_NoError;
            }
        }
        if (invalid) {
            Bnd_Box bounds;
            BRepBndLib::Add(explorer.Current(), bounds);
            double xMin = 0.0;
            double yMin = 0.0;
            double zMin = 0.0;
            double xMax = 0.0;
            double yMax = 0.0;
            double zMax = 0.0;
            bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            std::cerr << " face#" << faceIndex << " bounds=("
                      << xMin << ',' << yMin << ',' << zMin << ")..("
                      << xMax << ',' << yMax << ',' << zMax << ')';
        }
    }
    for (const auto &[type, name] :
         std::vector<std::pair<TopAbs_ShapeEnum, const char *>>{
             {TopAbs_SHELL, "shell"},
             {TopAbs_FACE, "face"},
             {TopAbs_WIRE, "wire"},
             {TopAbs_EDGE, "edge"},
             {TopAbs_VERTEX, "vertex"}}) {
        for (TopExp_Explorer explorer(solid, type);
             explorer.More();
             explorer.Next()) {
            report(explorer.Current(), name);
        }
    }
    std::cerr << '\n';
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: solid-step-test <step-file>\n";
        return 2;
    }

    const std::filesystem::path path = std::filesystem::u8path(argv[1]);
    STEPControl_Reader reader;
    if (reader.ReadFile(pathToUtf8(path).c_str()) != IFSelect_RetDone) {
        std::cerr << "Could not read CFD STEP file: " << pathToUtf8(path)
                  << '\n';
        return 2;
    }
    if (reader.TransferRoots() == 0) {
        std::cerr << "CFD STEP file contains no transferable roots.\n";
        return 2;
    }

    const TopoDS_Shape imported = reader.OneShape();
    if (imported.IsNull()) {
        std::cerr << "CFD STEP import produced no shape.\n";
        return 2;
    }

    std::vector<TopoDS_Solid> solids;
    if (imported.ShapeType() == TopAbs_SOLID) {
        solids.push_back(TopoDS::Solid(imported));
    } else {
        for (TopExp_Explorer explorer(imported, TopAbs_SOLID);
             explorer.More();
             explorer.Next()) {
            solids.push_back(TopoDS::Solid(explorer.Current()));
        }
    }
    if (solids.size() != 1) {
        std::cerr << "CFD STEP file contains " << solids.size()
                  << " solids instead of one.\n";
        return 2;
    }

    const TopoDS_Solid &solid = solids.front();
    const BRepCheck_Analyzer analyzer(solid, true);
    if (!analyzer.IsValid()) {
        std::cerr << "CFD STEP solid is topologically invalid after import.\n";
        reportInvalidShapes(analyzer, solid);
        return 2;
    }

    int shellCount = 0;
    for (TopExp_Explorer explorer(solid, TopAbs_SHELL);
         explorer.More();
         explorer.Next()) {
        ++shellCount;
        const TopoDS_Shell shell = TopoDS::Shell(explorer.Current());
        if (!BRep_Tool::IsClosed(shell)) {
            std::cerr << "CFD STEP solid contains an open shell.\n";
            return 2;
        }
    }
    if (shellCount != 1) {
        std::cerr << "CFD STEP solid contains " << shellCount
                  << " shells instead of one exterior shell.\n";
        return 2;
    }

    int faceCount = 0;
    for (TopExp_Explorer explorer(solid, TopAbs_FACE);
         explorer.More();
         explorer.Next()) {
        ++faceCount;
    }
    if (faceCount == 0) {
        std::cerr << "CFD STEP solid contains no faces.\n";
        return 2;
    }

    std::cout << "Validated one closed CFD solid with " << faceCount
              << " exterior faces.\n";
    return 0;
}
