#pragma once

#include "softwing/soft_body.h"

#include <cstddef>
#include <span>

namespace softwing {

// Geometric volume of one closed canopy bay. The input skin is intentionally
// open at its bounding ribs; those openings are closed by virtual polygon
// caps for this calculation only. No triangles or constraints are added to the
// SoftBody.
struct ClosedCellVolumeEstimate {
    double signedVolume = 0.0;
    double volume = 0.0;

    std::size_t skinBoundaryEdges = 0;
    std::size_t capBoundaryEdgesMatched = 0;
    std::size_t nonManifoldSkinEdges = 0;
    std::size_t degenerateSkinFaces = 0;

    // True when all references and both cap polygons were usable and a finite
    // non-zero enclosed volume was obtained. A useful estimate can be valid
    // even when watertightAfterCapping is false; in that case the geometric
    // orientation fallback was needed because skin and rib loops do not share
    // node indices.
    bool valid = false;
    bool watertightAfterCapping = false;
    bool usedGeometricOrientationFallback = false;
    bool ambiguousSkinOrientation = false;
};

// skinFaceIndices selects the body's triangles assigned to this bay.
// lowRibLoop and highRibLoop are ordered closed polygon loops (the first node
// is not repeated). Loop winding is not prescribed: shared boundary edges are
// used first, with a geometric outward-normal fallback when topology cannot
// establish the cap orientation.
[[nodiscard]] ClosedCellVolumeEstimate estimateClosedCellVolume(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::span<const std::size_t> skinFaceIndices,
    std::span<const std::size_t> lowRibLoop,
    std::span<const std::size_t> highRibLoop);

} // namespace softwing
