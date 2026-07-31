#include "flat_capture.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace {

// Section 5.3's box grid, in drawing units before the scale factor: boxes are
// 1260 wide and 890.95 tall, box (1,1) spanning x [-630, 630] and y [-50,
// 840.95]. Row and column are 1-based and match the plan titles ("1-3
// EXTRADOS PANELS").
constexpr double boxWidth = 1260.0;
constexpr double boxHeight = 890.95;
constexpr double boxCentreY = 395.475;

// Drawing coordinates are centimetres times the drawing scale.
constexpr double centimetresToMillimetres = 10.0;

// Endpoint match tolerance when chaining segments into polylines, in
// millimetres. The core writes DXF coordinates with four decimals of a
// centimetre, so anything above that resolution is a genuine gap.
constexpr double chainTolerance = 0.02;

enum class Role
{
    Cut,
    Seam,
    Mark,
};

const char *roleName(Role role)
{
    switch (role) {
    case Role::Cut:
        return "cut";
    case Role::Seam:
        return "seam";
    case Role::Mark:
        break;
    }
    return "mark";
}

// Which drawing colour is the cut edge, per category. Every value here was
// measured against emitted geometry, not read off a comment — the Fortran's own
// labels are unreliable (dpanelc_ calls its colour-1 run "Sobreamples", i.e.
// allowances, when colour 3 is demonstrably the outer boundary).
//
// Ribs and panels draw both an outer cut edge (colour 3) and a stitch line 15 mm
// inside it (colour 1), matching section 19's "line-external / cutexternal" and
// "line-sewing / cutinternal" layer names. The smaller parts are each drawn as a
// single-colour outline with no allowance/stitch distinction at all, so their
// one colour is taken as the cut edge.
Role roleFor(const std::string &category, int color)
{
    struct CategoryColors
    {
        const char *category;
        int cut;
        int seam;
    };
    static const CategoryColors table[] = {
        {"rib", 3, 1},
        {"middle-rib", 3, 1},
        {"extrados-panel", 3, 1},
        {"intrados-panel", 3, 1},
        {"v-rib-type6", 1, -1},
        {"rod-pocket", 30, -1},
        {"nose-mylar", 10, -1},
    };

    int cut = 3;
    int seam = 1;
    for (const CategoryColors &entry : table) {
        if (category == entry.category) {
            cut = entry.cut;
            seam = entry.seam;
            break;
        }
    }
    if (color == cut) {
        return Role::Cut;
    }
    if (color == seam) {
        return Role::Seam;
    }
    return Role::Mark;
}

struct Segment
{
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    int color = 0;
};

struct Circle
{
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    int color = 0;
};

struct Text
{
    double x = 0.0;
    double y = 0.0;
    double height = 0.0;
    std::string value;
};

struct Part
{
    std::string category;
    int index = 0;
    int subIndex = 0;
    double originX = 0.0;
    double originY = 0.0;
    int boxRow = 0;
    int boxColumn = 0;
    std::vector<Segment> segments;
    std::vector<Circle> circles;
    std::vector<Text> texts;
    // Diagnostic: how much geometry the tagged scope drew into each box,
    // including boxes the declared-box filter rejected. Set LEP_FLAT_TALLY to
    // have it written out — it is how a new tag site's box gets identified,
    // since the drawing origins are buried in per-design separation constants.
    std::map<std::pair<int, int>, long> boxTally;
};

struct Point
{
    double x = 0.0;
    double y = 0.0;
};

struct Polyline
{
    Role role = Role::Mark;
    bool closed = false;
    std::vector<Point> points;
};

struct Collector
{
    double drawingScale = 1.0;
    std::string wingName;
    double flatArea = 0.0;
    double projectedArea = 0.0;
    std::vector<Part> parts;
    // Index into parts, or -1 while no part is open. Untagged drawing (box
    // frames, titles, planform views, line plans) is dropped on the floor.
    int current = -1;
    // Diagnostic: what got dropped, by box. Geometry landing in a box that
    // holds cuttable parts is a missing tag site — this is the check that a
    // category has not been overlooked.
    std::map<std::pair<int, int>, long> untagged;
    // Diagnostic detail: dropped geometry keyed by the last part that was open
    // and the drawing colour, which is what localises a gap to a code region —
    // drawing runs in program order, so "3178 colour-5 lines in box 1-2 after
    // rib-21 closed" names the pass that drew them. Written under
    // LEP_FLAT_TALLY.
    std::map<std::string, long> untaggedDetail;
    std::string lastPart;
};

Collector &collector()
{
    static Collector instance;
    return instance;
}

Part *openPart()
{
    Collector &state = collector();
    if (state.current < 0) {
        return nullptr;
    }
    return &state.parts[static_cast<std::size_t>(state.current)];
}

std::pair<int, int> boxAt(double scale, double x, double y);

// Records a dropped segment against the last part that was open, so a gap can
// be traced back to the drawing pass responsible.
void noteDropped(Collector &state, double x, double y, int color)
{
    const std::pair<int, int> box = boxAt(state.drawingScale, x, y);
    ++state.untagged[box];
    char key[192];
    std::snprintf(key,
                  sizeof(key),
                  "box %d-%d colour %d after %s",
                  box.first,
                  box.second,
                  color,
                  state.lastPart.empty() ? "(start)" : state.lastPart.c_str());
    ++state.untaggedDetail[key];
}

// Which plan box a point falls in, by nearest box centre. Used for the
// diagnostic tally and to label a part with the plan page it came from.
std::pair<int, int> boxAt(double scale, double x, double y)
{
    const int column = static_cast<int>(std::lround(x / scale / boxWidth)) + 1;
    const int row =
        static_cast<int>(std::lround((y / scale - boxCentreY) / boxHeight)) + 1;
    return {row, column};
}

// Geometry belongs to the open part when it falls in a window around the part's
// declared origin. A window rather than the part's plan box, because several
// categories straddle a box boundary — V-ribs are laid out at
// `3300 + xrsep*i`, which walks out of the box as the rib index climbs, so
// box-filtering silently dropped the outermost ones.
//
// The bounds are what separates a part from the duplicate copies of itself that
// the same loop iteration draws elsewhere: the cutting-table copy sits 2520
// units away in x and the washin reference copy 890.95 away in y, so anything
// inside one box width horizontally and half a box height vertically is the
// part itself and nothing else.
constexpr double captureWindowX = boxWidth;
constexpr double captureWindowY = boxHeight * 0.5;

bool insideCaptureWindow(Part &part, double scale, double x, double y)
{
    ++part.boxTally[boxAt(scale, x, y)];
    const double dx = std::abs(x - part.originX) / scale;
    const double dy = std::abs(y - part.originY) / scale;
    return dx <= captureWindowX && dy <= captureWindowY;
}

// Part-local millimetres, y-up. The line_() argument frame is y-down (line_
// negates y on its way into the DXF), so the origin shift also flips it.
Point toLocal(const Part &part, double scale, double x, double y)
{
    const double factor = centimetresToMillimetres / scale;
    return Point{(x - part.originX) * factor, (part.originY - y) * factor};
}

// Chains segments sharing endpoints into polylines. Segment soup is what the
// core emits; the consumers (nesting, PDF stroking) all want runs.
std::vector<Polyline> chain(const std::vector<Segment> &segments,
                            const Part &part,
                            double scale)
{
    struct Edge
    {
        Point a;
        Point b;
        Role role = Role::Mark;
        bool used = false;
    };

    std::vector<Edge> edges;
    edges.reserve(segments.size());
    for (const Segment &segment : segments) {
        Edge edge;
        edge.a = toLocal(part, scale, segment.x1, segment.y1);
        edge.b = toLocal(part, scale, segment.x2, segment.y2);
        edge.role = roleFor(part.category, segment.color);
        const double dx = edge.b.x - edge.a.x;
        const double dy = edge.b.y - edge.a.y;
        if (std::sqrt(dx * dx + dy * dy) <= chainTolerance) {
            continue;
        }
        edges.push_back(edge);
    }

    // Spatial hash on the tolerance grid so endpoint lookup stays linear. A
    // point is matched against its own cell and the eight neighbours, which
    // covers coordinates that quantise either side of a cell boundary.
    const double cell = chainTolerance;
    std::map<std::pair<long, long>, std::vector<std::pair<std::size_t, int>>>
        buckets;
    const auto key = [cell](const Point &point) {
        return std::make_pair(static_cast<long>(std::floor(point.x / cell)),
                              static_cast<long>(std::floor(point.y / cell)));
    };
    for (std::size_t i = 0; i < edges.size(); ++i) {
        buckets[key(edges[i].a)].emplace_back(i, 0);
        buckets[key(edges[i].b)].emplace_back(i, 1);
    }

    const auto near = [](const Point &a, const Point &b) {
        return std::abs(a.x - b.x) <= chainTolerance
            && std::abs(a.y - b.y) <= chainTolerance;
    };

    // Finds an unused edge of the same role touching `point`, other than
    // `skip`; returns its index and the far endpoint.
    const auto findNext = [&](const Point &point,
                              Role role,
                              std::size_t skip,
                              std::size_t *found,
                              Point *far) {
        const std::pair<long, long> home = key(point);
        for (long dx = -1; dx <= 1; ++dx) {
            for (long dy = -1; dy <= 1; ++dy) {
                const auto bucket =
                    buckets.find({home.first + dx, home.second + dy});
                if (bucket == buckets.end()) {
                    continue;
                }
                for (const auto &entry : bucket->second) {
                    const std::size_t index = entry.first;
                    if (index == skip || edges[index].used
                        || edges[index].role != role) {
                        continue;
                    }
                    const Point &end = entry.second == 0 ? edges[index].a
                                                         : edges[index].b;
                    if (!near(end, point)) {
                        continue;
                    }
                    *found = index;
                    *far = entry.second == 0 ? edges[index].b : edges[index].a;
                    return true;
                }
            }
        }
        return false;
    };

    std::vector<Polyline> polylines;
    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (edges[i].used) {
            continue;
        }
        edges[i].used = true;

        std::vector<Point> forward{edges[i].a, edges[i].b};
        std::size_t next = 0;
        Point far;
        while (findNext(forward.back(), edges[i].role, i, &next, &far)) {
            edges[next].used = true;
            forward.push_back(far);
        }
        std::vector<Point> backward;
        Point head = edges[i].a;
        while (findNext(head, edges[i].role, i, &next, &far)) {
            edges[next].used = true;
            backward.push_back(far);
            head = far;
        }

        Polyline polyline;
        polyline.role = edges[i].role;
        polyline.points.reserve(forward.size() + backward.size());
        polyline.points.assign(backward.rbegin(), backward.rend());
        polyline.points.insert(
            polyline.points.end(), forward.begin(), forward.end());
        if (polyline.points.size() > 2
            && near(polyline.points.front(), polyline.points.back())) {
            polyline.closed = true;
            polyline.points.pop_back();
        }
        polylines.push_back(std::move(polyline));
    }
    return polylines;
}

struct Bounds
{
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    bool empty = true;

    void add(const Point &point)
    {
        if (empty) {
            minX = maxX = point.x;
            minY = maxY = point.y;
            empty = false;
            return;
        }
        minX = std::min(minX, point.x);
        minY = std::min(minY, point.y);
        maxX = std::max(maxX, point.x);
        maxY = std::max(maxY, point.y);
    }

    bool contains(const Point &point, double slack) const
    {
        return !empty && point.x >= minX - slack && point.x <= maxX + slack
            && point.y >= minY - slack && point.y <= maxY + slack;
    }

    // Overlapping by more than `slack` in both axes. Two clusters that share
    // space are the same piece drawn in disconnected strokes; genuinely
    // separate pieces are laid out with a gap between them.
    bool overlaps(const Bounds &other, double slack) const
    {
        return !empty && !other.empty && minX < other.maxX - slack
            && other.minX < maxX - slack && minY < other.maxY - slack
            && other.minY < maxY - slack;
    }

    void absorb(const Bounds &other)
    {
        if (other.empty) {
            return;
        }
        if (empty) {
            *this = other;
            return;
        }
        minX = std::min(minX, other.minX);
        minY = std::min(minY, other.minY);
        maxX = std::max(maxX, other.maxX);
        maxY = std::max(maxY, other.maxY);
    }

    double area() const
    {
        return empty ? 0.0 : (maxX - minX) * (maxY - minY);
    }
};

// One physically separate piece of fabric: what the nester places and the
// printer draws. A tagged part is not always one piece — panels longer than
// the fabric are cut chordwise into two or three, drawn a fixed gap apart in
// the same box.
struct Piece
{
    std::vector<Polyline> polylines;
    std::vector<Circle> circles;
    std::vector<Text> texts;
    Bounds bounds;
};

class DisjointSet
{
public:
    explicit DisjointSet(std::size_t count) : parent_(count)
    {
        for (std::size_t i = 0; i < count; ++i) {
            parent_[i] = i;
        }
    }

    std::size_t find(std::size_t item)
    {
        while (parent_[item] != item) {
            parent_[item] = parent_[parent_[item]];
            item = parent_[item];
        }
        return item;
    }

    void merge(std::size_t a, std::size_t b)
    {
        const std::size_t rootA = find(a);
        const std::size_t rootB = find(b);
        if (rootA != rootB) {
            parent_[rootB] = rootA;
        }
    }

private:
    std::vector<std::size_t> parent_;
};

// Splits a part's polylines into pieces. Polylines that touch end-to-end are
// the same piece; a group whose bounding box sits wholly inside a larger
// group's is not a separate piece but something drawn on it — a vent opening,
// a reinforcement outline — and is folded back in.
std::vector<Piece> splitIntoPieces(const std::vector<Polyline> &polylines,
                                   const std::vector<Circle> &circles,
                                   const std::vector<Text> &texts,
                                   const Part &part,
                                   double scale)
{
    DisjointSet groups(polylines.size());

    // Endpoint index: two polylines belong together when an end of one meets
    // an end of the other, which is how the corner segments tie a panel's
    // allowance edges to its stitch lines.
    const double cell = chainTolerance;
    std::map<std::pair<long, long>, std::vector<std::size_t>> ends;
    const auto key = [cell](const Point &point) {
        return std::make_pair(static_cast<long>(std::floor(point.x / cell)),
                              static_cast<long>(std::floor(point.y / cell)));
    };
    const auto record = [&](const Point &point, std::size_t index) {
        const std::pair<long, long> home = key(point);
        for (long dx = -1; dx <= 1; ++dx) {
            for (long dy = -1; dy <= 1; ++dy) {
                const auto bucket =
                    ends.find({home.first + dx, home.second + dy});
                if (bucket == ends.end()) {
                    continue;
                }
                for (const std::size_t other : bucket->second) {
                    groups.merge(index, other);
                }
            }
        }
        ends[home].push_back(index);
    };
    for (std::size_t i = 0; i < polylines.size(); ++i) {
        if (polylines[i].points.empty()) {
            continue;
        }
        record(polylines[i].points.front(), i);
        if (!polylines[i].closed) {
            record(polylines[i].points.back(), i);
        }
    }

    std::map<std::size_t, std::size_t> rootToPiece;
    std::vector<Piece> pieces;
    for (std::size_t i = 0; i < polylines.size(); ++i) {
        if (polylines[i].points.empty()) {
            continue;
        }
        const std::size_t root = groups.find(i);
        auto slot = rootToPiece.find(root);
        if (slot == rootToPiece.end()) {
            slot = rootToPiece.emplace(root, pieces.size()).first;
            pieces.emplace_back();
        }
        Piece &piece = pieces[slot->second];
        piece.polylines.push_back(polylines[i]);
        for (const Point &point : polylines[i].points) {
            piece.bounds.add(point);
        }
    }

    // Merge clusters that share space. Endpoint connectivity alone leaves a
    // panel in fragments — dpanelcc_ omits the corner segments that would tie
    // its allowance edges to its stitch lines whenever a leading or trailing
    // edge is suppressed, and vent openings and equidistant marks are drawn as
    // free-standing strokes. What genuinely separates two pieces is the gap the
    // plan leaves between them, so overlap is the test. Repeated to a fixed
    // point because merging grows the bounding box.
    std::vector<bool> folded(pieces.size(), false);
    const double overlapSlack = 1.0;
    for (bool merged = true; merged;) {
        merged = false;
        for (std::size_t host = 0; host < pieces.size(); ++host) {
            if (folded[host]) {
                continue;
            }
            for (std::size_t guest = host + 1; guest < pieces.size();
                 ++guest) {
                if (folded[guest]
                    || !pieces[host].bounds.overlaps(pieces[guest].bounds,
                                                     overlapSlack)) {
                    continue;
                }
                folded[guest] = true;
                merged = true;
                pieces[host].polylines.insert(pieces[host].polylines.end(),
                                              pieces[guest].polylines.begin(),
                                              pieces[guest].polylines.end());
                pieces[host].bounds.absorb(pieces[guest].bounds);
            }
        }
    }

    std::vector<Piece> kept;
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        if (!folded[i]) {
            kept.push_back(std::move(pieces[i]));
        }
    }

    // Strokes that end up beside a piece rather than on it — index marks drawn
    // just off the outline — are not pieces of fabric. Anything far smaller
    // than the largest cluster joins its nearest neighbour.
    if (kept.size() > 1) {
        double largest = 0.0;
        for (const Piece &piece : kept) {
            largest = std::max(largest, piece.bounds.area());
        }
        std::vector<Piece> substantial;
        std::vector<Piece> strays;
        for (Piece &piece : kept) {
            if (piece.bounds.area() >= largest * 0.01) {
                substantial.push_back(std::move(piece));
            } else {
                strays.push_back(std::move(piece));
            }
        }
        for (Piece &stray : strays) {
            Piece *best = nullptr;
            double bestDistance = 0.0;
            for (Piece &piece : substantial) {
                const double dx = std::max({piece.bounds.minX - stray.bounds.maxX,
                                            0.0,
                                            stray.bounds.minX - piece.bounds.maxX});
                const double dy = std::max({piece.bounds.minY - stray.bounds.maxY,
                                            0.0,
                                            stray.bounds.minY - piece.bounds.maxY});
                const double distance = dx * dx + dy * dy;
                if (best == nullptr || distance < bestDistance) {
                    best = &piece;
                    bestDistance = distance;
                }
            }
            // Only adopt a stray that is actually touching its host. Absorbing a
            // remote one stretches the piece's bounds out to reach it, which
            // silently turns a 500 mm panel into a 9 m one — the bounds are what
            // the nester packs against and what the part list reports, so the
            // damage is invisible until something measures it. A stray this far
            // from every piece is not part geometry; drop it.
            const double reach = 50.0;
            if (best != nullptr && bestDistance <= reach * reach) {
                best->polylines.insert(best->polylines.end(),
                                       stray.polylines.begin(),
                                       stray.polylines.end());
                best->bounds.absorb(stray.bounds);
            }
        }
        kept = std::move(substantial);
    }

    // Loose marks — punch circles, index labels — land on whichever piece
    // covers them, or the closest one when they sit just outside the outline.
    const auto host = [&kept](const Point &point) -> Piece * {
        Piece *best = nullptr;
        double bestDistance = 0.0;
        for (Piece &piece : kept) {
            if (piece.bounds.contains(point, 0.0)) {
                return &piece;
            }
            const double dx = std::max({piece.bounds.minX - point.x,
                                        0.0,
                                        point.x - piece.bounds.maxX});
            const double dy = std::max({piece.bounds.minY - point.y,
                                        0.0,
                                        point.y - piece.bounds.maxY});
            const double distance = dx * dx + dy * dy;
            if (best == nullptr || distance < bestDistance) {
                best = &piece;
                bestDistance = distance;
            }
        }
        return best;
    };

    const double factor = centimetresToMillimetres / scale;
    for (const Circle &circle : circles) {
        const Point centre = toLocal(part, scale, circle.x, circle.y);
        if (Piece *piece = host(centre)) {
            piece->circles.push_back(
                Circle{centre.x, centre.y, circle.radius * factor,
                       circle.color});
        }
    }
    for (const Text &text : texts) {
        const Point anchor = toLocal(part, scale, text.x, text.y);
        if (Piece *piece = host(anchor)) {
            piece->texts.push_back(
                Text{anchor.x, anchor.y, text.height * factor, text.value});
        }
    }

    // Leading edge first: the plan draws a split panel's pieces down the box
    // in chord order, so the highest piece is the one nearest the nose.
    std::sort(kept.begin(), kept.end(), [](const Piece &a, const Piece &b) {
        return a.bounds.maxY > b.bounds.maxY;
    });

    // Re-origin every piece to its own bottom-left so the nester can treat
    // each one as a standalone shape.
    for (Piece &piece : kept) {
        const double shiftX = piece.bounds.minX;
        const double shiftY = piece.bounds.minY;
        for (Polyline &polyline : piece.polylines) {
            for (Point &point : polyline.points) {
                point.x -= shiftX;
                point.y -= shiftY;
            }
        }
        for (Circle &circle : piece.circles) {
            circle.x -= shiftX;
            circle.y -= shiftY;
        }
        for (Text &text : piece.texts) {
            text.x -= shiftX;
            text.y -= shiftY;
        }
        piece.bounds.maxX -= shiftX;
        piece.bounds.maxY -= shiftY;
        piece.bounds.minX = 0.0;
        piece.bounds.minY = 0.0;
    }
    return kept;
}

void writeNumber(std::ostream &out, double value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.4g", value);
    out << buffer;
}

void writeString(std::ostream &out, const std::string &value)
{
    out << '"';
    for (const char character : value) {
        switch (character) {
        case '"':
            out << "\\\"";
            break;
        case '\\':
            out << "\\\\";
            break;
        case '\n':
            out << "\\n";
            break;
        default:
            if (static_cast<unsigned char>(character) < 0x20) {
                char escape[8];
                std::snprintf(escape,
                              sizeof(escape),
                              "\\u%04x",
                              static_cast<unsigned char>(character));
                out << escape;
            } else {
                out << character;
            }
            break;
        }
    }
    out << '"';
}

std::string trimmed(const char *text, int length)
{
    if (text == nullptr || length <= 0) {
        return std::string();
    }
    int end = length;
    while (end > 0 && (text[end - 1] == ' ' || text[end - 1] == '\0')) {
        --end;
    }
    int begin = 0;
    while (begin < end && text[begin] == ' ') {
        ++begin;
    }
    return std::string(text + begin, static_cast<std::size_t>(end - begin));
}

} // namespace

void lep_flat_set_design(double drawingScale,
                         const char *wingName,
                         int wingNameLength)
{
    Collector &state = collector();
    state.drawingScale = drawingScale > 0.0 ? drawingScale : 1.0;
    state.wingName = trimmed(wingName, wingNameLength);
}

void lep_flat_set_areas(double flatArea, double projectedArea)
{
    Collector &state = collector();
    state.flatArea = flatArea;
    state.projectedArea = projectedArea;
}

void lep_flat_begin_part(const char *category,
                         int categoryLength,
                         int index,
                         int subIndex,
                         double originX,
                         double originY,
                         int boxRow,
                         int boxColumn)
{
    Collector &state = collector();
    const std::string name = trimmed(category, categoryLength);

    // Re-opening a part that already exists resumes it. Drawing for one part is
    // not always contiguous — a rib's rod pockets and nose mylars are drawn
    // into box 1-7 midway through the rib's own loop iteration, and section
    // 11.4 comes back later to add equidistant marks to panels already drawn —
    // so tags interleave freely rather than each one starting a new part.
    for (std::size_t i = 0; i < state.parts.size(); ++i) {
        const Part &existing = state.parts[i];
        if (existing.category == name && existing.index == index
            && existing.subIndex == subIndex && existing.boxRow == boxRow
            && existing.boxColumn == boxColumn) {
            state.current = static_cast<int>(i);
            return;
        }
    }

    Part part;
    part.category = name;
    part.index = index;
    part.subIndex = subIndex;
    part.originX = originX;
    part.originY = originY;
    part.boxRow = boxRow;
    part.boxColumn = boxColumn;
    state.parts.push_back(std::move(part));
    state.current = static_cast<int>(state.parts.size()) - 1;
    char label[160];
    std::snprintf(label, sizeof(label), "%s-%d", name.c_str(), index);
    state.lastPart = label;
}

void lep_flat_end_part(void)
{
    collector().current = -1;
}

void lep_flat_resume_at(const char *category,
                        int categoryLength,
                        double originX,
                        double originY)
{
    Collector &state = collector();
    const std::string name = trimmed(category, categoryLength);
    // The layout slot is recomputed identically by each pass, so an exact hit is
    // expected; the tolerance only absorbs float/double round-tripping.
    const double tolerance = 0.01 * state.drawingScale;
    state.current = -1;
    for (std::size_t i = 0; i < state.parts.size(); ++i) {
        const Part &part = state.parts[i];
        if (part.category == name
            && std::abs(part.originX - originX) <= tolerance
            && std::abs(part.originY - originY) <= tolerance) {
            state.current = static_cast<int>(i);
            char label[160];
            std::snprintf(
                label, sizeof(label), "%s-%d", name.c_str(), part.index);
            state.lastPart = label;
            return;
        }
    }
}

void lep_flat_capture_line(double x1, double y1, double x2, double y2,
                           int color)
{
    Part *part = openPart();
    if (part == nullptr) {
        Collector &state = collector();
        noteDropped(state, x1, y1, color);
        return;
    }
    Collector &state = collector();
    const double scale = state.drawingScale;
    // Both endpoints must be in the window: a segment that bridges out to a
    // reference copy is not part geometry. Rejections count as untagged too —
    // otherwise leaving a part open across a later drawing pass would hide that
    // pass's geometry from the missing-tag-site diagnostic instead of the
    // diagnostic reporting it.
    if (!insideCaptureWindow(*part, scale, x1, y1)
        || !insideCaptureWindow(*part, scale, x2, y2)) {
        noteDropped(state, x1, y1, color);
        return;
    }
    part->segments.push_back(Segment{x1, y1, x2, y2, color});
}

void lep_flat_capture_circle(double x, double y, double radius, int color)
{
    Part *part = openPart();
    if (part == nullptr) {
        return;
    }
    if (!insideCaptureWindow(*part, collector().drawingScale, x, y)) {
        return;
    }
    part->circles.push_back(Circle{x, y, radius, color});
}

void lep_flat_capture_text(double x,
                           double y,
                           double height,
                           const char *text,
                           int textLength)
{
    Part *part = openPart();
    if (part == nullptr) {
        return;
    }
    if (!insideCaptureWindow(*part, collector().drawingScale, x, y)) {
        return;
    }
    part->texts.push_back(Text{x, y, height, trimmed(text, textLength)});
}

void lep_flat_capture_integer_text(double x,
                                   double y,
                                   double height,
                                   int value)
{
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%d", value);
    lep_flat_capture_text(
        x, y, height, buffer, static_cast<int>(std::strlen(buffer)));
}

void lep_flat_write(const char *path)
{
    Collector &state = collector();
    std::ofstream out(path);
    if (!out) {
        return;
    }

    const double scale = state.drawingScale;
    const double factor = centimetresToMillimetres / scale;

    out << "{\n";
    out << "  \"schema\": 1,\n";
    out << "  \"units\": \"mm\",\n";
    out << "  \"drawingScale\": ";
    writeNumber(out, scale);
    out << ",\n";
    out << "  \"wing\": ";
    writeString(out, state.wingName);
    out << ",\n";
    out << "  \"flatArea\": ";
    writeNumber(out, state.flatArea);
    out << ",\n";
    out << "  \"projectedArea\": ";
    writeNumber(out, state.projectedArea);
    out << ",\n";
    if (std::getenv("LEP_FLAT_TALLY") != nullptr) {
        out << "  \"untaggedDetail\": {";
        bool firstDetail = true;
        for (const auto &entry : state.untaggedDetail) {
            if (entry.second < 20) {
                continue;
            }
            if (!firstDetail) {
                out << ", ";
            }
            firstDetail = false;
            writeString(out, entry.first);
            out << ": " << entry.second;
        }
        out << "},\n";
        out << "  \"untagged\": {";
        bool firstUntagged = true;
        for (const auto &entry : state.untagged) {
            if (!firstUntagged) {
                out << ", ";
            }
            firstUntagged = false;
            char box[16];
            std::snprintf(box,
                          sizeof(box),
                          "%d-%d",
                          entry.first.first,
                          entry.first.second);
            writeString(out, box);
            out << ": " << entry.second;
        }
        out << "},\n";
        out << "  \"boxTally\": {\n";
        bool firstTally = true;
        for (const Part &part : state.parts) {
            if (part.boxTally.empty()) {
                continue;
            }
            if (!firstTally) {
                out << ",\n";
            }
            firstTally = false;
            char label[128];
            std::snprintf(label,
                          sizeof(label),
                          "%s-%d",
                          part.category.c_str(),
                          part.index);
            out << "    ";
            writeString(out, label);
            out << ": {";
            bool firstBox = true;
            for (const auto &entry : part.boxTally) {
                if (!firstBox) {
                    out << ", ";
                }
                firstBox = false;
                char box[16];
                std::snprintf(box,
                              sizeof(box),
                              "%d-%d",
                              entry.first.first,
                              entry.first.second);
                writeString(out, box);
                out << ": " << entry.second;
            }
            out << "}";
        }
        out << "\n  },\n";
    }
    out << "  \"parts\": [\n";

    bool firstPart = true;
    for (const Part &part : state.parts) {
        if (part.segments.empty()) {
            continue;
        }
        const std::vector<Polyline> polylines =
            chain(part.segments, part, scale);
        const std::vector<Piece> pieces = splitIntoPieces(
            polylines, part.circles, part.texts, part, scale);

        for (std::size_t p = 0; p < pieces.size(); ++p) {
            const Piece &piece = pieces[p];
            // Two independent levels of subdivision. `subIndex` comes from the
            // tag: one V-rib record draws several distinct strips (a type-2
            // draws its 1-2 and 2-3 strips from separate blocks). `piece` is
            // what the splitter found inside one tagged scope, which is how a
            // chordwise-cut panel divides. Either can be absent, and the id
            // only carries the levels that apply.
            const int pieceNumber =
                pieces.size() > 1 ? static_cast<int>(p) + 1 : 0;

            if (!firstPart) {
                out << ",\n";
            }
            firstPart = false;

            char id[160];
            int written = std::snprintf(
                id, sizeof(id), "%s-%d", part.category.c_str(), part.index);
            if (part.subIndex > 0) {
                written += std::snprintf(id + written,
                                         sizeof(id) - written,
                                         "-%d",
                                         part.subIndex);
            }
            if (pieceNumber > 0) {
                std::snprintf(
                    id + written, sizeof(id) - written, "-%d", pieceNumber);
            }

            out << "    {\n";
            out << "      \"id\": ";
            writeString(out, id);
            out << ",\n";
            out << "      \"category\": ";
            writeString(out, part.category);
            out << ",\n";
            out << "      \"index\": " << part.index << ",\n";
            out << "      \"subIndex\": " << part.subIndex << ",\n";
            out << "      \"piece\": " << pieceNumber << ",\n";
            out << "      \"pieceCount\": " << pieces.size() << ",\n";
            char box[16];
            std::snprintf(
                box, sizeof(box), "%d-%d", part.boxRow, part.boxColumn);
            out << "      \"box\": ";
            writeString(out, box);
            out << ",\n";
            out << "      \"width\": ";
            writeNumber(out, piece.bounds.maxX);
            out << ",\n";
            out << "      \"height\": ";
            writeNumber(out, piece.bounds.maxY);
            out << ",\n";
            // The plan orientation is NOT a statement about fabric grain — the
            // core lays parts out to fill its drawing boxes and never records
            // which way the weave should run. This is emitted so consumers have
            // somewhere to read a real value from later, and tagged "assumed"
            // so nothing mistakes the placeholder for data. See
            // docs/flat-part-orientation.md.
            out << "      \"grainAngleDeg\": 90,\n";
            out << "      \"grainSource\": \"assumed-from-plan-layout\",\n";

            out << "      \"polylines\": [\n";
            bool firstPolyline = true;
            for (const Polyline &polyline : piece.polylines) {
                if (polyline.points.size() < 2) {
                    continue;
                }
                if (!firstPolyline) {
                    out << ",\n";
                }
                firstPolyline = false;
                out << "        {\"role\": \"" << roleName(polyline.role)
                    << "\", \"closed\": "
                    << (polyline.closed ? "true" : "false") << ", \"pts\": [";
                bool firstPoint = true;
                for (const Point &point : polyline.points) {
                    if (!firstPoint) {
                        out << ", ";
                    }
                    firstPoint = false;
                    out << '[';
                    writeNumber(out, point.x);
                    out << ", ";
                    writeNumber(out, point.y);
                    out << ']';
                }
                out << "]}";
            }
            out << "\n      ],\n";

            out << "      \"circles\": [";
            bool firstCircle = true;
            for (const Circle &circle : piece.circles) {
                if (!firstCircle) {
                    out << ", ";
                }
                firstCircle = false;
                out << "{\"x\": ";
                writeNumber(out, circle.x);
                out << ", \"y\": ";
                writeNumber(out, circle.y);
                out << ", \"r\": ";
                writeNumber(out, circle.radius);
                out << '}';
            }
            out << "],\n";

            out << "      \"texts\": [";
            bool firstText = true;
            for (const Text &text : piece.texts) {
                if (!firstText) {
                    out << ", ";
                }
                firstText = false;
                out << "{\"x\": ";
                writeNumber(out, text.x);
                out << ", \"y\": ";
                writeNumber(out, text.y);
                out << ", \"h\": ";
                writeNumber(out, text.height);
                out << ", \"s\": ";
                writeString(out, text.value);
                out << '}';
            }
            out << "]\n";
            out << "    }";
        }
    }

    out << "\n  ]\n}\n";
}
