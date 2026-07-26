#include "section_specs.h"

#include <QHash>

// Populated from the 3.28 reference design (tests/fixtures/3.28), the
// shipped presets (formats 2.60-3.28) and the translated engine reader.
// Headers describe the section's dominant table and apply by column index;
// rows with other shapes (scalar records, group headers) still display
// under them, and the structure line explains the record layout. Sections
// not listed fall back to the generic numbered grid.

namespace {

SectionColumnSpec column(const char *header, const char *tooltip)
{
    return {QString::fromUtf8(header), QString::fromUtf8(tooltip)};
}

QHash<int, SectionSpec> buildSpecs()
{
    QHash<int, SectionSpec> specs;

    {
        SectionSpec s;
        s.columns = {
            column("Rib", "Rib number. Not required to be sequential; the "
                          "engine reads rows in order."),
            column("Airfoil file", "Profile file name, relative to the "
                                   "design file."),
            column("Intake in (%)", "Air intake start, % of chord from the "
                                    "leading edge."),
            column("Intake out (%)", "Air intake end, % of chord."),
            column("Open (0/1)", "1 = open cell (inlet), 0 = closed cell."),
            column("Displacement", "Airfoil displacement parameter."),
            column("Rib weight", "Rib reinforcement width parameter."),
            column("Minirib", "Minirib / rrw parameter (later formats)."),
        };
        s.structure = QStringLiteral(
            "One row per half-wing rib, in order from the centre out.");
        s.onePerRib = true;
        specs.insert(2, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Airfoil", "Rib the anchors apply to."),
            column("Anchors", "How many of the A-E anchors the engine "
                              "reads on this row: with 3, only A, B and C "
                              "are used and D/E are ignored. The brake "
                              "column is separate."),
            column("A (%)", "A-line anchor, % of chord from the leading "
                            "edge."),
            column("B (%)", "B-line anchor, % of chord. Used when "
                            "Anchors ≥ 2."),
            column("C (%)", "C-line anchor, % of chord. Used when "
                            "Anchors ≥ 3."),
            column("D (%)", "D-line anchor, % of chord. Only used when "
                            "Anchors ≥ 4 — otherwise the engine "
                            "ignores this value."),
            column("E (%)", "E-line anchor, % of chord. Only used when "
                            "Anchors ≥ 5."),
            column("F (Brake)", "Brake attachment, % of chord. Independent "
                                "of the Anchors count: active whenever "
                                "non-zero (0 disables it)."),
        };
        s.structure = QStringLiteral(
            "One row per half-wing rib. The Anchors count gates columns "
            "A-E (a D value on an Anchors=3 row is ignored); the brake "
            "column is always its own anchor. The wing is mirrored, so "
            "every change shows on both halves.");
        s.onePerRib = true;
        for (int c = 2; c <= 6; ++c) {
            SectionCurveColumn anchor{c, 0.0, 100.0, 2};
            anchor.gateColumn = 1;
            anchor.gateThreshold = c - 1;
            anchor.disabledNote =
                QStringLiteral("The %1 anchor is ignored by the engine: no "
                               "row has an Anchors count of %2 or more.")
                    .arg(QChar('A' + c - 2))
                    .arg(c - 1);
            s.curveColumns.append(anchor);
        }
        SectionCurveColumn brake{7, 0.0, 100.0, 2};
        brake.gateOwnNonZero = true;
        brake.disabledNote = QStringLiteral(
            "No row has a non-zero brake anchor, so the engine attaches no "
            "brake here. Set a value in the Values tab to enable it.");
        s.curveColumns.append(brake);
        specs.insert(3, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "First record: number of hole groups. Each group: first rib, "
            "last rib and hole count on their own lines, then one row per "
            "hole (shape type, chordwise position, vertical position, "
            "width, height, plus reserved zeros — 8 or 9 values depending "
            "on the design version).");
        specs.insert(4, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Extrados chord (%)", "Upper panel control point, % of "
                                         "chord."),
            column("Extrados tension", "Tension/shortening at that point."),
            column("Intrados chord (%)", "Lower panel control point, % of "
                                         "chord."),
            column("Intrados tension", "Tension/shortening at that point."),
        };
        s.structure = QStringLiteral(
            "A label record, six 4-value control rows (upper and lower "
            "chord/tension pairs), a fabric coefficient, and a final "
            "\"strain samples, elasticity\" record.");
        specs.insert(5, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Leading (mm)", "Allowance at the leading edge seam."),
            column("Internal (mm)", "Allowance at internal seams."),
            column("Trailing (mm)", "Allowance at the trailing edge seam."),
        };
        s.structure = QStringLiteral(
            "Two rows for upper/lower panels (3 values each), then single "
            "values for ribs and V-ribs. Millimetres; trailing words are "
            "labels.");
        specs.insert(6, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Spacing (cm)", "Distance between sewing marks."),
            column("Radius (cm)", "Mark point radius."),
            column("Displacement (cm)", "Mark displacement from the edge."),
        };
        s.structure =
            QStringLiteral("A single record of three values, centimetres.");
        specs.insert(7, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Six labelled single-value records: glide ratio, centre of "
            "pressure %, calage %, riser length, line length and karabiner "
            "separation (cm).");
        specs.insert(8, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Line plan: a control record, the number of line groups, then "
            "per group a row count followed by that many rows. Each row: "
            "branch count N, N (level, index) pairs padded with \"0 0\", "
            "then the final (panel, rib) attachment. Designs before 3.28 "
            "append \"- t1 t2 t3\" line-type codes.");
        specs.insert(9, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Brake line length, then a line plan with the same row grammar "
            "as section 9, then the brake distribution: a row of chordwise "
            "positions and a row of deflection values.");
        specs.insert(10, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Four records: branch level plus target lengths (cm) for the "
            "main and stabilo cascades.");
        specs.insert(11, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Index", "Reinforcement number (not required unique)."),
            column("Type", "Type code — it decides how many values follow: "
                           "1 = H strap, 3 = V-rib, 6 = continuous V-rib, "
                           "11/13/15 = diagonal variants, 16 = VH."),
        };
        s.structure = QStringLiteral(
            "First record: reinforcement count, then two global "
            "parameters. Each following row is one reinforcement whose "
            "layout depends on the type code in the second column.");
        specs.insert(12, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Number of colored panel groups; per group a (rib, color "
            "count) header followed by that many (color id, chord %, "
            "parameter) rows.");
        specs.insert(15, s);
        specs.insert(16, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Point count (0 = none), then one (x, y) row per point.");
        specs.insert(17, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Load %, then four rows distributing it over 2-5 line levels, "
            "then per-group correction rows (cm). Some designs append "
            "extra values after a \"-\" separator.");
        specs.insert(18, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Category", "Fixed drawing category token — do not "
                               "rename."),
            column("DXF layer", "Layer name used in the exported DXF."),
        };
        s.structure = QStringLiteral(
            "Layer-name count, then one (category, layer) row each.");
        specs.insert(19, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Mark type", "Fixed mark category token."),
            column("n1", "First shape code/count."),
            column("x1", "First shape dimension (mm)."),
            column("y1", "First shape dimension (mm)."),
            column("n2", "Second shape code/count."),
            column("x2", "Second shape dimension (mm)."),
            column("y2", "Second shape dimension (mm)."),
        };
        s.structure = QStringLiteral(
            "Mark-type count, then one row per mark category with two "
            "shape definitions.");
        specs.insert(20, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Nylon rod channels. 3.28: type, then group/preamble records, "
            "a (group, first rib, last rib) header and five profile rows. "
            "Older designs: enable flag, group count, and three profile "
            "rows per group.");
        specs.insert(21, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "0 disables the module. No shipped design enables it; the "
            "enabled layout is version-specific.");
        specs.insert(22, s);
        specs.insert(23, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Line family", "A-F line group whose drawing color is "
                                  "set."),
            column("Color", "AutoCAD color index."),
            column("Color name", "Documentary color name."),
        };
        s.structure = QStringLiteral(
            "Enable flag, then one color row per line family A-F.");
        specs.insert(24, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Enable flag, six (family, color, name) rows like section 24, "
            "then Extrados/Vents/Intrados rows with an extra 0/1 "
            "visibility flag before the color.");
        specs.insert(25, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Cell", "1-based cell, centre outwards; keep "
                           "sequential."),
            column("Type", "0 separate piece · 1 glue to upper · -1 glue "
                           "to lower · ±2/±3 fixed diagonals · ±4 diagonal "
                           "with endpoints · ±5 arc · ±6 ellipse."),
            column("Left (%)", "Left endpoint, % across the inlet (types "
                               "±4/±5) or X width (±6)."),
            column("Right (%)", "Right endpoint, % (types ±4/±5) or Y "
                                "width (±6)."),
            column("Arc (%)", "Bow depth, % of inlet length (type ±5 "
                              "only)."),
        };
        s.structure = QStringLiteral(
            "First record 0 = automatic vents; 1 = one row per cell "
            "(int(cells/2)+1 rows).");
        specs.insert(26, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "0 = standard wingtip. 1 adds AngleLE and AngleTE records "
            "(degrees).");
        specs.insert(27, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Enable flag, anchor-line count, a row of anchor chord "
            "percentages, then the calage variation parameters (two "
            "angle/step pairs).");
        specs.insert(28, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Enable flag and group records; per group an (index, first "
            "rib, last rib) row, upper/lower cut rows (an extra data row "
            "follows when the flag is 1), then the five Print parameter "
            "rows.");
        specs.insert(29, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Rib", "Rib number, centre outwards."),
            column("Factor", "Thickness multiplier: 1 keeps the profile, "
                             "0 collapses it (wingtip)."),
        };
        s.structure = QStringLiteral(
            "Enable flag, then one (rib, factor) row per half-wing rib.");
        s.curveColumns.append({1, 0.0, 2.0, 2});
        specs.insert(30, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Point", "Control point number within the group."),
            column("Extrados chord (%)", "Upper panel position, % of "
                                         "chord."),
            column("Extrados tension", "Tension at that point."),
            column("Intrados chord (%)", "Lower panel position, % of "
                                         "chord."),
            column("Intrados tension", "Tension at that point."),
        };
        s.structure = QStringLiteral(
            "Enable flag and group count; per group a (group, first rib, "
            "last rib, points, type) header followed by that many control "
            "rows.");
        specs.insert(31, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Parameter", "Slot name; meaning is positional (older "
                                "designs call these parameter6-10)."),
            column("Value", "Separation coefficient, normally near 1.0."),
        };
        s.structure = QStringLiteral(
            "Enable flag, then ten positional (name, value) rows for 2D "
            "part spacing.");
        specs.insert(32, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Riser", "Riser group label (A-E)."),
            column("Length (cm)", "Individual riser length."),
            column("Unit", "Documentary text; the engine ignores it."),
        };
        s.structure = QStringLiteral(
            "Enable flag and layout type, then one (label, length) row "
            "per riser group from section 9.");
        specs.insert(33, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Id", "Line type number referenced by section 9."),
            column("Shape", "c = cylindrical (one diameter) · r = "
                            "rectangular riser (width and thickness)."),
            column("Diameter (mm)", "Diameter, or width for shape r."),
            column("Name", "Product/type label for reports."),
            column("Strength", "Breaking strength."),
            column("daN", "Unit label."),
            column("Material", "Material label for the line report."),
            column("Mass", "Linear mass."),
            column("g", "Unit label (grams per metre)."),
            column("Loop", "Loop construction code (s or p)."),
            column("Loop len", "Extra length per end."),
            column("cm", "Unit label."),
            column("Color", "AutoCAD color index (used with special code "
                            "1341)."),
        };
        s.structure = QStringLiteral(
            "Enable flag and type count, then one catalogue row per line "
            "type.");
        specs.insert(34, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Name", "Physical quantity (order matters, names are "
                           "labels)."),
            column("Value", "Numeric value."),
            column("Unit", "Documentary unit."),
        };
        s.structure = QStringLiteral(
            "Enable flag, then 18 named values: g ro mu V Alpha Cl cle Cd "
            "cde Cm Spilot Cdpilot Mw Mp Pmc Mql Ycp Zcp.");
        specs.insert(35, s);
    }
    {
        SectionSpec s;
        s.structure = QStringLiteral(
            "Enable flag, four panel-mesh values (chordwise panels, per "
            "cell, chord distribution, span distribution), then the "
            "billowed-airfoil switch (unused in 3.28).");
        specs.insert(36, s);
    }
    {
        SectionSpec s;
        s.columns = {
            column("Code", "Feature switch code (e.g. 1291 STL segments, "
                           "1146 gravity centre %, 1341 line colors)."),
            column("Value", "Feature value; some codes take extra values "
                            "or a quoted string."),
        };
        s.structure = QStringLiteral(
            "Enable flag and code count, then one (code, value, "
            "description) row per switch. Value shape depends on the "
            "code.");
        specs.insert(37, s);
    }

    return specs;
}

} // namespace

const SectionSpec *sectionSpec(int number)
{
    static const QHash<int, SectionSpec> specs = buildSpecs();
    const auto it = specs.constFind(number);
    return it == specs.constEnd() ? nullptr : &it.value();
}
