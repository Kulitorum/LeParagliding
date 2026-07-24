#include "section_help.h"

SectionHelp helpForSection(int number, const QString &fallbackTitle)
{
    SectionHelp help;
    help.title = fallbackTitle;
    help.format = QStringLiteral(
        "Keep the records in their original order. Lines beginning with <code>*</code> "
        "are structural comments/placeholders; do not remove them or insert blank lines.");

    switch (number) {
    case 1:
        help.title = QStringLiteral("Geometry");
        help.purpose = QStringLiteral(
            "Defines the wing identity, drawing and wing scales, cell/rib counts, "
            "planform, vault and rib rotations. The rib matrix controls leading and "
            "trailing edges, chord position, height, beta, rotation point and wash-in.");
        help.notes = QStringLiteral(
            "Distances use centimetres. Ribs are entered for one half-wing and mirrored. "
            "Changing rib or cell counts also changes the expected row counts in many "
            "later sections.");
        break;
    case 2:
        help.title = QStringLiteral("Airfoils");
        help.purpose = QStringLiteral(
            "Assigns an airfoil coordinate file to every rib and defines inlet start/end, "
            "cell opening state, displacement and rib reinforcement width.");
        help.notes = QStringLiteral(
            "Airfoil paths are relative to the design file. Assigned profiles should use "
            "compatible point ordering and point counts.");
        break;
    case 3:
        help.title = QStringLiteral("Anchor points");
        help.purpose = QStringLiteral(
            "Defines suspension attachment positions along each rib chord for the A, B, "
            "C, D and optional additional line groups.");
        help.notes = QStringLiteral(
            "Positions are percentages of chord from the leading edge. The count at the "
            "start of each row controls how many anchor values are consumed.");
        break;
    case 4:
        help.title = QStringLiteral("Airfoil holes");
        help.purpose = QStringLiteral(
            "Creates lightening holes and structural cut-outs in selected ribs. Hole "
            "records choose a shape and its chordwise position, radii/widths and offsets.");
        help.notes = QStringLiteral(
            "A section starts with the number of rib groups, followed by rib identifiers, "
            "hole counts and shape records. Keep the nested counts consistent.");
        break;
    case 5:
        help.title = QStringLiteral("Skin tension");
        help.purpose = QStringLiteral(
            "Defines chordwise tension or shortening applied to upper and lower panels, "
            "plus the fabric elasticity model used to flatten panel geometry.");
        help.notes = QStringLiteral(
            "Control points are percentages of chord paired with tension values. Abrupt "
            "changes can create distorted panel outlines.");
        break;
    case 6:
        help.title = QStringLiteral("Sewing allowances");
        help.purpose = QStringLiteral(
            "Sets seam allowances for upper panels, lower panels, ribs and diagonal/V-ribs.");
        help.notes = QStringLiteral(
            "Unlike the main geometry, these values are millimetres. Each panel row gives "
            "leading, internal and trailing allowances.");
        break;
    case 7:
        help.title = QStringLiteral("Sewing marks");
        help.purpose = QStringLiteral(
            "Controls matching marks placed on ribs and panels: spacing, point radius and "
            "mark displacement.");
        help.notes = QStringLiteral(
            "Values are centimetres and affect manufacturing annotations, not the 3D wing.");
        break;
    case 8:
        help.title = QStringLiteral("Global angle of attack");
        help.purpose = QStringLiteral(
            "Estimates the general angle of attack from target glide ratio, center of "
            "pressure, calage, riser length, line length and carabiner separation.");
        help.notes = QStringLiteral(
            "These are design estimates used to place the pilot and suspension system; "
            "they do not replace aerodynamic validation or flight testing.");
        break;
    case 9:
        help.title = QStringLiteral("Suspension lines");
        help.purpose = QStringLiteral(
            "Describes the complete branching matrices for each riser group. Records map "
            "line levels, branches and final sail attachment points.");
        help.notes = QStringLiteral(
            "This is a counted hierarchy. Every group and row count must agree with the "
            "records that follow, or subsequent sections will be misread.");
        break;
    case 10:
        help.title = QStringLiteral("Brakes");
        help.purpose = QStringLiteral(
            "Defines brake-line branching, attachment rows and chordwise brake distribution "
            "across the span.");
        help.notes = QStringLiteral(
            "The final distribution records shape the brake trailing edge. Keep attachment "
            "indices valid for the rib and line topology.");
        break;
    case 11:
        help.title = QStringLiteral("Ramification lengths");
        help.purpose = QStringLiteral(
            "Specifies target lengths for intermediate suspension-line branches at each "
            "branching level.");
        help.notes = QStringLiteral(
            "Values are centimetres and are scaled with the wing. The number of values on "
            "a row depends on the associated branch level.");
        break;
    case 12:
        help.title = QStringLiteral("H, V and VH ribs");
        help.purpose = QStringLiteral(
            "Defines horizontal straps, diagonal V-ribs, continuous V-ribs and mixed VH "
            "reinforcements between ribs and chord positions.");
        help.notes = QStringLiteral(
            "The first value is the reinforcement count. Type codes change the number and "
            "meaning of fields; some modern types use percentages of chord.");
        break;
    case 15:
    case 16:
        help.title = number == 15 ? QStringLiteral("Upper-surface colors")
                                  : QStringLiteral("Lower-surface colors");
        help.purpose = QStringLiteral(
            "Splits upper or lower panels into color regions. Each selected rib/cell has a "
            "counted list of color boundaries and chordwise percentages.");
        help.notes = QStringLiteral(
            "Region indices are used as DXF layers/identifiers. Boundary percentages should "
            "remain ordered along the chord.");
        break;
    case 17:
        help.title = QStringLiteral("Additional rib points");
        help.purpose = QStringLiteral(
            "Adds explicit construction points to selected ribs for marks or local geometry.");
        help.notes = QStringLiteral(
            "Set the leading count to zero when unused. Otherwise provide exactly the "
            "declared number of point records.");
        break;
    case 18:
        help.title = QStringLiteral("Elastic line corrections");
        help.purpose = QStringLiteral(
            "Applies manufacturing/elastic corrections to calculated suspension-line "
            "lengths.");
        help.notes = QStringLiteral(
            "Use conservative corrections and verify the resulting values in "
            "<code>lep-out.txt</code> and <code>lines.txt</code>.");
        break;
    case 19:
        help.title = QStringLiteral("DXF layer names");
        help.purpose = QStringLiteral(
            "Maps fixed drawing categories—external cuts, sewing lines, points, circles, "
            "text and notes—to chosen DXF layer names.");
        help.notes = QStringLiteral(
            "Do not change the category token in the first column. Layer names should avoid "
            "spaces and characters unsupported by downstream CAD software.");
        break;
    case 20:
        help.title = QStringLiteral("Mark types");
        help.purpose = QStringLiteral(
            "Selects the geometry and dimensions used for manufacturing marks, such as "
            "points, circles, triangles or short line segments.");
        help.notes = QStringLiteral(
            "Several dimensions are millimetres. Settings should match the printer, laser "
            "or cutter workflow used by the manufacturer.");
        break;
    case 21:
        help.title = QStringLiteral("Nylon rods");
        help.purpose = QStringLiteral(
            "Defines chordwise nylon-rod channels (joncs), their affected ribs, endpoints, "
            "diameters and construction offsets.");
        help.notes = QStringLiteral(
            "The leading switch disables or enables the module. Keep group counts and rib "
            "ranges consistent.");
        break;
    case 22:
        help.title = QStringLiteral("Nose mylars");
        help.purpose = QStringLiteral(
            "Defines rigid mylar reinforcements around the leading edge and their extent "
            "along selected ribs.");
        help.notes = QStringLiteral(
            "Use zero to disable the module. Enabled definitions are counted and may use "
            "different upper/lower chordwise endpoints.");
        break;
    case 23:
        help.title = QStringLiteral("Tab reinforcements");
        help.purpose = QStringLiteral(
            "Controls reinforcement patches and tabs around suspension anchor points.");
        help.notes = QStringLiteral(
            "Use zero when unused. Patch dimensions and mark options affect 2D manufacturing "
            "patterns.");
        break;
    case 24:
        help.title = QStringLiteral("General 2D DXF options");
        help.purpose = QStringLiteral(
            "Controls which plan groups, annotations, lines and construction elements are "
            "included in the manufacturing DXF.");
        help.notes = QStringLiteral(
            "Most fields are boolean 0/1 switches followed by DXF color indices. These "
            "options do not change the aerodynamic geometry.");
        break;
    case 25:
        help.title = QStringLiteral("General 3D DXF options");
        help.purpose = QStringLiteral(
            "Controls the entities drawn into <code>lep-3d.dxf</code>, including profiles, "
            "ribs, panels, suspension lines and reference geometry.");
        help.notes = QStringLiteral(
            "Enable the geometry needed in the viewport while avoiding unnecessary detail "
            "for very large designs.");
        break;
    case 26:
        help.title = QStringLiteral("Glue vents");
        help.purpose = QStringLiteral(
            "Adds glue/seam vents and related cut geometry at specified ribs and chordwise "
            "positions.");
        help.notes = QStringLiteral(
            "Definitions are grouped by vent type and rib range. Dimensions are manufacturing "
            "parameters.");
        break;
    case 27:
        help.title = QStringLiteral("Special wingtip");
        help.purpose = QStringLiteral(
            "Activates special wingtip construction rules beyond the normal final rib/cell.");
        help.notes = QStringLiteral(
            "Use zero for the standard wingtip. Other type codes are version-specific and "
            "should be copied from a compatible design template.");
        break;
    case 28:
        help.title = QStringLiteral("Calage variation");
        help.purpose = QStringLiteral(
            "Defines alternative calage/speed-system or trimmer cases and the line-group "
            "changes used to generate comparison geometry and reports.");
        help.notes = QStringLiteral(
            "The base design is unchanged when the module is disabled. Validate every case "
            "against riser travel and safe line geometry.");
        break;
    case 29:
        help.title = QStringLiteral("3D shaping");
        help.purpose = QStringLiteral(
            "Defines transverse 3D-shaping cuts on upper and lower panels, grouped over rib "
            "ranges with one or more chordwise shaping positions.");
        help.notes = QStringLiteral(
            "The first switch enables the module. Negative shaping is supported, but large "
            "or discontinuous values can make panel patterns invalid.");
        break;
    case 30:
        help.title = QStringLiteral("Airfoil thickness modification");
        help.purpose = QStringLiteral(
            "Scales airfoil thickness independently by rib, allowing smooth tapering or a "
            "zero-thickness wingtip without a separate profile file.");
        help.notes = QStringLiteral(
            "Use smoothly varying coefficients. A coefficient of 1 preserves the source "
            "profile; 0 collapses its thickness.");
        break;
    case 31:
        help.title = QStringLiteral("New skin tension");
        help.purpose = QStringLiteral(
            "Provides panel-specific skin-tension curves with up to many chordwise control "
            "points, superseding the simpler global module where enabled.");
        help.notes = QStringLiteral(
            "Definitions are counted per panel/rib range. Keep chordwise coordinates ordered "
            "and avoid abrupt tension transitions between adjacent panels.");
        break;
    case 32:
        help.title = QStringLiteral("Parts separation");
        help.purpose = QStringLiteral(
            "Adjusts automatic horizontal and vertical spacing between individual 2D parts "
            "in the generated manufacturing plans.");
        help.notes = QStringLiteral(
            "Use zero to retain defaults. Enabled coefficients are normally close to 1.0 "
            "and affect layout only, not the wing geometry.");
        break;
    case 33:
        help.title = QStringLiteral("Detailed risers");
        help.purpose = QStringLiteral(
            "Defines detailed riser geometry and connection points for newer file versions.");
        help.notes = QStringLiteral(
            "This section is version-specific; retain the counted structure from a current "
            "template.");
        break;
    case 34:
        help.title = QStringLiteral("Line characteristics");
        help.purpose = QStringLiteral(
            "Assigns line types, diameters, colors and material characteristics used in "
            "reports and optional colored 3D line drawings.");
        help.notes = QStringLiteral(
            "Line type identifiers must match the suspension topology and declared table.");
        break;
    case 35:
        help.title = QStringLiteral("Equilibrium equations");
        help.purpose = QStringLiteral(
            "Configures the optional force and equilibrium calculation for wing, pilot and "
            "line-system loads.");
        help.notes = QStringLiteral(
            "Mass, drag and center-of-pressure inputs require validated engineering data.");
        break;
    case 36:
        help.title = QStringLiteral("XFLR5 export");
        help.purpose = QStringLiteral(
            "Creates panel and airfoil files for an optional XFLR5 aerodynamic analysis.");
        help.notes = QStringLiteral(
            "Use zero to disable it. XFLR5 cannot represent every paraglider geometry, "
            "especially rotated or single-skin configurations.");
        break;
    case 37:
        help.title = QStringLiteral("Special parameters");
        help.purpose = QStringLiteral(
            "A versioned extension table for special control codes that do not fit earlier "
            "sections.");
        help.notes = QStringLiteral(
            "Use zero unless a documented control code is required. Unknown codes can alter "
            "unrelated calculations.");
        break;
    default:
        help.purpose = QStringLiteral(
            "This section is not described by the bundled metadata. Preserve its record "
            "order and consult the manual for the matching program version.");
        help.notes = QStringLiteral(
            "Newer LEparagliding versions may add sections while retaining all earlier ones.");
        break;
    }
    return help;
}
