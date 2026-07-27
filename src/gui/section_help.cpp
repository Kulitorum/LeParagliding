#include "section_help.h"

#include "section1_curves.h"

#include <QFile>

namespace {

// Section explanations converted from the official manual
// (https://www.laboratoridenvol.com/leparagliding/manual.en.html) by
// tools/extract_manual.py into resources/manual/.
QString manualHtmlForSection(int number)
{
    QFile file(QStringLiteral(":/manual/sections/section%1.html")
                   .arg(number, 2, 10, QLatin1Char('0')));
    if (!file.open(QIODevice::ReadOnly)) {
        return {};
    }
    return QString::fromUtf8(file.readAll());
}

// Field reference for Section 1, generated from the same column metadata the
// graphical curve editor uses, so the two cannot drift apart.
QString section1FieldReferenceHtml()
{
    QString html = QStringLiteral(
        "<p>Scalar records before the matrix: brand and wing name (in "
        "quotes), drawing scale, wing scale, number of cells, number of "
        "ribs, \"Alpha max\" (the maximum automatic washin angle plus a "
        "distribution parameter), and the paraglider type code with its "
        "parameter.</p>"
        "<p>Rib matrix columns, one row per rib of the half wing:</p>"
        "<table cellspacing=\"0\" cellpadding=\"4\">");
    for (const lep::Section1Column &column : lep::section1Columns()) {
        html += QStringLiteral(
                    "<tr><td><b>%1</b>&nbsp;</td><td>%2&nbsp;</td>"
                    "<td>%3</td></tr>")
                    .arg(QLatin1String(column.label),
                         QLatin1String(column.unit),
                         QLatin1String(column.description));
    }
    html += QStringLiteral("</table>");
    return html;
}

} // namespace

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
        help.details = section1FieldReferenceHtml();
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
            "These switches affect only the legacy DXF reference output. The OCCT STEP "
            "model and viewport always use the complete calculated NURBS geometry.");
        break;
    case 26:
        help.title = QStringLiteral("Glue vents");
        help.purpose = QStringLiteral(
            "Chooses how the air inlet in every cell is joined to the upper skin "
            "(extrados), lower skin (intrados), or kept as a separate piece. Advanced "
            "types make the inlet diagonal, curved, or elliptical.");
        help.format = QStringLiteral(
            "The first data record is <code>0</code> for the old automatic vent style, or "
            "<code>1</code> for the explicit table. With <code>1</code>, provide exactly "
            "<code>floor(Number of cells / 2) + 1</code> rows (15 rows for the bundled "
            "29-cell design): "
            "<code>cell type [left% right% [arc%]]</code>. The row order is significant; "
            "keep the first column sequential from 1 through that row count.");
        help.notes = QStringLiteral(
            "Positive types attach the vent to the upper panel; negative types attach it "
            "to the lower panel. Percentages normally stay between 0 and 100. Section 25 "
            "must have <code>Vents 1 …</code> if you want the vent edges visible in the "
            "legacy 3D DXF; the STEP viewport is independent of DXF visibility settings.");
        help.details = QStringLiteral(
            R"(<table cellspacing="0" cellpadding="5" border="1">
<tr><th>Record/value</th><th>Meaning</th></tr>
<tr><td><code>0</code> (first record)</td><td>Use the old automatic inlet construction; no per-cell rows follow.</td></tr>
<tr><td><code>1</code> (first record)</td><td>Enable the explicit table; <code>floor(Number of cells / 2) + 1</code> rows follow.</td></tr>
<tr><td><code>cell</code></td><td>1-based cell between rib <i>cell-1</i> and rib <i>cell</i>. The parser consumes rows in order, so this label must match the row position.</td></tr>
<tr><td><code>type 0</code></td><td>Do not glue the vent to either skin; produce it as a separate open-inlet piece.</td></tr>
<tr><td><code>type 1</code></td><td>Glue the vent to the upper skin (extrados); commonly used for single-skin construction.</td></tr>
<tr><td><code>type -1</code></td><td>Glue the vent to the lower skin (intrados); commonly used for a closed cell.</td></tr>
<tr><td><code>type -2</code></td><td>Fixed diagonal: fully open at the left side, glued to the lower skin.</td></tr>
<tr><td><code>type -3</code></td><td>Fixed diagonal: fully open at the right side, glued to the lower skin.</td></tr>
<tr><td><code>type ±4 left right</code></td><td>General straight diagonal. <i>left</i> and <i>right</i> locate its two endpoints as percentages across the inlet sides. The sign chooses upper (+) or lower (-) attachment.</td></tr>
<tr><td><code>type ±5 left right arc</code></td><td>Same endpoints as ±4, joined by an arc. <i>arc</i> is the bow/deflection depth as a percentage of average inlet length.</td></tr>
<tr><td><code>type ±6 widthX widthY</code></td><td>Elliptical inlet. The two values are ellipse width percentages in the local X and Y directions.</td></tr>
</table>
<p><b>Bundled design example:</b></p>
<ul>
<li><code>1 -6 70 100</code>: cell 1, elliptical inlet attached to the lower skin, 70% × 100%.</li>
<li><code>2 -4 0 80</code>: cell 2, lower-skin straight diagonal from 0% on the left to 80% on the right.</li>
<li><code>9 4 100 0</code>: cell 9, upper-skin straight diagonal in the opposite direction.</li>
<li><code>10 5 0 0 80</code>: cell 10, upper-skin curved inlet with endpoints at 0% and an 80% bow depth.</li>
<li><code>12 -5 50 50 50</code>: cell 12, lower-skin curved inlet, both endpoints at 50%, with 50% bow depth.</li>
<li><code>15 -1</code>: cell 15 is attached directly to the lower skin; no extra parameters are read.</li>
</ul>)");
        help.experiment = QStringLiteral(
            R"(<ol>
<li>In section 25 change <code>Vents 0 1 red</code> to <code>Vents 1 1 red</code>.</li>
<li>Change only one cell at a time—for example cell 2 from <code>-4 0 80</code> to <code>-4 20 80</code>.</li>
<li>Press <b>Enter</b> to build and reload the viewport. Use <b>Shift+Enter</b> only when you need to insert a new record.</li>
<li>Compare the updated 3D seam and the 2D manufacturing panel before changing another value. Start with small percentage changes.</li>
</ol>)");
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
            "Overrides the common riser length from section 8 with an individual length "
            "for each riser group.");
        help.format = QStringLiteral(
            "The first data record enables the module. With <code>0</code>, every riser "
            "uses <code>clengr</code> from section 8. With <code>1</code>, the next record "
            "is the riser-layout type, followed by one <code>label length</code> record "
            "per riser group declared by section 9.");
        help.notes = QStringLiteral(
            "Lengths are centimetres. Type <code>1</code> (normal separate risers) is the "
            "implemented 3.28 layout. Types 2–4 are reserved placeholders and currently "
            "read no geometry.");
        help.details = QStringLiteral(
            R"(<table cellspacing="0" cellpadding="5" border="1">
<tr><th>Record</th><th>Meaning</th></tr>
<tr><td><code>0</code></td><td>Disable detailed risers; copy the section 8 riser length to A–E.</td></tr>
<tr><td><code>1</code></td><td>Enable detailed risers.</td></tr>
<tr><td><code>1</code> (layout type)</td><td>Normal risers. One following row is required for each riser group.</td></tr>
<tr><td><code>A 47.0 cm</code></td><td>Label A and its 47.0 cm length. Text after the number is ignored, so the unit is documentary.</td></tr>
</table>)");
        break;
    case 34:
        help.title = QStringLiteral("Line characteristics");
        help.purpose = QStringLiteral(
            "Defines the physical line/riser catalogue used to calculate length, mass, "
            "frontal area, drag, strength labels and CAD colors.");
        help.format = QStringLiteral(
            "Use <code>0</code> for the six built-in types. For a custom catalogue use "
            "<code>1</code>, then the number of types, followed by that many rows. "
            "Cylindrical rows use "
            "<code>id c diameter label strength daN material mass g loop loopLength cm color</code>. "
            "Rectangular riser rows use <code>id r width thickness</code> before the label.");
        help.notes = QStringLiteral(
            "Diameter/width and thickness are millimetres; strength is daN; mass is grams "
            "per metre; loop length is centimetres. The final integer is the AutoCAD color "
            "index. Type IDs referenced by the section 9 line matrix must exist here.");
        help.details = QStringLiteral(
            R"(<table cellspacing="0" cellpadding="5" border="1">
<tr><th>Field</th><th>Meaning</th></tr>
<tr><td><code>id</code></td><td>Line type number used by the suspension-line matrix.</td></tr>
<tr><td><code>c</code> / <code>r</code></td><td>Cylindrical line/riser with one diameter, or rectangular riser with width and thickness.</td></tr>
<tr><td><code>label</code></td><td>Short human-readable product/type name printed in reports.</td></tr>
<tr><td><code>strength</code></td><td>Rated breaking strength in daN.</td></tr>
<tr><td><code>material</code></td><td>Material label printed in the line report.</td></tr>
<tr><td><code>mass</code></td><td>Linear mass in g/m, used for total line-system mass.</td></tr>
<tr><td><code>loop</code></td><td>Loop construction code (<code>s</code> or <code>p</code>); it changes the drag-area correction.</td></tr>
<tr><td><code>loopLength</code></td><td>Extra length at each end in centimetres.</td></tr>
<tr><td><code>color</code></td><td>AutoCAD color index, used when special code 1341 enables per-type colors.</td></tr>
</table>)");
        break;
    case 35:
        help.title = QStringLiteral("Equilibrium equations");
        help.purpose = QStringLiteral(
            "Solves an informative longitudinal force balance for the wing, line system "
            "and pilot, and writes estimated speed, glide angle, loads and drag.");
        help.format = QStringLiteral(
            "Use <code>0</code> to disable the solver. With <code>1</code>, supply exactly "
            "18 named values in this order: <code>g ro mu V Alpha Cl cle Cd cde Cm "
            "Spilot Cdpilot Mw Mp Pmc Mql Ycp Zcp</code>. The names and trailing unit text "
            "are labels; record order determines what is read.");
        help.notes = QStringLiteral(
            "Alpha is degrees; masses are kg except quick-link mass <code>Mql</code> in "
            "grams; areas and center coordinates are metres. Coefficients normally come "
            "from XFLR5/CFD or measured data. This is an estimate, not structural or "
            "flight-safety validation.");
        help.details = QStringLiteral(
            R"(<table cellspacing="0" cellpadding="5" border="1">
<tr><th>Field</th><th>Meaning / unit</th></tr>
<tr><td><code>g</code></td><td>Gravity, m/s².</td></tr>
<tr><td><code>ro</code>, <code>mu</code></td><td>Air density in kg/m³ and dynamic viscosity in µPa·s.</td></tr>
<tr><td><code>V</code>, <code>Alpha</code></td><td>Initial flow speed in m/s and wing angle of attack in degrees.</td></tr>
<tr><td><code>Cl</code>, <code>Cd</code>, <code>Cm</code></td><td>Wing lift, drag and moment coefficients.</td></tr>
<tr><td><code>cle</code>, <code>cde</code></td><td>Multipliers for the lift and drag coefficients.</td></tr>
<tr><td><code>Spilot</code>, <code>Cdpilot</code></td><td>Pilot/harness frontal area in m² and drag coefficient.</td></tr>
<tr><td><code>Mw</code>, <code>Mp</code></td><td>Wing and pilot-plus-harness masses in kg.</td></tr>
<tr><td><code>Pmc</code></td><td>Pilot mass-center distance below the main carabiners, m.</td></tr>
<tr><td><code>Mql</code></td><td>Mass of one quick link, grams.</td></tr>
<tr><td><code>Ycp</code>, <code>Zcp</code></td><td>Wing center-of-pressure coordinates, m, in LEparagliding's Y/Z axes.</td></tr>
</table>)");
        break;
    case 36:
        help.title = QStringLiteral("XFLR5 export");
        help.purpose = QStringLiteral(
            "Exports an XFLR5 wing-import file and one normalized <code>.dat</code> airfoil "
            "per rib into the output folder's <code>xflr5</code> subdirectory.");
        help.format = QStringLiteral(
            "Use <code>0</code> to disable export. With <code>1</code>, provide a comment "
            "record, then four integer panel controls: chordwise panels, panels per cell, "
            "chordwise distribution, spanwise distribution; provide another comment "
            "record and the billowed-airfoil switch.");
        help.notes = QStringLiteral(
            "Distribution <code>0</code> is uniform and <code>1</code> requests cosine "
            "spacing. The final billowed-airfoil value is read but not used by the 3.28 "
            "exporter. XFLR5 cannot represent every paraglider or single-skin detail.");
        help.details = QStringLiteral(
            R"(<table cellspacing="0" cellpadding="5" border="1">
<tr><th>Example</th><th>Meaning</th></tr>
<tr><td><code>10</code></td><td>10 panels along the chord.</td></tr>
<tr><td><code>5</code></td><td>5 spanwise panels per cell.</td></tr>
<tr><td><code>1</code></td><td>Cosine distribution along the chord.</td></tr>
<tr><td><code>0</code></td><td>Uniform distribution along the span.</td></tr>
<tr><td><code>0</code></td><td>Billowed-airfoil option; reserved/unused in 3.28.</td></tr>
</table>)");
        break;
    case 37:
        help.title = QStringLiteral("Special parameters");
        help.purpose = QStringLiteral(
            "Holds optional versioned feature switches that alter STL resolution, line "
            "colors, equilibrium reporting, mass-center assumptions and minirib shaping.");
        help.format = QStringLiteral(
            "Use <code>0</code> for all 3.28 defaults. With <code>1</code>, the next record "
            "is the number of code rows, followed by exactly that many "
            "<code>code value...</code> records.");
        help.notes = QStringLiteral(
            "Unknown codes are consumed but ignored. Keep the declared row count exact. "
            "These are low-level controls; change one at a time and inspect both the report "
            "and generated geometry.");
        help.details = QStringLiteral(
            R"(<table cellspacing="0" cellpadding="5" border="1">
<tr><th>Code</th><th>Value</th></tr>
<tr><td><code>1291 n</code></td><td>Transverse segment count for tessellated STL surfaces; default 12.</td></tr>
<tr><td><code>1341 0|1</code></td><td>Use the section 34 AutoCAD color for each line type.</td></tr>
<tr><td><code>1146 percent</code></td><td>Typical section center of mass as percent chord; default 36.</td></tr>
<tr><td><code>1351 method</code></td><td>Equilibrium solver method. Method 1 is the normal Casellas method; other experimental methods are retained by upstream.</td></tr>
<tr><td><code>1352 1|2</code></td><td>Use flat (1) or projected (2) wing area in the equilibrium solver; default 2.</td></tr>
<tr><td><code>1353 0|1|2</code></td><td>Normal (0), detailed (1), or extended detailed (2) report output.</td></tr>
<tr><td><code>2000 "date"</code></td><td>Edition date printed into output metadata.</td></tr>
<tr><td><code>2003 angle</code></td><td>Force the assiette angle in degrees for local angle calculations.</td></tr>
<tr><td><code>3001 mode a b</code></td><td>Minirib transition: 0 disabled, 1 linear, 2 parabolic, 3 cosinusoidal, with two shaping parameters.</td></tr>
</table>)");
        break;
    default:
        help.purpose = QStringLiteral(
            "This section is not described by the bundled metadata. Preserve its record "
            "order and consult the manual for the matching program version.");
        help.notes = QStringLiteral(
            "Newer LEparagliding versions may add sections while retaining all earlier ones.");
        break;
    }
    help.manual = manualHtmlForSection(number);
    return help;
}
