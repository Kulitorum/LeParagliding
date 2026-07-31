#include "mainwindow.h"

#include "design_document.h"
#include "paraglider_view.h"
#include "preset_catalog.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QSurfaceFormat>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPlainTextEdit>
#include <QProcess>
#include <QStyleFactory>
#include <QTextCursor>
#include <QTextDocument>
#include <QTextStream>
#include <QTemporaryDir>

#include <xfl3d/views/gl3dview.h>

namespace {

constexpr auto headlessOptionName = "headless";

void configureApplicationMetadata()
{
    QCoreApplication::setOrganizationName(QStringLiteral("Laboratori d'envol"));
    QCoreApplication::setApplicationName(QStringLiteral("LEparagliding"));
    QCoreApplication::setApplicationVersion(QStringLiteral("3.28"));
}

QCommandLineOption headlessOption()
{
    return QCommandLineOption(
        QString::fromLatin1(headlessOptionName),
        QStringLiteral("Run the calculation without opening the GUI."));
}

QString enginePath()
{
#ifdef Q_OS_WIN
    constexpr auto engineName = "leparagliding-engine.exe";
#else
    constexpr auto engineName = "leparagliding-engine";
#endif
    return QCoreApplication::applicationDirPath()
        + QLatin1Char('/')
        + QString::fromLatin1(engineName);
}

bool isHeadlessRequested(int argc, char *argv[])
{
    for (int index = 1; index < argc; ++index) {
        if (QString::fromLocal8Bit(argv[index])
            == QStringLiteral("--") + QString::fromLatin1(headlessOptionName)) {
            return true;
        }
    }
    return false;
}

int runStudioSelfTest(const QStringList &arguments)
{
    if (arguments.size() != 1) {
        QTextStream(stderr)
            << "--studio-self-test requires a design file.\n";
        return 2;
    }

    QString error;
    DesignDocument document;
    if (!document.load(arguments.at(0), &error)) {
        QTextStream(stderr) << "Design load failed: " << error << '\n';
        return 2;
    }
    if (!document.validationError().isEmpty()) {
        QTextStream(stderr)
            << "Design validation failed: " << document.validationError() << '\n';
        return 2;
    }
    if (document.sections().size() < 30) {
        QTextStream(stderr)
            << "Expected at least 30 sample sections, got "
            << document.sections().size() << ".\n";
        return 2;
    }
    for (int requiredSection = 33; requiredSection <= 37; ++requiredSection) {
        bool found = false;
        for (const DesignSection &section : document.sections()) {
            if (section.number == requiredSection) {
                found = true;
                break;
            }
        }
        if (!found) {
            QTextStream(stderr)
                << "Sample design is missing 3.28 section "
                << requiredSection << ".\n";
            return 2;
        }
    }

    // Exercise the exact widget conversion used by every section page.
    for (qsizetype index = 0; index < document.sections().size(); ++index) {
        QPlainTextEdit editor;
        editor.setPlainText(document.sections().at(index).text);
        document.setSectionText(index, editor.toPlainText());
    }

    QPlainTextEdit editorA;
    QPlainTextEdit editorB;
    QPlainTextEdit editorC;
    const auto prepareUndoEditor = [](QPlainTextEdit &editor, const QString &text) {
        editor.setPlainText(text);
        editor.setUndoRedoEnabled(true);
        editor.document()->clearUndoRedoStacks();
    };
    const auto appendEdit = [](QPlainTextEdit &editor, const QString &text) {
        editor.moveCursor(QTextCursor::End);
        editor.insertPlainText(text);
    };
    prepareUndoEditor(editorA, QStringLiteral("A"));
    prepareUndoEditor(editorB, QStringLiteral("B"));
    prepareUndoEditor(editorC, QStringLiteral("C"));
    appendEdit(editorA, QStringLiteral("-a1"));
    appendEdit(editorB, QStringLiteral("-b1"));
    appendEdit(editorC, QStringLiteral("-c1"));
    appendEdit(editorA, QStringLiteral("-a2"));
    appendEdit(editorC, QStringLiteral("-c2"));

    if (!editorA.document()->isUndoRedoEnabled()
        || editorA.document()->maximumBlockCount() != 0
        || !editorA.document()->isUndoAvailable()
        || !editorB.document()->isUndoAvailable()
        || !editorC.document()->isUndoAvailable()) {
        QTextStream(stderr) << "Section undo history was not enabled.\n";
        return 2;
    }

    const QString editorABeforeUndo = editorA.toPlainText();
    const QString editorBBeforeUndo = editorB.toPlainText();
    while (editorC.document()->isUndoAvailable()) {
        editorC.undo();
    }
    if (editorC.toPlainText() != QStringLiteral("C")
        || editorA.toPlainText() != editorABeforeUndo
        || editorB.toPlainText() != editorBBeforeUndo) {
        QTextStream(stderr)
            << "Undo history leaked between independent section editors.\n";
        return 2;
    }
    while (editorC.document()->isRedoAvailable()) {
        editorC.redo();
    }
    if (editorC.toPlainText() != QStringLiteral("C-c1-c2")) {
        QTextStream(stderr) << "Section redo did not restore all edits.\n";
        return 2;
    }

    QFile source(arguments.at(0));
    if (!source.open(QIODevice::ReadOnly)) {
        QTextStream(stderr) << "Could not reopen design: " << source.errorString() << '\n';
        return 2;
    }
    const QByteArray original = source.readAll();
    if (document.assembledText().toUtf8() != original) {
        QTextStream(stderr) << "Section editor round-trip changed the design file.\n";
        return 2;
    }

    QTemporaryDir historyDirectory;
    if (!historyDirectory.isValid()) {
        QTextStream(stderr) << "Could not create history test directory.\n";
        return 2;
    }
    const QString historyPath =
        historyDirectory.filePath(QStringLiteral("wing-with-history.txt"));
    if (!QFile::copy(arguments.at(0), historyPath)) {
        QTextStream(stderr) << "Could not create history test design.\n";
        return 2;
    }

    DesignDocument historyDocument;
    if (!historyDocument.load(historyPath, &error)
        || historyDocument.revisionCount() != 1) {
        QTextStream(stderr)
            << "Initial embedded-history state failed: " << error << '\n';
        return 2;
    }
    QString editedSection = historyDocument.sections().constFirst().text;
    if (!editedSection.endsWith(QLatin1Char('\n'))) {
        editedSection.append(QLatin1Char('\n'));
    }
    editedSection.append(QStringLiteral("* Studio persisted-history test\n"));
    historyDocument.setSectionText(0, editedSection);
    if (!historyDocument.save(&error)
        || historyDocument.revisionCount() != 2) {
        QTextStream(stderr)
            << "Could not save an embedded wing version: " << error << '\n';
        return 2;
    }

    QFile historyFile(historyPath);
    if (!historyFile.open(QIODevice::ReadOnly)
        || !historyFile.readAll().contains(
            "* >>> LEPARAGLIDING STUDIO HISTORY V1 >>>")) {
        QTextStream(stderr) << "Saved design has no embedded history trailer.\n";
        return 2;
    }
    historyFile.close();

    DesignDocument reopenedHistory;
    int sectionHistoryPosition = -1;
    if (!reopenedHistory.load(historyPath, &error)
        || reopenedHistory.revisionCount() != 2
        || reopenedHistory.sectionHistory(1, &sectionHistoryPosition).size() != 2
        || sectionHistoryPosition != 1
        || !reopenedHistory.restoreRevision(0, &error)
        || reopenedHistory.assembledText().toUtf8() != original
        || !reopenedHistory.save(&error)
        || reopenedHistory.revisionCount() != 3) {
        QTextStream(stderr)
            << "Embedded history restore failed: " << error << '\n';
        return 2;
    }

    DesignDocument restoredHistory;
    if (!restoredHistory.load(historyPath, &error)
        || restoredHistory.revisionCount() != 3
        || restoredHistory.assembledText().toUtf8() != original) {
        QTextStream(stderr)
            << "Restored wing did not survive reload: " << error << '\n';
        return 2;
    }

    QTemporaryDir modelDirectory;
    if (!modelDirectory.isValid()) {
        QTextStream(stderr) << "Could not create STEP model test directory.\n";
        return 2;
    }
    const QString outputDirectory =
        modelDirectory.filePath(QStringLiteral("output"));
    if (!QDir().mkpath(outputDirectory)) {
        QTextStream(stderr) << "Could not create STEP model output directory.\n";
        return 2;
    }

    QProcess engine;
    engine.setProgram(enginePath());
    engine.setArguments({arguments.at(0), outputDirectory});
    engine.setProcessChannelMode(QProcess::MergedChannels);
    engine.start();
    if (!engine.waitForStarted()
        || !engine.waitForFinished(150000)
        || engine.exitStatus() != QProcess::NormalExit
        || engine.exitCode() != 0) {
        QTextStream(stderr)
            << "NURBS engine self-test failed:\n"
            << QString::fromLocal8Bit(engine.readAll()) << '\n';
        return 2;
    }

    const QString stepPath =
        QDir(outputDirectory).filePath(QStringLiteral("lep-3d.step"));
    QFile stepFile(stepPath);
    if (!stepFile.open(QIODevice::ReadOnly)
        || !stepFile.read(8192).contains(
            "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING")) {
        QTextStream(stderr)
            << "Generated model is not an AP242 STEP file.\n";
        return 2;
    }
    stepFile.close();

    ParagliderView viewport;
    if (!viewport.loadStep(stepPath, &error)) {
        QTextStream(stderr) << "3D STEP load failed: " << error << '\n';
        return 2;
    }
    if (viewport.surfaceCount() < 50
        || viewport.rationalSurfaceCount() < 1
        || viewport.partCount() < 100
        || viewport.splineCount() < 500
        || viewport.triangleCount() < 1000) {
        QTextStream(stderr)
            << "Expected a non-trivial OCCT NURBS model, got "
            << viewport.modelSummary() << ".\n";
        return 2;
    }

    // The engine exports a named assembly; the viewport must reconstruct
    // the part structure from it.
    bool hasExtrados = false;
    bool hasRibs = false;
    bool hasLinePlan = false;
    bool hasPanelLeaf = false;
    for (const ParagliderView::PartInfo &part : viewport.partTree()) {
        if (part.isGroup && part.name == QStringLiteral("Extrados")) {
            hasExtrados = true;
        }
        if (part.isGroup && part.name == QStringLiteral("Ribs")) {
            hasRibs = true;
        }
        if (part.isGroup && part.name.startsWith(QStringLiteral("Plan "))) {
            hasLinePlan = true;
        }
        if (!part.isGroup
            && part.name.startsWith(QStringLiteral("Panel "))
            && part.role == ParagliderView::ColorRole::Extrados) {
            hasPanelLeaf = true;
        }
    }
    if (!hasExtrados || !hasRibs || !hasLinePlan || !hasPanelLeaf) {
        QTextStream(stderr)
            << "The STEP assembly part tree is missing expected groups "
            << "(Extrados/Ribs/Plan */Panel leaves).\n";
        return 2;
    }

    // The preview pipeline hands the model over as binary XCAF instead of
    // STEP; it must load and describe the same assembly.
    QProcess previewEngine;
    previewEngine.setProgram(enginePath());
    previewEngine.setArguments(
        {QStringLiteral("--preview"), arguments.at(0), outputDirectory});
    previewEngine.setProcessChannelMode(QProcess::MergedChannels);
    previewEngine.start();
    if (!previewEngine.waitForStarted()
        || !previewEngine.waitForFinished(150000)
        || previewEngine.exitStatus() != QProcess::NormalExit
        || previewEngine.exitCode() != 0) {
        QTextStream(stderr)
            << "NURBS engine preview self-test failed:\n"
            << QString::fromLocal8Bit(previewEngine.readAll()) << '\n';
        return 2;
    }
    const QString xbfPath =
        QDir(outputDirectory).filePath(QStringLiteral("lep-3d.xbf"));
    ParagliderView previewViewport;
    if (!previewViewport.loadStep(xbfPath, &error)) {
        QTextStream(stderr)
            << "Binary XCAF preview load failed: " << error << '\n';
        return 2;
    }
    if (previewViewport.partCount() != viewport.partCount()
        || previewViewport.surfaceCount() != viewport.surfaceCount()
        || previewViewport.splineCount() != viewport.splineCount()) {
        QTextStream(stderr)
            << "Preview (binary XCAF) and STEP disagree: "
            << previewViewport.modelSummary() << " vs "
            << viewport.modelSummary() << ".\n";
        return 2;
    }

    // Exercise the native OCCT WNT/OpenGL presentation as well as STEP
    // import and meshing. This catches viewer/runtime deployment failures
    // that a shape-only test would miss.
    viewport.resize(800, 600);
    viewport.show();
    QApplication::processEvents();
    viewport.fitAll();
    QApplication::processEvents();
    viewport.hide();

    QTextStream(stdout)
        << document.sections().size() << " sections; "
        << viewport.modelSummary() << '\n';
    return 0;
}

// The engine tolerates upstream quirks (blank lines, unread trailer blocks
// after section 37), but the Studio editor refuses to start a calculation on
// them. Every shipped preset must satisfy the stricter of the two, so this
// walks the whole catalog through the same load + validation the GUI uses.
int runPresetValidation(const QStringList &arguments)
{
    if (arguments.size() != 1) {
        QTextStream(stderr)
            << "--validate-presets requires a presets directory.\n";
        return 2;
    }
    const QString directory = arguments.at(0);

    qsizetype manifestVariants = 0;
    QFile manifest(QDir(directory).filePath(QStringLiteral("presets.json")));
    if (manifest.open(QIODevice::ReadOnly)) {
        const QJsonArray wings = QJsonDocument::fromJson(manifest.readAll())
                                     .object()
                                     .value(QStringLiteral("wings"))
                                     .toArray();
        for (const QJsonValue &wing : wings) {
            manifestVariants +=
                wing.toObject().value(QStringLiteral("variants")).toArray().size();
        }
    }

    const QList<PresetWing> catalog = loadPresetCatalog(directory);
    if (catalog.isEmpty()) {
        QTextStream(stderr) << "No presets were found in " << directory << '\n';
        return 2;
    }

    int failures = 0;
    qsizetype loadedVariants = 0;
    for (const PresetWing &wing : catalog) {
        for (const PresetVariant &variant : wing.variants) {
            ++loadedVariants;
            QString error;
            DesignDocument document;
            if (document.load(variant.designFile, &error)) {
                error = document.validationError();
            }
            if (!error.isEmpty()) {
                ++failures;
                QTextStream(stderr)
                    << wing.name << " (" << variant.label << "): " << error
                    << '\n' << "    " << variant.designFile << '\n';
            }
        }
    }

    // loadPresetCatalog silently drops variants whose design file is
    // missing, so a packaging mistake must be caught by comparing against
    // the manifest itself.
    if (loadedVariants != manifestVariants) {
        QTextStream(stderr)
            << "presets.json lists " << manifestVariants
            << " variants but only " << loadedVariants
            << " design files were found.\n";
        return 2;
    }

    if (failures != 0) {
        QTextStream(stderr)
            << failures << " of " << loadedVariants
            << " preset variants failed Studio validation.\n";
        return 2;
    }
    QTextStream(stdout)
        << loadedVariants << " preset variants load and validate in Studio.\n";
    return 0;
}

int runHeadless(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);
    configureApplicationMetadata();

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("LEparagliding command-line calculation"));
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addOption(headlessOption());
    parser.addPositionalArgument(
        QStringLiteral("design-file"),
        QStringLiteral("LEparagliding input design file."));
    parser.addPositionalArgument(
        QStringLiteral("output-directory"),
        QStringLiteral("Directory for the generated result files."));
    parser.process(application);

    const QStringList arguments = parser.positionalArguments();
    if (arguments.size() != 2) {
        QTextStream(stderr)
            << "Headless mode requires a design file and an output directory.\n\n";
        parser.showHelp(2);
    }

    const QString executable = enginePath();
    if (!QFileInfo(executable).isExecutable()) {
        QTextStream(stderr)
            << "Calculation engine not found: " << executable << '\n';
        return 2;
    }

    QProcess process;
    process.setProcessChannelMode(QProcess::ForwardedChannels);
    process.setProgram(executable);
    process.setArguments(arguments);
    process.start();
    if (!process.waitForStarted()) {
        QTextStream(stderr)
            << "Could not start calculation engine: "
            << process.errorString() << '\n';
        return 2;
    }

    process.closeWriteChannel();
    if (!process.waitForFinished(-1)) {
        QTextStream(stderr)
            << "Calculation engine did not finish: "
            << process.errorString() << '\n';
        return 2;
    }
    if (process.exitStatus() != QProcess::NormalExit) {
        QTextStream(stderr) << "Calculation engine crashed.\n";
        return 2;
    }
    return process.exitCode();
}

// The embedded XFLR5 uses QOpenGLWidget views that require shared contexts
// and a default surface format chosen before QApplication exists. Mirrors
// setOGLDefaultFormat() in XFLR5's own main.cpp (third_party/xflr5), reading
// the same XFLR5 settings file so a standalone XFLR5's OpenGL preferences
// carry over. The OCCT viewport is unaffected: it renders into a native
// window with its own WGL context.
void configureXflr5OpenGL()
{
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);

#if defined(Q_OS_LINUX) || defined(Q_OS_MAC)
    QSettings settings(QSettings::NativeFormat, QSettings::UserScope,
                       QStringLiteral("sourceforge.net"), QStringLiteral("xflr5"));
#else
    QSettings settings(QSettings::IniFormat, QSettings::UserScope,
                       QStringLiteral("XFLR5"));
#endif
    int oglMajor = 3;
    int oglMinor = 3;
    if (QFile(settings.fileName()).exists()) {
        gl3dView::loadSettings(settings);
        oglMajor = gl3dView::oglMajor();
        oglMinor = gl3dView::oglMinor();
    }

    if (oglMajor <= 2 || (oglMajor == 3 && oglMinor < 3)) {
        gl3dView::setProfile(QSurfaceFormat::NoProfile);
        gl3dView::setDeprecatedFuncs(true);
    } else {
        gl3dView::setProfile(QSurfaceFormat::CoreProfile);
        gl3dView::setDeprecatedFuncs(false);
    }
    gl3dView::setRenderableType(QSurfaceFormat::OpenGL);
    gl3dView::setOGLVersion(oglMajor, oglMinor);
    if (gl3dView::defaultXflSurfaceFormat().samples() < 0) {
        gl3dView::setDefaultSamples(4);
    }
    QSurfaceFormat sharedFormat = gl3dView::defaultXflSurfaceFormat();
    // A stencil buffer app-wide: the Playground draws its legend with
    // QPainter over the GL scene, and the painter's GL engine silently
    // drops filled paths without one. App-wide rather than per-widget —
    // a per-widget format that diverges from the shared context left
    // that view blank.
    sharedFormat.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(sharedFormat);
}

} // namespace

int main(int argc, char *argv[])
{
    if (isHeadlessRequested(argc, argv)) {
        return runHeadless(argc, argv);
    }

    configureXflr5OpenGL();

    bool smokeTestRequested = false;
    for (int index = 1; index < argc; ++index) {
        if (QString::fromLocal8Bit(argv[index]) == QStringLiteral("--smoke-test")) {
            smokeTestRequested = true;
        }
    }

    QApplication application(argc, argv);
    configureApplicationMetadata();
    application.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));

    QCommandLineParser parser;
    parser.setApplicationDescription(
        QStringLiteral("LEparagliding C++ / Qt"));
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addOption(headlessOption());
    QCommandLineOption smokeTest(QStringLiteral("smoke-test"),
                                 QStringLiteral("Construct the GUI and exit immediately."));
    parser.addOption(smokeTest);
    QCommandLineOption studioSelfTest(
        QStringLiteral("studio-self-test"),
        QStringLiteral("Build, reload, and validate the OCCT NURBS model, then exit."));
    parser.addOption(studioSelfTest);
    QCommandLineOption validatePresets(
        QStringLiteral("validate-presets"),
        QStringLiteral("Load every catalog preset through the Studio editor "
                       "checks, then exit."));
    parser.addOption(validatePresets);
    QCommandLineOption xflr5Tab(
        QStringLiteral("xflr5"),
        QStringLiteral("Open on the Aerodynamics (XFLR5) tab."));
    parser.addOption(xflr5Tab);
    QCommandLineOption playgroundTab(
        QStringLiteral("playground"),
        QStringLiteral("Open on the Playground tab with the given "
                       "lep-sim.json loaded."),
        QStringLiteral("sim-mesh"));
    parser.addOption(playgroundTab);
    parser.addPositionalArgument(
        QStringLiteral("studio-files"),
        QStringLiteral("Design file used by --studio-self-test, or presets "
                       "directory used by --validate-presets."),
        QStringLiteral("[design-file]"));
    parser.process(application);

    if (parser.isSet(studioSelfTest)) {
        return runStudioSelfTest(parser.positionalArguments());
    }
    if (parser.isSet(validatePresets)) {
        return runPresetValidation(parser.positionalArguments());
    }

    MainWindow window;
    if (smokeTestRequested || parser.isSet(smokeTest)) {
        // Also construct the embedded XFLR5 MainFrame so regressions in the
        // vendored code surface in CI, not on first tab click — but without
        // the wing transfer, which would spawn an engine run.
        window.showXflr5Tab(false);
        return 0;
    }

    if (parser.isSet(xflr5Tab)) {
        window.showXflr5Tab();
    }
    if (parser.isSet(playgroundTab)) {
        window.showPlaygroundTab(parser.value(playgroundTab));
    }
    // Maximised rather than true full screen: the design tabs, the part tree
    // and the 2D preview all want the space, but a CAD-style tool still needs
    // its title bar and the rest of the desktop reachable.
    window.showMaximized();
    return application.exec();
}
