#include "mainwindow.h"

#include "design_document.h"
#include "paraglider_view.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QPlainTextEdit>
#include <QProcess>
#include <QStyleFactory>
#include <QTextCursor>
#include <QTextDocument>
#include <QTextStream>
#include <QTemporaryDir>

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
        || viewport.shellCount() < 1
        || viewport.splineCount() < 500
        || viewport.triangleCount() < 1000) {
        QTextStream(stderr)
            << "Expected a non-trivial OCCT NURBS model, got "
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

} // namespace

int main(int argc, char *argv[])
{
    if (isHeadlessRequested(argc, argv)) {
        return runHeadless(argc, argv);
    }

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
    parser.addPositionalArgument(
        QStringLiteral("studio-files"),
        QStringLiteral("Design file used by --studio-self-test."),
        QStringLiteral("[design-file]"));
    parser.process(application);

    if (parser.isSet(studioSelfTest)) {
        return runStudioSelfTest(parser.positionalArguments());
    }

    MainWindow window;
    if (smokeTestRequested || parser.isSet(smokeTest)) {
        return 0;
    }

    window.show();
    return application.exec();
}
