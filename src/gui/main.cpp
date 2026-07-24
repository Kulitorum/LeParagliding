#include "mainwindow.h"

#include "design_document.h"
#include "paraglider_view.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QPlainTextEdit>
#include <QProcess>
#include <QStyleFactory>
#include <QTextStream>

namespace {

constexpr auto headlessOptionName = "headless";

void configureApplicationMetadata()
{
    QCoreApplication::setOrganizationName(QStringLiteral("Laboratori d'envol"));
    QCoreApplication::setApplicationName(QStringLiteral("LEparagliding"));
    QCoreApplication::setApplicationVersion(QStringLiteral("3.17"));
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
    if (arguments.size() != 2) {
        QTextStream(stderr)
            << "--studio-self-test requires a design file and a 3D DXF file.\n";
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
    if (document.sections().size() != 30) {
        QTextStream(stderr)
            << "Expected 30 sample sections, got "
            << document.sections().size() << ".\n";
        return 2;
    }

    // Exercise the exact widget conversion used by every section page.
    for (qsizetype index = 0; index < document.sections().size(); ++index) {
        QPlainTextEdit editor;
        editor.setPlainText(document.sections().at(index).text);
        document.setSectionText(index, editor.toPlainText());
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

    ParagliderView viewport;
    if (!viewport.loadDxf(arguments.at(1), &error)) {
        QTextStream(stderr) << "3D DXF load failed: " << error << '\n';
        return 2;
    }
    if (viewport.segmentCount() != 5515) {
        QTextStream(stderr)
            << "Expected 5515 sample DXF segments, got "
            << viewport.segmentCount() << ".\n";
        return 2;
    }

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
        QStringLiteral("Directory for the four generated result files."));
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
        QStringLiteral("Validate section parsing and 3D DXF loading, then exit."));
    parser.addOption(studioSelfTest);
    parser.addPositionalArgument(
        QStringLiteral("studio-files"),
        QStringLiteral("Design and DXF files used by --studio-self-test."),
        QStringLiteral("[design-file] [dxf-file]"));
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
