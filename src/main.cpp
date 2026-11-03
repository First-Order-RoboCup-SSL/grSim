/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <QtWidgets/QApplication>
#include "mainwindow.h"
#include <cstdio>
#include <stdexcept>

#ifdef GRSIM_BUILD_SMOKE_TEST
int runApplicationSmokeTest(QApplication& app, const QString& settingsPath);
#endif

int main(int argc, char *argv[])
{
    std::locale::global( std::locale( "" ) );
    
    QCoreApplication::setOrganizationName("Parsian");
    QCoreApplication::setOrganizationDomain("parsian-robotics.com");
    QCoreApplication::setApplicationName("grSim");
    QApplication a(argc, argv);

    QCommandLineParser parser;
    parser.setApplicationDescription("RoboCup Small Size League Simulator");
    parser.addHelpOption();
    QCommandLineOption headlessOption(QStringList() << "H" << "headless", 
                                      QCoreApplication::translate("main", "Run without a UI"));
    parser.addOption(headlessOption);
    QCommandLineOption settingsOption("settings",
        "Read an XML configuration for this run without changing saved settings.", "file");
    parser.addOption(settingsOption);
#ifdef GRSIM_BUILD_SMOKE_TEST
    QCommandLineOption smokeOption("smoke-test", "Check application resources, rendering and UDP, then exit.");
    parser.addOption(smokeOption);
#endif
    parser.process(a);
    ConfigWidget* config = nullptr;
    try {
        if (parser.isSet(settingsOption) && parser.value(settingsOption).isEmpty())
            throw std::runtime_error("Expected a nonempty settings filename");
#ifdef GRSIM_BUILD_SMOKE_TEST
        if (parser.isSet(smokeOption)) return runApplicationSmokeTest(a, parser.value(settingsOption));
#endif
        if (parser.isSet(settingsOption)) config = new ConfigWidget(false, parser.value(settingsOption));
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }

    MainWindow w(nullptr, config);
    if (parser.isSet(headlessOption)) {
        // enable headless mode
        w.hide();
        w.setIsGlEnabled(false);
    } else {
        // Run normally
        w.showMaximized();
    }
    return a.exec();
}
