#include <QApplication>
#include <QFile>
#include <QTemporaryDir>
#include <cstdio>
#include "configwidget.h"

static bool check(bool passed, const char* description)
{
    std::printf("%s %s\n", passed ? "PASS" : "FAIL", description);
    return passed;
}

class InspectableConfig : public ConfigWidget {
public:
    using ConfigWidget::ConfigWidget;
    VarPtr worldSettings() const {
        for (const auto& item : world) {
            if (item->getName() == "Physics") {
                for (const auto& child : item->getChildren())
                    if (child->getName() == "World") return child;
            }
        }
        return {};
    }
};

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QTemporaryDir temporary;
    if (!temporary.isValid()) return 2;
    const QString path = temporary.filePath(".grsim.xml");
    QFile initial(path);
    if (!initial.open(QIODevice::WriteOnly) || initial.write("<VarXML/>") < 0) return 2;
    initial.close();
    auto physics = std::make_shared<VarList>("Physics");
    auto world = std::make_shared<VarList>("World");
    physics->addChild(world);
    world->addChild(std::make_shared<VarDouble>("Desired FPS", 7.0));
    world->addChild(std::make_shared<VarDouble>("ODE time step", 0.008));
    world->addChild(std::make_shared<VarBool>("Synchronize ODE with OpenGL", true));
    VarXML::write(physics, path.toStdString());
    bool passed = true;
    {
        int simulationEdits = 0;
        ConfigWidget config(true, path);
        passed = check(!config.ShowSkybox(),
                       "skybox defaults off when older settings omit the option") && passed;
        passed = check(config.DesiredFPS() == 7.0 && config.DeltaTime() == 0.008,
                       "legacy timing values load into the active controls") && passed;
        const auto settings = config.simulationSettings();
        passed = check(settings.DesiredFPS() == 7.0 && settings.DeltaTime() == 0.008,
                       "saved timestep reaches copied simulation settings") && passed;
        config.v_DesiredFPS->setDouble(13);
        config.v_DeltaTime->setDouble(0.01);
        QObject::connect(&config, &ConfigWidget::settingsEdited, &config, [&]() { ++simulationEdits; });
        config.v_ShowSkybox->setBool(true);
        emit config.v_ShowSkybox->wasEdited(config.v_ShowSkybox);
        QCoreApplication::processEvents();
        passed = check(simulationEdits == 0, "editing the skybox leaves the simulation configuration alone") && passed;
    }
    QFile saved(path);
    if (!saved.open(QIODevice::ReadOnly)) return 2;
    const QByteArray xml = saved.readAll();
    saved.close();
    passed = check(xml.count("name=\"Desired FPS\"") == 1 &&
                   xml.count("name=\"ODE time step\"") == 1 &&
                   !xml.contains("Render FPS Cap") && !xml.contains("Physics Time Step"),
                   "timing edits update the original keys without inactive duplicates") && passed;
    passed = check(!xml.contains("Synchronize ODE with OpenGL"),
                   "obsolete synchronization control is removed from saved settings") && passed;
    passed = check(xml.count("name=\"Show Skybox\"") == 1,
                   "skybox has one persistent settings entry") && passed;
    {
        ConfigWidget config(true, path);
        passed = check(config.DesiredFPS() == 13 && config.DeltaTime() == 0.01,
                       "edited timing values survive a settings round trip") && passed;
        passed = check(config.ShowSkybox(),
                       "enabled skybox survives a settings round trip") && passed;
        config.v_ShowSkybox->setBool(false);
    }
    {
        ConfigWidget config(false, path);
        passed = check(!config.ShowSkybox(),
                       "disabled skybox survives a settings round trip") && passed;
    }
    auto renamedPhysics = std::make_shared<VarList>("Physics");
    auto renamedWorld = std::make_shared<VarList>("World");
    renamedPhysics->addChild(renamedWorld);
    renamedWorld->addChild(std::make_shared<VarDouble>("Render FPS Cap", 5.0));
    renamedWorld->addChild(std::make_shared<VarDouble>("Physics Time Step (s)", 0.02));
    VarXML::write(renamedPhysics, path.toStdString());
    {
        ConfigWidget config(true, path);
        passed = check(config.DesiredFPS() == 5 && config.DeltaTime() == 0.02,
                       "intermediate renamed timing keys migrate into active controls") && passed;
    }
    renamedWorld->addChild(std::make_shared<VarDouble>("Desired FPS", 60.0));
    renamedWorld->addChild(std::make_shared<VarDouble>("ODE time step", 0.01));
    VarXML::write(renamedPhysics, path.toStdString());
    {
        ConfigWidget config(true, path);
        passed = check(config.DesiredFPS() == 60 && config.DeltaTime() == 0.01,
                       "stable timing keys take precedence over aliases even at the default FPS") && passed;
    }
    auto screenshotPhysics = std::make_shared<VarList>("Physics");
    auto screenshotWorld = std::make_shared<VarList>("World");
    screenshotPhysics->addChild(screenshotWorld);
    screenshotWorld->addChild(std::make_shared<VarDouble>("Render FPS Cap", 240.0));
    screenshotWorld->addChild(std::make_shared<VarDouble>("Physics Time Step (s)", 0.016667));
    screenshotWorld->addChild(std::make_shared<VarDouble>("Desired FPS", 60.0));
    screenshotWorld->addChild(std::make_shared<VarDouble>("ODE time step", 0.016670));
    screenshotWorld->addChild(std::make_shared<VarBool>("Synchronize ODE with OpenGL", false));
    VarXML::write(screenshotPhysics, path.toStdString());
    {
        InspectableConfig config(true, path);
        passed = check(config.DesiredFPS() == 60 && config.DeltaTime() == 0.016670,
                       "screenshot settings preserve this branch's active timing values") && passed;
        const auto settings = config.worldSettings();
        int fpsRows = 0, timestepRows = 0;
        bool aliases = false;
        if (settings) for (const auto& child : settings->getChildren()) {
            const auto name = child->getName();
            if (name == "Desired FPS") ++fpsRows;
            if (name == "ODE time step") ++timestepRows;
            if (name == "Render FPS Cap" || name == "Physics Time Step (s)" ||
                name == "Synchronize ODE with OpenGL") aliases = true;
        }
        passed = check(settings && fpsRows == 1 && timestepRows == 1 && !aliases,
                       "screenshot settings produce one FPS row and one timestep row in the UI tree") && passed;
    }
    return passed ? 0 : 1;
}
