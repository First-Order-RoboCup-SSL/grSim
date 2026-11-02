#ifndef SIMULATION_H
#define SIMULATION_H

#include <QObject>
#include <QThread>
#include <QElapsedTimer>
#include <QTimer>
#include <deque>
#include "scenesnapshot.h"

class SSLWorld;
class QUdpSocket;
class RoboCupSSLServer;

// Remote RobotSpecs can change while a GUI edit or its acknowledgement is queued.
struct SimulationSettingsEdit {
    SimulationSettings before, after;
    quint64 revision = 0;
    bool rebuild = false;
    void applyRobotChanges(SimulationSettings& settings) const;
};
Q_DECLARE_METATYPE(SimulationSettingsEdit)

class LagMonitor {
public:
    bool update(double pending, double step, qint64 now);
    void reset();
private:
    bool visible = false;
    qint64 transition = -1;
};

class SimulationWorker : public QObject {
    Q_OBJECT
public:
    explicit SimulationWorker(std::shared_ptr<SnapshotMailbox> mailbox);
public slots:
    void start(SimulationSettings settings);
    void configure(SimulationSettingsEdit edit);
    void act(SimulationAction action);
    void stop();
private slots:
    void tick();
signals:
    void settingsChanged(SimulationSettings settings, quint64 appliedRevision);
private:
    void rebuildWorld();
    void connectSockets(const SimulationSettings* previous = nullptr);
    void publish();
    void expireCommands();
    std::shared_ptr<SnapshotMailbox> mailbox;
    SimulationSettings settings;
    SSLWorld* world = nullptr;
    QTimer* timer = nullptr;
    QUdpSocket* commandSocket = nullptr;
    QUdpSocket* simControlSocket = nullptr;
    QUdpSocket* blueControlSocket = nullptr;
    QUdpSocket* yellowControlSocket = nullptr;
    QUdpSocket* blueStatusSocket = nullptr;
    QUdpSocket* yellowStatusSocket = nullptr;
    RoboCupSSLServer* visionServer = nullptr;
    QElapsedTimer clock, rateClock;
    qint64 lastTick = 0;
    double accumulator = 0, physicsHz = 0;
    quint64 generation = 0;
    quint64 appliedRevision = 0;
    int stepsThisSecond = 0;
    bool initialized = false;
    LagMonitor lag;
    TimingWindow stepTimings;
    TimingSummary recentStepTimings;
};

// GUI-facing interface. All operations are queued; only snapshots cross back.
class SimulationController : public QObject {
    Q_OBJECT
public:
    explicit SimulationController(const SimulationSettings& settings, QObject* parent = nullptr);
    ~SimulationController() override;
    SceneSnapshotPtr latest() const { return mailbox->latest(); }
    void configure(const SimulationSettings& settings, bool rebuild = false);
    void submit(const SimulationAction& action);
    void stop();
signals:
    void configureRequested(SimulationSettingsEdit edit);
    void actionRequested(SimulationAction action);
    void remoteSettingsChanged(SimulationSettings settings);
private:
    void receiveSettings(SimulationSettings settings, quint64 appliedRevision);
    std::shared_ptr<SnapshotMailbox> mailbox;
    QThread thread;
    SimulationWorker* worker;
    SimulationSettings guiSettings;
    quint64 nextRevision = 0;
    std::deque<SimulationSettingsEdit> pendingEdits;
};
#endif
