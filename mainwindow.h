#ifndef WORLDVIEWER_MAINWINDOW_H
#define WORLDVIEWER_MAINWINDOW_H

#include "CallbackSink.h"
#include "ui_MainWindow.h"
#include "part.h"

#include <RobSim/Collision/DebugDrawer.h>
#include <RobSim/Collision/DefaultCollisionWorld.h>
#include <RobSim/Math.h>
#include <RobSim/Robot/Path.h>
#include <RobSim/Simulation.h>
#include <RobSim/LoadRapidCode.h>

#include <OgreManualObject.h>

#include <QMainWindow>
#include <QPropertyAnimation>
#include <QScopedPointer>

#include <vector>

namespace Ogre
{
class SceneManager;
}

namespace RobSim
{
class Joint;
class Robot;
class TransformObject;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void loadWorld();
    void loadWorld(const QString &path);

    void loadCode();
    void loadCode(const QString &path);

    void loadModel();
    void loadModel(const QString &path);

    void chooseRobot(int index);
    void chooseJoint(int index);
    void chooseConfiguration(const QString &name);

    void changeJoint(int value);
    void changePathPosition(int value);
    void playOrPause();

    void toggleVisualisation(int state);
    void toggleCollision(int state);
    void toggleAabb(int state);

    void on_print_clicked();
    void on_path_clicked();
    void on_slice_clicked();
    void on_button_clicked();

    void updateIK();

    void setStart();
    void setGoal();
    void planPath();

private:
    void setupSimulation();

    void populateConfigurations(QComboBox *box);

    void processFile(const QString &path);

    void createSphere(RobSim::Vector3 position);
    void updateWeld();

    bool m_weld = false;
    RobSim::Vector3 m_last;
    Ogre::ManualObject *m_draw;

    QScopedPointer<Ui::MainWindow> m_ui;
    QScopedPointer<RobSim::DefaultCollisionWorld> m_collision;
    QScopedPointer<RobSim::DebugDrawer> m_collisionDrawer;
    QScopedPointer<RobSim::Simulation> m_simulation;

    CallbackSink::Buffer m_buffer;
    std::streambuf *m_cout;

    Ogre::SceneManager *m_manager;

    RobSim::Robot *m_robot = nullptr;
    RobSim::Joint *m_joint = nullptr;

    QScopedPointer<RobSim::Path> m_path;
    QPropertyAnimation* m_pathAnimation;

    RobSim::Vector m_startConfig;
    RobSim::Vector m_goalConfig;

    std::vector<RobSim::Robot *> m_robots;

    std::unique_ptr<Part> m_part = nullptr;
    RobSim::TransformObject *m_workpiece = nullptr;
};

#endif // WORLDVIEWER_MAINWINDOW_H
