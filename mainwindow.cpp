#include "MainWindow.h"
#include "WorldViewerException.h"

#include "abbinversekinematics.h"

#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreViewport.h>

#include <QDebug>
#include <QException>
#include <QFileDialog>
#include <QMessageBox>
#include <QtConcurrent>


#include <RobSim/Collision/CollisionSystem.h>
#include <RobSim/Collision/CollisionComponent.h>
#include <RobSim/Math.h>
#include <RobSim/Planning/PathSmoother.h>
#include <RobSim/Planning/PlanningSpace.h>
#include <RobSim/Planning/Prm.h>
#include <RobSim/Robot/LinearPath.h>
#include <RobSim/Robot/Joint.h>
#include <RobSim/Robot/JointPath.h>
#include <RobSim/Robot/LinearPath.h>
#include <RobSim/Robot/JointSet.h>
#include <RobSim/Robot/Robot.h>
#include <RobSim/Visual/OgreMaterials.h>
#include <RobSim/Visual/VisualComponent.h>
#include <RobSim/Visual/VisualSystem.h>
#include <RobSim/Loader/SimulationLoader.h>
#include <RobSim/World.h>

#include <cassert>

//robot IK
static const std::array<float, 6> IRB_1410_A = {150, 600, 120, 0, 0, 0};
static const std::array<float, 6> IRB_1410_D = {475, 0, 0, (805-86.5), 0, (73.5+13)};

static const std::array<float, 6> IRB_4400_A = {200, 890, 150, 0, 0, 0};
static const std::array<float, 6> IRB_4400_D = {680, 0, 0, 880, 0, 140};

static const std::array<float, 6> IRB_6660_A = {300, 700, 280, 0, 0, 0};
static const std::array<float, 6> IRB_6660_D = {814.5, 0, 0, 893, 0, 200};


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_collision(new RobSim::DefaultCollisionWorld)
{
    m_ui->setupUi(this);

    m_ui->sidebarSplitter->setStretchFactor(0, 1);
    m_ui->sidebarSplitter->setStretchFactor(1, 0);

    m_ui->messagesSplitter->setStretchFactor(0, 1);
    m_ui->messagesSplitter->setStretchFactor(1, 0);

    m_ui->viewer->createDefaultLights();
    m_ui->viewer->getViewport()->setBackgroundColour(Ogre::ColourValue(0.75f, 0.75f, 0.75f));

    m_manager = m_ui->viewer->getManager();

    m_collisionDrawer.reset(new RobSim::DebugDrawer(m_manager));
    m_collision->get()->setDebugDrawer(m_collisionDrawer.data());
    m_ui->viewer->getRoot()->addFrameListener(m_collisionDrawer.data());

    connect(m_ui->viewer, &RobSim::OgreWidget::rendered, this, &MainWindow::updateIK);

    connect(m_ui->loadWorld, SIGNAL(clicked()), this, SLOT(loadWorld()));

    //Upload code and 3D mesh
    connect(m_ui->loadCode, SIGNAL(clicked()), this, SLOT(loadCode()));
    connect(m_ui->loadModel, SIGNAL(clicked()), this, SLOT(loadModel()));

    connect(m_ui->robot, SIGNAL(activated(int)), this, SLOT(chooseRobot(int)));
    connect(m_ui->joint, SIGNAL(activated(int)), this, SLOT(chooseJoint(int)));
    connect(m_ui->configuration, SIGNAL(activated(QString)), this, SLOT(chooseConfiguration(QString)));

    connect(m_ui->value, &QSlider::valueChanged, this, &MainWindow::changeJoint);
    connect(m_ui->pathSlider, &QSlider::valueChanged, this, &MainWindow::changePathPosition);
    connect(m_ui->play, &QPushButton::clicked, this, &MainWindow::playOrPause);

    m_pathAnimation = new QPropertyAnimation(m_ui->pathSlider, "value", this);
    m_pathAnimation->setStartValue(m_ui->pathSlider->minimum());
    m_pathAnimation->setEndValue(m_ui->pathSlider->maximum());
    m_pathAnimation->setDuration(5000);
    m_pathAnimation->setLoopCount(-1);

    connect(m_ui->viewVisualisation, &QCheckBox::stateChanged, this, &MainWindow::toggleVisualisation);
    connect(m_ui->viewCollision, &QCheckBox::stateChanged, this, &MainWindow::toggleCollision);
    connect(m_ui->viewAabb, &QCheckBox::stateChanged, this, &MainWindow::toggleAabb);

    connect(m_ui->setStart, &QPushButton::clicked, this, &MainWindow::setStart);
    connect(m_ui->setGoal, &QPushButton::clicked, this, &MainWindow::setGoal);
    connect(m_ui->plan, &QPushButton::clicked, this, &MainWindow::planPath);

    m_draw = m_ui->viewer->getManager()->createManualObject();
    m_ui->viewer->getManager()->getRootSceneNode()->createChildSceneNode()->attachObject(m_draw);


    connect(m_ui->viewer, &RobSim::OgreWidget::rendering, [this]()
    {
        if (m_simulation.isNull() || !m_simulation->getSystem<RobSim::CollisionSystem>())
        {
            return;
        }

        m_ui->collision->setChecked(m_simulation->getSystem<RobSim::CollisionSystem>()->isColliding());
        m_collision->get()->debugDrawWorld();
        updateWeld();
    });

    m_buffer.open(CallbackSink([this](const std::string &str)
    {
        m_ui->messages->moveCursor(QTextCursor::End);
        m_ui->messages->insertPlainText(str.c_str());
    }));
    m_cout = std::cout.rdbuf(&m_buffer);
}

MainWindow::~MainWindow()
{
    std::cout.rdbuf(m_cout);
}

void MainWindow::loadWorld()
{
    QFileInfo fi("C:/Development/3dPrinter/world/mrl.lua");
    QString file = fi.absoluteFilePath();
    if (!file.isEmpty())
    {
        loadWorld(file);
    }
}

void MainWindow::loadWorld(const QString &path)
{
    auto watcher = new QFutureWatcher<void>(this);

    m_simulation.reset(new RobSim::Simulation);
    m_simulation->setBullet(m_collision->get());
    m_simulation->setOgre(m_manager);

    m_ui->loadWorld->setEnabled(false);
    m_ui->loadWorld->setText("Loading...");

    connect(watcher, &QFutureWatcher<void>::finished, [this, watcher]()
    {
        try
        {
            watcher->waitForFinished();

            m_simulation->initialise();
            m_ui->viewer->update();
            m_ui->viewer->zoomExtents();

            setupSimulation();
        }
        catch (const QException& e)
        {
            QMessageBox::critical(this, "Error Loading World", e.what());
        }

        m_ui->loadWorld->setEnabled(true);
        m_ui->loadWorld->setText("Load World");
    });

    watcher->setFuture(QtConcurrent::run([this, path]()
    {
        try
        {
            m_simulation->load(path.toStdString());

            m_ui->viewer->zoomExtents();
        }
        catch (const std::exception &e)
        {
            throw WorldViewerException(e.what());
        }
    }));
}

void MainWindow::setupSimulation()
{
    m_robots = m_simulation->getWorld()->get<RobSim::Robot>();

    m_ui->robot->clear();
    m_ui->robot->setEnabled(true);

    m_ui->joint->clear();
    m_ui->joint->setEnabled(false);
    m_ui->value->setEnabled(false);

    toggleVisualisation(m_ui->viewVisualisation->checkState());
    toggleCollision(m_ui->viewCollision->checkState());

    for (auto robot : m_robots)
    {
        m_ui->robot->addItem(robot->getName().c_str());

        // Jump all robots to home initially.
        if (robot->getHome())
        {
            robot->getHome()->apply();
        }
    }

    // Set IK for robots.
    for (auto *robot : m_simulation->getWorld()->get<RobSim::Robot>())
    {
        RobSim::JointSet *joints = robot->getJoints().getSize() == 6 ? &robot->getJoints() : robot->getJointSet("Arm");

        std::string name = robot->getName();
        std::string id = robot->getId();

        std::unique_ptr<AbbInverseKinematics> ik;

        if (name == "ABB IRB 1410")
        {
            ik = std::make_unique<AbbInverseKinematics>(joints, IRB_1410_A, IRB_1410_D);
            ik->parallel = true;
        }
        else if (name == "ABB IRB 4400")
        {
            ik = std::make_unique<AbbInverseKinematics>(joints, IRB_4400_A, IRB_4400_D);
            ik->parallel = true;
        }
        else if (name == "ABB IRB 6660")
        {
            ik = std::make_unique<AbbInverseKinematics>(joints, IRB_6660_A, IRB_6660_D);
            ik->parallel = true;
        }
        else
        {
            QMessageBox::warning(this, "", "Unrecognised robot name");
        }

        if (joints)
        {
            joints->setIk(std::move(ik));
        }
    }

    if (!m_robots.empty())
    {
        chooseRobot(0);
    }

    // set workpiece

    m_workpiece = m_simulation->getWorld()->create<RobSim::TransformObject>("Workpiece");
    m_workpiece->addComponent<RobSim::VisualComponent>();

    m_workpiece->setPosition(1000, -4000, 800);
}

void MainWindow::chooseRobot(int index)
{
    assert(index < m_robots.size());

    m_ui->joint->clear();
    m_ui->joint->setEnabled(true);

    m_robot = m_robots.at(index);

    populateConfigurations(m_ui->configuration);

    for (auto joint : m_robot->getJoints())
    {
        m_ui->joint->addItem(joint->getName().c_str());
    }

    if (m_robot->getJoints().getSize() > 0)
    {
        chooseJoint(0);
    }
}

void MainWindow::populateConfigurations(QComboBox *box)
{
    box->clear();
    box->setEnabled(true);
    box->addItem(QString());

    for (auto &conf : m_robot->getConfigurations())
    {
        box->addItem(conf->getName().c_str());
    }
}

void MainWindow::chooseJoint(int index)
{
    assert(index < m_robot->getJoints().getSize());

    m_joint = m_robot->getJoints()[index];

    auto lim = m_joint->getLimits();
    auto val = m_joint->getValue();

    m_ui->valueLabel->setText(QString::number(val));
    m_ui->minLabel->setText(QString::number(lim.first));
    m_ui->maxLabel->setText(QString::number(lim.second));

    m_ui->value->setValue((val - lim.first) / (lim.second - lim.first) * 1000.0f);
    m_ui->value->setEnabled(true);
}

void MainWindow::chooseConfiguration(const QString &name)
{
    if (!name.isEmpty())
    {
        m_robot->getConfiguration(name.toStdString())->apply();
    }
}

void MainWindow::changeJoint(int slider)
{
    auto lim = m_joint->getLimits();
    auto value = lim.first + (slider / 1000.0f) * (lim.second - lim.first);

    m_ui->valueLabel->setText(QString::number(value));
    m_joint->setValue(value);
}

void MainWindow::changePathPosition(int value)
{
    if (!m_path.isNull())
    {
        m_path->apply(value / 1000.0f);
    }
}

void MainWindow::playOrPause()
{
    switch (m_pathAnimation->state())
    {
        case QPropertyAnimation::Stopped: m_pathAnimation->start(); break;
        case QPropertyAnimation::Paused: m_pathAnimation->resume(); break;
        case QPropertyAnimation::Running: m_pathAnimation->pause(); break;
    }

    m_ui->play->setIcon(QIcon(m_pathAnimation->state() == QPropertyAnimation::Running
                              ? ":/icons/pause.png"
                              : ":/icons/play.png"));
}

void MainWindow::toggleVisualisation(int state)
{
    auto system = m_simulation->getWorld()->getSystemManager().get<RobSim::VisualSystem>();

    if (system)
    {
        system->setVisible(state == Qt::Checked);
    }
}

void MainWindow::toggleCollision(int state)
{
    if (state == Qt::Checked)
    {
        m_collisionDrawer->setDebugMode(m_collisionDrawer->getDebugMode() | btIDebugDraw::DBG_DrawWireframe);
    }
    else
    {
        m_collisionDrawer->setDebugMode(m_collisionDrawer->getDebugMode() & ~btIDebugDraw::DBG_DrawWireframe);
    }
}

void MainWindow::toggleAabb(int state)
{
    if (state == Qt::Checked)
    {
        m_collisionDrawer->setDebugMode(m_collisionDrawer->getDebugMode() | btIDebugDraw::DBG_DrawAabb);
    }
    else
    {
        m_collisionDrawer->setDebugMode(m_collisionDrawer->getDebugMode() & ~btIDebugDraw::DBG_DrawAabb);
    }
}

void MainWindow::setStart()
{
    if (m_robot)
    {
        m_startConfig = m_robot->getJoints().getJointValues();
    }
}

void MainWindow::setGoal()
{
    if (m_robot)
    {
        m_goalConfig = m_robot->getJoints().getJointValues();
    }
}

void MainWindow::planPath()
{
    if (!m_startConfig.size() || !m_goalConfig.size())
    {
        QMessageBox::critical(this,
                              "Invalid Configuration",
                              "Both a start and goal configuration must be selected");
        return;
    }

    RobSim::PlanningSpace pspace(&m_robot->getJoints(), m_simulation->getSystem<RobSim::CollisionSystem>());
    RobSim::Prm prm(&pspace);
    prm.setStart(m_startConfig);
    prm.setGoal(m_goalConfig);

    if (!prm.plan())
    {
        QMessageBox::critical(this, "Planning Failure", "A path could not be planned");
        return;
    }

    RobSim::PathSmoother smoother(static_cast<RobSim::JointPath *>(prm.getPath().get()), &pspace);
    m_path.reset(smoother.smooth().release());

    m_path.reset(prm.getPath().release());
    m_ui->pathSlider->setEnabled(true);
    m_ui->play->setEnabled(true);
}

void MainWindow::on_print_clicked()
{
    m_weld = !m_weld;
}

void MainWindow::updateIK()
{
    m_ui->ik->clear();

    if (!m_robot) {
        return;
    }

    auto *joints = m_robot->getJoints().getSize() == 6 ? &m_robot->getJoints() : m_robot->getJointSet("Arm");
    auto *ik = joints->getIk();

    if (!ik) {
        return;
    }

    for (int i = 1; i <= ik->getNumConfigurations(); ++i)
    {
        auto *item = new QTreeWidgetItem(m_ui->ik);
        item->setText(0, QString::number(i));

        RobSim::Vector q;
        const RobSim::Matrix4 &pose = joints->getTcp()->getPose();

        if (ik->solve(q, pose, i))
        {
            QStringList values;

            for (int j = 0; j < q.size(); ++j)
            {
                values.push_back(QString::number(q(j), 'f', 2));
            }

            item->setText(1, values.join(", "));
        }
        else
        {
            item->setText(1, "invalid");
        }
    }
}

void MainWindow::on_path_clicked()
{
    RobSim::Matrix4 p = RobSim::Matrix4::Identity();
    p.topRightCorner<3, 1>() = RobSim::Vector3(1500, 0, 500);
    p.topLeftCorner<3, 3>() = RobSim::AngleAxis(M_PI, RobSim::Vector3::UnitX()).toRotationMatrix();

    RobSim::Matrix4 pb = p;
    pb(0, 3) = 1250;

    auto *path = new RobSim::LinearPath(m_robot->getJointSet("Arm"), 1);
    path->addPoint(p);
    path->addPoint(pb);

    m_path.reset(path);

    m_ui->pathSlider->setEnabled(true);
    m_ui->play->setEnabled(true);
}

void MainWindow::updateWeld()
{
    if(!m_weld) {
        return; //if not welding return;
    }
    RobSim::Robot *robot = m_robots[0];
    RobSim::Matrix4 pose = robot->getTcp()->getPose();

    if ((pose.topRightCorner<3, 1>() - m_last).norm() < 5.0f) {
        return;
    }
    createSphere(pose.topRightCorner<3, 1>());
    m_last = pose.topRightCorner<3, 1>();
}

void MainWindow::createSphere(RobSim::Vector3 position)
{
    RobSim::TransformObject *weld;
    weld = m_simulation->getWorld()->create<RobSim::TransformObject>("Weld");
    weld->addComponent<RobSim::VisualComponent>();
    weld->setPosition(position.x(), position.y(), position.z());

    RobSim::WorldObject *object = m_simulation->getWorld()->create<RobSim::WorldObject>("Sphere");
    weld->addChild(object);

    auto collisionComponent = new RobSim::CollisionComponent;
    collisionComponent->addSphere(5.0f);

    object->addComponent(collisionComponent);

    m_draw->begin(RobSim::OgreMaterials::debug(), Ogre::RenderOperation::OT_POINT_LIST);
    m_draw->position(Ogre::Vector3(position.x(), position.y(), position.z()));
    m_draw->colour(Ogre::ColourValue::White);
    m_draw->end();
}

void MainWindow::loadCode()
{
    //auto file = QFileDialog::getOpenFileName(this, "Select RAPID File", "", "RAPID Code (*.txt)");

    QFileInfo fi("C:/Development/3dPrinter/RAPID code/CURVE_THING_JOB.txt");
    QString file = fi.absoluteFilePath();
    if (!file.isEmpty())
    {
        loadCode(file);
    }
}

void MainWindow::loadCode(const QString &path)
{
    m_ui->loadCode->setEnabled(false);
    m_ui->loadCode->setText("Loading...");

    processFile(path);

    m_ui->loadCode->setEnabled(true);
    m_ui->loadCode->setText("Load Code");
}

void MainWindow::processFile(const QString &path)
{
    QFile file(path);
    if(!file.open(QIODevice::ReadOnly )) {
        QMessageBox::information(0, "error something went wrong", file.errorString());
    }
    QString line;
    QTextStream in(&file);
    QStringList Data;
    QRegExp rx("(\\+?\\-?\\d*\\.?\\d+)");
    int pos = 0;
    QStringList posData;
    QVector<QString> positionData;


    while(!in.atEnd())
    {
        //reading line by line
        line = in.readLine();
        m_ui->messages->insertPlainText(line + "\n");
        if (line == "PROC Path_10()")
            break;

        //line process
        Data = line.split("=");
        if (Data.size() < 2)
            continue;
        while ((pos = rx.indexIn(Data.at(1), pos)) != -1)
        {

            posData << rx.cap(1);
            pos += rx.matchedLength();

            positionData = posData.toVector();
        }
        for (int i = 0; i<=2; ++i)
        {
            m_ui->messages->insertPlainText(positionData[i] + " ");
            if (i == 2)
            {
                m_ui->messages->insertPlainText("\n");
            }
        }

    }
    file.close();

}

void MainWindow::loadModel()
{
    auto file = QFileDialog::getOpenFileName(this, "Select 3D Model", "", "3D Model (*.stl)");
    if (!file.isEmpty())
    {
        loadModel(file);
    }
}

void MainWindow::loadModel(const QString &path)
{
    m_ui->loadModel->setEnabled(false);
    m_ui->loadModel->setText("Loading...");

    if (!m_workpiece) {
        return;
    }

    m_part = std::make_unique<Part>("part");
    m_part->load(path.toStdString());

    auto object = m_simulation->getWorld()->create<RobSim::WorldObject>("part");
    m_workpiece->addChild(object);

    auto collisionComponent = new RobSim::CollisionComponent;
    collisionComponent->addMesh(m_part->getMesh());

    object->addComponent<RobSim::VisualComponent>(m_part->getMesh());
    object->addComponent(collisionComponent);
}

void MainWindow::on_slice_clicked()
{
    if(!m_part) {
        return;
    }

    m_part->process(m_simulation->getSystem<RobSim::CollisionSystem>(), m_workpiece->getPosition());
    m_part->drawSlicePoints(m_draw);
}
