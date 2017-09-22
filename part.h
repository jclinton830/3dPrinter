#ifndef PART_H
#define PART_H

#include <RobSim/Math.h>

#include <OgreManualObject.h>

#include <memory>

namespace RobSim
{
class Mesh;
class CollisionSystem;
}

class Part
{
public:
    ~Part();
    Part(std::string name);

    void load(const std::string &path);
    std::shared_ptr<RobSim::Mesh> getMesh();

    void process(RobSim::CollisionSystem *coll, RobSim::Vector3 position);
    void drawSlicePoints(Ogre::ManualObject *draw);

    std::vector<RobSim::Vector3> getSlicePoints();

    const std::string name;

private:
    std::shared_ptr<RobSim::Mesh> m_mesh;
    std::vector<RobSim::Vector3> m_slicePoints;
};

#endif // PART_H
