#include "part.h"

#include <RobSim/Mesh.h>
#include <RobSim/Collision/CollisionSystem.h>
#include <RobSim/Visual/OgreMaterials.h>

#include <OgreManualObject.h>


Part::~Part()
{
}

Part::Part(std::string name)
    :name(std::move(name))
{
}

void Part::load(const std::string &path)
{
    m_mesh = RobSim::Mesh::load(path);
}

std::shared_ptr<RobSim::Mesh> Part::getMesh()
{
    return m_mesh;
}

void Part::process(RobSim::CollisionSystem *coll, RobSim::Vector3 position)
{
    // clear existing results
    m_slicePoints.clear();

    // create heights
    std::vector<float> heights;
    for(float z = 0; z < 500.0f; z+=5.0f)
    {
        heights.push_back(z);
    }

    // create angles
    std::vector<float> angles;
    for(float alpha = 0; alpha < 360.0f; alpha+=15.0f)
    {
        angles.push_back(alpha);
    }

    for (std::size_t z = 0; z < heights.size(); ++z)
    {
        for (std::size_t a = 0; a < angles.size(); ++a)
        {
             //create raytrace
             RobSim::Vector3 start(0, 0, heights[z]);
             RobSim::Vector3 end(0, 0, heights[z]);

             // move start out
             RobSim::Vector3 direction = RobSim::Vector3::UnitX();
             RobSim::AngleAxis rotation = RobSim::AngleAxis(RobSim::radians(angles[a] + heights[z]/100.0f*5.0f), RobSim::Vector3::UnitZ());
             direction = rotation.toRotationMatrix()*direction;
             start = start + direction * 500.0f;

             start+=position;
             end+=position;

             RobSim::RayCastResult result = coll->rayCast(start, end);
             std::cout << result.point.transpose() <<std::endl;
             if(result.hit) {
                 m_slicePoints.push_back(result.point);
             }
        }
    }
}

void Part::drawSlicePoints(Ogre::ManualObject *draw)
{
    draw->begin(RobSim::OgreMaterials::debug(), Ogre::RenderOperation::OT_POINT_LIST);
    draw->colour(Ogre::ColourValue::Red);
    for (std::size_t i = 0; i < m_slicePoints.size(); ++i)
    {
        Ogre::Vector3 point(m_slicePoints[i].x(), m_slicePoints[i].y(), m_slicePoints[i].z());
        draw->position(point);
    }
    draw->end();
}

