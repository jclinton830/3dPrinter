#ifndef ABBPARALLELINVERSEKINEMATICS_H
#define ABBPARALLELINVERSEKINEMATICS_H

#include <RobSim/Robot/InverseKinematics.h>

#include <array>


class AbbInverseKinematics : public RobSim::InverseKinematics
{
public:
    typedef std::array<RobSim::Real, 6> Params;

    AbbInverseKinematics(RobSim::JointSet *joints, const Params &a, const Params &d);

    const Params a;
    const Params d;

    int getNumConfigurations() const override;

    bool solve(RobSim::Vector &result,
               const RobSim::Matrix4 &pose,
               int config,
               const RobSim::Vector &prev = RobSim::Vector()) override;

    bool parallel = false;

//    int getCfg(RobSim::Vector q) override;
};


#endif // ABBPARALLELINVERSEKINEMATICS_H
