#include "abbinversekinematics.h"

#include <RobSim/Robot/Joint.h>
#include <RobSim/Robot/JointSet.h>
#include <RobSim/Robot/Tool.h>

#include <iostream>

using RobSim::JointSet;
using RobSim::Matrix4;
using RobSim::Matrix3;
using RobSim::Vector;
using RobSim::Vector3;
using RobSim::WorldObject;

AbbInverseKinematics::AbbInverseKinematics(JointSet *joints, const Params &a, const Params &d) :
    RobSim::InverseKinematics(joints), a(a), d(d)
{
}

int AbbInverseKinematics::getNumConfigurations() const
{
    return 8;
}

bool AbbInverseKinematics::solve(Vector &q, const Matrix4 &pose, int config, const Vector &prev)
{
    q.resize(6);


    if (joints->getSize() != 6) {
        throw std::exception("The robot must have6 joints");
    }

    //std::cout << "IK solve: " << config << " , " << prev.transpose() << std::endl;

    // Gets the ith joint (one indexed).
    auto joint = [&](std::size_t i) { return (*joints)[joints->getSize() - (6 - i) - 1]; };

    Matrix4 Ttool0_tcp = joints->getTool() ? joints->getTool()->getTcpTransform() : Matrix4::Identity();
    Matrix4 Tworld_base = joint(1)->getParent()->getPose();
    Matrix4 Tbase_tcp = Tworld_base.inverse()*pose;

    Matrix4 local = Tbase_tcp * Ttool0_tcp.inverse();
    //std::cout << local << std::endl;


    // Offset negative joint 6 to get at the wrist.
    Matrix4 wristOffset = Matrix4::Identity();
    wristOffset(2, 3) = d[5];

    local *= wristOffset.inverse();

    //std::cout << wristOffset << std::endl;
    //std::cout << local << std::endl;

    if (a[3] != 0 || a[4] != 0 || a[5] != 0) {
        throw std::exception("The robot wrist must be spherical");
    }

    Vector3 O = local.block<3, 1>(0, 1);
    Vector3 A = local.block<3, 1>(0, 2);
    Vector3 P = local.block<3, 1>(0, 3);

    int n1 = (config & 1) ? 1 : -1;
    int n2 = (config & 1 << 1) ? 1 : -1;
    int n4 = (config & 1 << 2) ? 1 : -1;

    // Solve for q(1).
    q(0) = atan2(P.y(), P.x()) - (n1 == 1 ? - 0 : ROBSIM_PI);

    if (q(0) < -ROBSIM_PI) {
        q(0) = q(0) + 2 * ROBSIM_PI;
    }

    //std::cout << "1: " << q(0) << std::endl;

/*    if (prev.size() > 0) {
        if (q(0) - prev(0) > ROBSIM_PI) {
            q(0) -= 2 * ROBSIM_PI;
        } else if (q(0) - prev(0) < -ROBSIM_PI) {
            q(0) += 2 * ROBSIM_PI;
        }
    }*/ //not sure if this is required for a parallel link robot

    // Solve for q(2).
    float v114 = P.x() * cos(q(0)) - a[0] + P.y() * sin(q(0));  //1070
    float v124 = d[0] - P.z();                                  //-1040
    float r = sqrt(v114 * v114 + v124 * v124);                  //1492
    float Psi = ((a[1] * a[1] - d[3] * d[3] - a[2] * a[2] + v114 * v114 + v124 * v124) / (2.0 * a[1] * r));

    if (Psi < -1.0 || Psi > 1.0) {
        return false;
    }

    q(1) = -atan2(v114, v124) + n2 * acos(Psi) + (n1 == 1 ? ROBSIM_PI : -ROBSIM_PI);

    if (q(1) < -ROBSIM_PI) {
        q(1) += static_cast<float>(2 * ROBSIM_PI);
    } else if (q(1) > ROBSIM_PI) {
        q(1) -= static_cast<float>(2 * ROBSIM_PI);
    }

    //std::cout << "2: " << q(1) << std::endl;

    //q(1) = q(1) - ROBSIM_PI / 2;

    // Solve for q(3).
    float v214 = sin(q(1)) * (P.x() * cos(q(0)) - a[0] + P.y() * sin(q(0))) - a[1] + cos(q(1)) * (P.z() - d[0]);
    float v224 = cos(q(1)) * (P.x() * cos(q(0)) - a[0] + P.y() * sin(q(0))) - sin(q(1)) * (P.z() - d[0]);



    q(2) = atan2(a[2], d[3]) - atan2(v214, v224);


    if (q(2) < -ROBSIM_PI) {
        q(2) += static_cast<float>(2 * ROBSIM_PI);
    } else if (q(2) > ROBSIM_PI) {
        q(2) -= static_cast<float>(2 * ROBSIM_PI);
    }

    // Solve for q(4).
    float v313 =
          cos(q(2)) * (A.z() * cos(q(1)) + sin(q(1)) * (A.x() * cos(q(0)) + A.y() * sin(q(0))))
        - sin(q(2)) * (A.z() * sin(q(1)) - cos(q(1)) * (A.x() * cos(q(0)) + A.y() * sin(q(0))));
    float v323 = A.x() * sin(q(0)) - A.y() * cos(q(0));

    q(3) = atan2((n4 * v323), (n4 * v313));

    // Ensure same configuration.
    if (prev.size() > 0) {
        if (q(3) - prev(3) > ROBSIM_PI) {
            q(3) -= 2 * ROBSIM_PI;
        } else if (q(3) - prev(3) < -ROBSIM_PI) {
            q(3) += 2 * ROBSIM_PI;
        }
    }


    // Solve for q(5).
    float v413 =
          sin(q(3)) * (A.y() * cos(q(0)) - A.x() * sin(q(0)))
        - cos(q(3)) * (cos(q(2)) * (A.z() * cos(q(1)) + sin(q(1)) * (A.x() * cos(q(0)) + A.y() * sin(q(0))))
        - sin(q(2)) * (A.z() * sin(q(1)) - cos(q(1)) * (A.x() * cos(q(0)) + A.y() * sin(q(0)))));

    float v423 =
          cos(q(2)) * (A.z() * sin(q(1)) - cos(q(1)) * (A.x() * cos(q(0)) + A.y() * sin(q(0))))
        + sin(q(2)) * (A.z() * cos(q(1)) + sin(q(1)) * (A.x() * cos(q(0)) + A.y() * sin(q(0))));

    q(4) = atan2(v413, -v423);

    if (q(4) < -ROBSIM_PI) {
        q(4) += static_cast<float>(2 * ROBSIM_PI);
    }

    // Solve for q(6).
    float v512 =
            cos(q(4)) * (cos(q(3)) * (cos(q(2)) * (O.z() * cos(q(1)) + sin(q(1)) * (O.x() * cos(q(0)) + O.y() * sin(q(0))))
                - sin(q(2)) * (O.z() * sin(q(1)) - cos(q(1)) * (O.x() * cos(q(0)) + O.y() * sin(q(0)))))
                - sin(q(3)) * (O.y() * cos(q(0)) - O.x() * sin(q(0))))
                - sin(q(4)) * (cos(q(2)) * (O.z() * sin(q(1)) - cos(q(1)) * (O.x() * cos(q(0)) + O.y() * sin(q(0))))
                + sin(q(2)) * (O.z() * cos(q(1)) + sin(q(1)) * (O.x() * cos(q(0)) + O.y()*sin(q(0)))));


    float v522 =
              sin(q(3)) * (cos(q(2)) * (O.z() * cos(q(1)) + sin(q(1)) * (O.x() * cos(q(0)) + O.y() * sin(q(0))))
            - sin(q(2)) * (O.z() * sin(q(1)) - cos(q(1)) * (O.x() * cos(q(0)) + O.y() * sin(q(0)))))
            + cos(q(3)) * (O.y() * cos(q(0)) - O.x() * sin(q(0)));

    //std::cout << v512 << std::endl;
    //std::cout << v522 << std::endl;

    q(5) = atan2(v512, v522);

    if (q(5) < - ROBSIM_PI) {
        q(5) += static_cast<float>(2 * ROBSIM_PI);
    }

    // Ensure same configuration.
    if (prev.size() > 0) {
        if (q(5) - prev(5) > ROBSIM_PI) {
            q(5) -= 2 * ROBSIM_PI;
        } else if (q(5) - prev(5) < -ROBSIM_PI) {
            q(5) += 2 * ROBSIM_PI;
        }
    }

    if (parallel) {
        q(2) += q(1);
    }

    if (!joints->isValid(q)) {
        return false;
    }

    if (abs(q[4]) < (float) 10 / 180 * ROBSIM_PI) {
        return false;
    }

    return true;
}

//int AbbInverseKinematics::getCfg(RobSim::Vector q)
//{
//    Matrix4 pose = joints->getPose(q);

//    for (std::size_t cfg = 1; cfg <= 8; cfg++) {
//        RobSim::Vector candidate;
//        joints->getIk()->solve(candidate, pose, cfg);
//        if ((q-candidate).norm() < 0.001) {
//            return cfg;
//        }
//    }

//    return -1;
//}
