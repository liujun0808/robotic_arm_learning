#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/Goal.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include <ompl/config.h>
#include <Eigen/Dense>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "ompl/base/goals/GoalState.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Pose {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

static double orientationErrorAngle(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2) {
    // smallest angle to rotate from q1 to q2
    Eigen::Quaterniond dq = q1.conjugate() * q2;
    dq.normalize();
    // angle = 2 * acos(|w|)
    double w = std::max(-1.0, std::min(1.0, dq.w()));
    return 2.0 * std::acos(std::abs(w));
}

// 自定义状态有效性检查器（包含碰撞检测和关节限位）
class ValidityChecker : public ob::StateValidityChecker {
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) 
        : ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* state) const override {
        const auto* jointState = state->as<ob::RealVectorStateSpace::StateType>();
        
        // 示例：检查关节角度限位 [-π, π]
        for (int i = 0; i < 7; ++i) {
            double angle = (*jointState)[i];
            if (angle < -M_PI || angle > M_PI) 
                return false;
        }

        // TODO: 添加实际碰撞检测逻辑
        // 例如使用FCL或自定义几何模型
        
        return true;
    }
};



class planner
{
private:

    
public:
    planner();
    bool plan(Eigen::VectorXd &startJoints,Eigen::VectorXd &targetJoints);
    std::vector<std::vector<double>> trajectory;
};

