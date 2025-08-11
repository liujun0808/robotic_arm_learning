#include "planner.h"

planner::planner()
{
    // trajectory.resize(0);
}

bool planner::plan(Eigen::VectorXd &startJoints,Eigen::VectorXd &targetJoints){
    // -----------------planner--------------
    auto space(std::make_shared<ob::RealVectorStateSpace>(7)); // 创建向量空间
    ob::RealVectorBounds bounds(7);
    bounds.setLow(0,-2.96706);bounds.setLow(1,-2.0944);bounds.setLow(2,-2.96706);
    bounds.setLow(3,-2.0944);bounds.setLow(4,-2.96706);bounds.setLow(5,-2.0944);bounds.setLow(6,-3.05433);
    bounds.setHigh(0,2.96706);bounds.setHigh(1,2.0944);bounds.setHigh(2,2.96706);
    bounds.setHigh(3,2.0944);bounds.setHigh(4,2.96706);bounds.setHigh(5,2.0944);bounds.setHigh(6,3.05433);
    space->setBounds(bounds);
    // og::SimpleSetup ss(space);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    // std::cout<<"1"<<"\n";
    // ss.setStateValidityChecker(std::make_shared<ArmStateValidityChecker>(ss.getSpaceInformation()));
        // 2. 设置状态有效性检查
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));
    // std::cout<<"2"<<"\n";
    si->setup();
    auto setup = std::make_shared<og::SimpleSetup>(si);

    ob::ScopedState<> start(space);
    ob::ScopedState<> target(space);
    for (size_t i = 0; i < 7; i++)
    {
        start[i] = startJoints[i];
        target[i] = targetJoints[i];
    }
    setup->setStartAndGoalStates(start,target);

    auto planner = std::make_shared<og::RRTConnect>(si);
    setup->setPlanner(planner);
    // std::cout<<"3"<<"\n";


    // double posTol = 0.01;
    // double oriTol = 0.08;
    // auto goal = std::make_shared<WorkspaceGoal>(ss.getSpaceInformation(),targetPose,posTol,oriTol,arm_);

    // // 设置起始状态与目标
    // ss.setStartState(start);
    // ss.setGoal(goal);

    // // 选择规划器
    // auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    // ss.setPlanner(planner);

    // 运行规划
    double solveTime = 10.0;
    ob::PlannerStatus solved = setup->solve(solveTime);
    // std::cout<<"4"<<"\n";
    trajectory.clear();
    if (solved)
    {
        std::cout<<"Found solution in "<<solveTime<<"s\n";
        og::PathGeometric path = setup->getSolutionPath();
        std::cout<<"Number of waypoints: "<<path.getStateCount()<<"\n";
        // 简化与差值平滑轨迹
        og::PathSimplifier simplifier(setup->getSpaceInformation());
        simplifier.simplifyMax(path);
        path.interpolate(50);
        std::cout << "After interpolation: " << path.getStateCount() << " points\n";
        trajectory.resize(path.getStateCount());
        // std::cout<<"5"<<path.getStateCount()<<"\n";
        for (size_t i = 0; i < path.getStateCount(); i++)
        {
            const ob::RealVectorStateSpace::StateType *s =  path.getState(i)->as<ob::RealVectorStateSpace::StateType>(); // 得到第i个轨迹点的 关节状态
            std::vector<double> q(7);
            for (size_t j = 0; j < 7; j++)
            {
                q[j] = s->values[j];
            }
            trajectory[i] = q;
        }
        // std::cout<<"6"<<"\n";
        return true;
    }else{
        std::cout << "No solution found within " << solveTime << " s\n";
        return false;
    }
}