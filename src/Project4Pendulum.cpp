///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <cmath>
#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 0;
    }

    void project(const ompl::base::State * state , Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
    }
};

bool isValidStatePendulum(ompl::control::SpaceInformation *si, ompl::base::state *state){
      return si->satifiesBounds(state);
}

void pendulumODE(const ompl::control::ODESolver::StateType &/* q */, const ompl::control::Control * control,
                 ompl::control::ODESolver::StateType &/* qdot */)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // TODO: Create and setup the pendulum's state space, control space, validity checker, everything you need for
    // planning.

    // State Space
    auto theta(std::make_shared<ompl::base::SO2StateSpace>());
    auto omega(std::make_shared<ompl::base::RealVectorStateSpace>(1));
    ompl::base::RealVectorBounds wbounds(1);
    wbounds.setLow(-10);
    wbounds.setHigh(10);
    omega->as<ompl::base::RealVectorStateSpace>()->setBounds(wbounds);

    ompl::base::StateSpacePtr space;
    space = theta + omega;

    // Control Space
    ompl::control::ControlSpacePtr cSpace(new ompl::control::RealVectorControlSpace(space, 1));

    // Set the bounds for Control SpaceInformation
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cSpace->as<ompl::base::RealVectorControlSpace>()->setBounds(cbounds);


    ompl::control::SimpleSetupPtr ss(cSpace);

    ss.setStateValidityChecker(std::bind(isValidStatePendulum, ss->getSpaceInformation().get(), _1);

    // Propagation Routine


    //Start and Goal States
    ompl::base::ScopedState<> start(space);
    start[0] = -M_PI_2;
    start[1] = 0;

    ompl::base::ScopedState<> goal(space);
    goal[0] = M_PI_2;
    goal[1] = 0;

    ss->setStartAndGoalStates(start, goal);


    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
