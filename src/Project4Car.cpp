///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <fstream>

#include <ompl/tools/benchmark/Benchmark.h>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 0;
    }

    void project(const ompl::base::State * /* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the car
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double velocity = u[0];
    const double steeringAngle = u[1];

    // Retrieve the current orientation of the car.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: x
    // 1: y
    // 2: theta
    const double theta = q[2];

    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);

    qdot[0] = velocity * cos(theta);            // x-dot
    qdot[1] = velocity * sin(theta);            // y-dot
    qdot[2] = velocity * tan(steeringAngle);    // theta-dot
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle rect;

    rect.x = 3.0;
    rect.y = 3.0;
    rect.width = 2.0;
    rect.height - 5.0;
    obstacles.push_back(rect);

    rect.x = 7.0;
    rect.y = 7.0;
    rect.width = 1.0;
    rect.height = 1.0;
    obstacles.push_back(rect);
}

bool isValidStateCar(const ompl::control::SpaceInformation *si, const ompl::base::State *state)
{
    const auto *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
    const auto *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const auto *rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1);
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}


ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    // state space
    auto se2 = std::make_shared<ompl::base::SE2StateSpace>();
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(10);
    se2->setBounds(bounds);

    // control space
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(se2, 2));
    ompl::base::RealVectorBounds cbounds(2);
    bounds.setLow(-0.3);
    bounds.setHigh(0.3);
    cspace->setBounds(cbounds);

    // Simple Setup
    ompl::control::SimpleSetupPtr ss(new ompl::control::SimpleSetup(cspace));

    // Propagation
//    ss->setStatePropagator(propagate);

    // Validity checker
//    ss->setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, sideLength, obstacles));

    // start state
    ompl::base::ScopedState<> start(se2);
    // goal state
    ompl::base::ScopedState<> goal(se2);
    ss->setStartAndGoalStates(start, goal);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr & ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    if (choice == 1)
    {
        //RRT
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    }
    else if (choice == 2)
    {
        //KPIECE1
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    }
    else if (choice == 3)
    {
        //RG-RRT
        //ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    }
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(30.0);

    if (solved) {
        /*Print Path as geometric*/
        // ss->simplifySolution();
        //ompl::control::PathControl &path = ss->getSolutionPath();
        ompl::geometric::PathGeometric path = ss->getSolutionPath().asGeometric();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "R2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr & ss)
{
    // TODO: Do some benchmarking for the car
    ompl::tools::Benchmark::Request request(100., 10000., 10);  // runtime (s), memory (MB), run count

//    ss->setup();
//
//    ompl::tools::Benchmark b(ss, ss.getName());
//    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
//    b.addPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
//    b.benchmark(request);
//    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
