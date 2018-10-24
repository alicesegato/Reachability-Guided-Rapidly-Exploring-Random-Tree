///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <cmath>

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
    CarProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State *  state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        auto compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
        auto se2 = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
        auto r2 = se2->as<ompl::base::RealVectorStateSpace::StateType>(0);
        projection[0] = r2->values[0];
        projection[1] = r2->values[1];

    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double angularVelocity = u[0];
    const double forwardAcceleration = u[1];

    // Retrieve the current orientation of the car.
    // 0: x
    // 1: y
    // 2: heading
    // 3: velocity
    const double heading = q[2];
    const double velocity = q[3];

    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);

    qdot[0] = velocity * cos(heading);            // x-dot
    qdot[1] = velocity * sin(heading);            // y-dot
    qdot[2] = angularVelocity;
    qdot[3] = forwardAcceleration;
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle rect;

    rect.x = 0.0;
    rect.y = 8.0;
    rect.width = 8.0;
    rect.height = 2.0;
    obstacles.push_back(rect);

    rect.x = 2.0;
    rect.y = 4.0;
    rect.width = 8.0;
    rect.height = 2.0;
    obstacles.push_back(rect);

    rect.x = 3.0;
    rect.y = 2.0;
    rect.width = 3.0;
    rect.height = 2.0;
    obstacles.push_back(rect);
}

bool isValidStateCar(ompl::control::SpaceInformation* si, const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    // check for collisions in the state space
    auto compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
    auto se2 = compoundState->as<ompl::base::SE2StateSpace::StateType>(0);
    auto r2 = se2->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2->values[0];
    double y = r2->values[1];
    double angle = r2->values[3];
    
//    std::cout << "(x, y): (" << x << " ," << y << ")" << std::endl;
//    std::cout << "1: (" << obstacles.at(0).x << ", " << obstacles.at(0).y << ")" << std::endl;
//    std::cout << "2: (" << obstacles.at(1).x << ", " << obstacles.at(1).y << ")" << std::endl;

    return isValidSquare(x, y, angle, 0.3, obstacles) && si->satisfiesBounds(state);
}

void carPostIntegration(const ompl::base::State* /*state*/, const ompl::control::Control * /*control*/, const double /*duration*/, ompl::base::State *result)
{
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0));
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    // state space (x, y, heading)
    auto se2 = std::make_shared<ompl::base::SE2StateSpace>();
    // bounds for heading
    ompl::base::RealVectorBounds se2bounds(2);
    se2bounds.setLow(0);
    se2bounds.setHigh(10);
    se2->setBounds(se2bounds);

    // (forward velocity)
    auto r1 = std::make_shared<ompl::base::RealVectorStateSpace>(1);
    ompl::base::RealVectorBounds r1bounds(1);
    r1bounds.setLow(-5);
    r1bounds.setHigh(5);
    r1->setBounds(r1bounds);

    // (x, y, heading, forward velocity)
    ompl::base::StateSpacePtr space;
    space = se2 + r1;

    space->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new CarProjection(space)));

    // make obstacles
    makeStreet(obstacles);

    // control space (angular velocity, forward acceleration)
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 2));
    // bounds for forward acceleration
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(-5);
    cbounds.setHigh(5);
    cspace->setBounds(cbounds);

    // Simple Setup
    ompl::control::SimpleSetupPtr ss(new ompl::control::SimpleSetup(cspace));

    // Propagation
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));
    //ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // Validity checker
    auto si = ss->getSpaceInformation().get();
    //auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    ss->setStateValidityChecker([obstacles, si](const ompl::base::State *state) {
        return isValidStateCar(si, state, obstacles);
    });

    //ss->setStateValidityChecker(std::bind(isValidStateCar, ss->getSpaceInformation(), std::placeholders::_1, obstacles));

    // start state
    ompl::base::ScopedState<> start(space);
    start[0] = 9;
    start[1] = 3;
    start[2] = M_PI_4;
    start[3] = 3;


    // goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = 9;
    goal[1] = 9;
    goal[2] = M_PI_4;
    goal[3] = 6;

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
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    }
    else if (choice == 2)
    {
        //KPIECE1
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    }
    else if (choice == 3)
    {
        //RG-RRT
        ss->getSpaceInformation()->setPropagationStepSize(0.05);
        ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    }
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(10.0);

    if (solved) {
        /*Print Path as geometric*/
        // ss->simplifySolution();
        //ompl::control::PathControl &path = ss->getSolutionPath();

        ompl::control::PathControl &controlPath = ss->getSolutionPath();
        controlPath.interpolate();
        ompl::geometric::PathGeometric geometricPath = controlPath.asGeometric();
        geometricPath.interpolate();
        geometricPath.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        geometricPath.printAsMatrix(fout);
        fout.close();
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr & ss)
{
    // TODO: Do some benchmarking for the car
    ompl::tools::Benchmark::Request request(100., 10000., 10);  // runtime (s), memory (MB), run count

    ss->setup();
    ss->print();
    ompl::tools::Benchmark b(*ss, "Car Benchmarking");
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss->getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::KPIECE1(ss->getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RGRRT(ss->getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 60;
    req.maxMem = 1000;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile();
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

/*
 * 1. Is the state space and control space good?
 * 2. What are reasonable bounds to set? Why does setting bounds give me an error of dimensionality? Where to enforce bounds?
 * 3. Do I need to check control bounds or just state space bounds in the isValidState? Or just collision?
 * 4. Post integration procedures?
 * 5. ODEs correct?
 * 6. Projections?
 */