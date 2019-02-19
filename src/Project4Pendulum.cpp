#include <iostream>
#include <cmath>
#include <functional>
#include <valarray>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include "ompl/tools/benchmark/Benchmark.h"


#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"


// Your projection for the pendulum

class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        unsigned int dim = 1;
        return dim;
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
         auto compoundState = state->as<ompl::base::CompoundState>();
         auto omega = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(1);

         projection[0] = 0.5*pow(omega->values[0],2);
    }
};

bool isValidStatePendulum(const ompl::base::State *state)
{
      const ompl::base::CompoundState *space = state->as<ompl::base::CompoundState>();

      //const ompl::base::SO2StateSpace::StateType *theta = space->as<ompl::base::SO2StateSpace::StateType>(0);
      const ompl::base::RealVectorStateSpace::StateType *omega = space->as<ompl::base::RealVectorStateSpace::StateType>(1);

      if (omega->values[0]<-10 or omega->values[0]>10) {
        return false;
      }
      return true;
}

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control * control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[0];
    const double omega = q[1];

    qdot.resize (q.size (),0);
    qdot[0] = omega;
    qdot[1] = -9.81 * cos(theta) + u[0];
}

void PendulumPostIntegration (const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result)
{
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(0));

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

    space->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space)));

    // Control Space
    ompl::control::ControlSpacePtr cSpace(new ompl::control::RealVectorControlSpace(space, 1));

    // Set the bounds for Control SpaceInformation
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);


    ompl::control::SimpleSetupPtr ss(new ompl::control::SimpleSetup(cSpace));

    ss->setStateValidityChecker(&isValidStatePendulum);

    // Propagation Routine
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &PendulumPostIntegration));

    //Start and Goal States
    ompl::base::ScopedState<> start(space);
    start[0] = -M_PI_2;
    start[1] = 0;

    ompl::base::ScopedState<> goal(space);
    goal[0] = M_PI_2;
    goal[1] = 0;

    ss->setStartAndGoalStates(start, goal, 0.05);


    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    if (choice == 1) {
      /* RRT */
      ss->getSpaceInformation()->setPropagationStepSize(0.05);
      auto planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
      ss->setPlanner(planner);

    } else if (choice == 2) {
      /* KPIECE1 */
      ss->getSpaceInformation()->setPropagationStepSize(0.05);
      auto planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
      // space->registerDefaultProjection(ompl::base::ProjectionEvaluatorPtr(new PendulumProjection(space)));
      ss->setPlanner(planner);


    } else if (choice == 3) {
      /* RG_RRT */
      ss->getSpaceInformation()->setPropagationStepSize(0.05);
      auto planner = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
      ss->setPlanner(planner);

    }
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(40.0);

    if (solved) {
      /*Print Path as geometric*/
      // ss->simplifySolution();

      ompl::control::PathControl &controlPath = ss->getSolutionPath();
      controlPath.interpolate();
      ompl::geometric::PathGeometric geometricPath = controlPath.asGeometric();
      geometricPath.interpolate();
      geometricPath.printAsMatrix(std::cout);

      // print path to file
      std::ofstream fout("path.txt");
      fout << "R2" << std::endl;
      geometricPath.printAsMatrix(fout);
      fout.close();
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the pendulum
    ss->setup();
    ss->print();
    ompl::tools::Benchmark b(*ss, "Pendulum Benchmarking");
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss->getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::KPIECE1(ss->getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RGRRT(ss->getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 30;
    req.maxMem = 1000;
    req.runCount = 30;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile();
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
