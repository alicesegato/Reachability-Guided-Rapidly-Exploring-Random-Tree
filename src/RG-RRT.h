#ifndef RGRRT_H
#define RGRRT_H

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

namespace ompl
{
    namespace control
    {
        /**
           @anchor cRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           This implementation is intended for systems with differential constraints.
           @par External documentation
           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol.
           20, pp. 378–400, May 2001. DOI: [10.1177/02783640122067453](http://dx.doi.org/10.1177/02783640122067453)<br>
           [[PDF]](http://ijr.sagepub.com/content/20/5/378.full.pdf)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */

        /** \brief Rapidly-exploring Random Tree */
        class RGRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            RGRRT(const SpaceInformationPtr &si);

            ~RGRRT() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;


        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{1};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                std::vector<base::State *> reachableSet;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *qnear, const Motion *qrand) const
            {
                double d = si_->distance(qrand->state, qnear->state);
                for (base::State *s : qnear->reachableSet) {
                  if (si_->distance(s, qrand->state) < d) {
                    return d;
                  }
                }
                double inf = std::numeric_limits<double>::infinity();
                return inf;
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighborsLinear<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            std::vector<Control*> controlVector;

            std::vector<Control *> makeControlVector()
            {
              base::RealVectorBounds bounds = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds();
              std::vector<double> min = bounds.low;
              double diff = bounds.getDifference().at(0);
              double controlAsRealVect = min.at(0);
              double approxStep = diff/10;
              std::vector<Control *> controlVector;

                for (size_t i = 0; i < 10; i++) {
                  Control *c = siC_->allocControl();
                  c->as<RealVectorControlSpace::ControlType>()->values[0] = controlAsRealVect;
                  c->as<RealVectorControlSpace::ControlType>()->values[1] = 0;
                  controlVector.push_back(c);
                  controlAsRealVect = controlAsRealVect + approxStep;
                }
                return controlVector;
            }

            void addReachablitySet(Motion *motion, std::vector<Control *> controlVector){
              std::vector<base::State *> states;
              for (Control *c : controlVector)
              {
                ompl::base::State *result = siC_->allocState();
                siC_->propagate(motion->state, c, 1, result);
                if (si_->isValid(result)) {
                  states.push_back(result);
                }
              }
              motion->reachableSet = states;
            }

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
            double inf = std::numeric_limits<double>::infinity();
            unsigned int cd = 1;
        };
    }
}

#endif
