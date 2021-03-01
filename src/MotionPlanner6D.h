#pragma once

#include "Stepper.h"
#include "accelerators/LinStepAccelerator.h"
#include "Target.h"

namespace TeensyStep
{
    /**
     * \brief Implementation of a motion planner with N points buffer
     */
    template<unsigned N = 100>
    class MotionPlanner6D
    {
        protected:
            static constexpr unsigned maxSize = N;
            unsigned numTargets = 1;
            Target targetsX[maxSize];
            Target targetsY[maxSize];
            Target targetsZ[maxSize];
            Target targetsA[maxSize];
            Target targetsB[maxSize];
            Target targetsC[maxSize];
            LinStepAccelerator accelerator;

        public:
            MotionPlanner6D() {}
            unsigned getMaxSize() {return maxSize; }

            unsigned size() { return numTargets - 1; }
            void clear() { numTargets = 1; }
            Target* getX() { return &targetsX[1]; }
            Target* getY() { return &targetsY[1]; }
            Target* getZ() { return &targetsZ[1]; }
            Target* getA() { return &targetsA[1]; }
            Target* getB() { return &targetsB[1]; }
            Target* getC() { return &targetsC[1]; }

            /**
             * \brief Add point to motion.
             */
            bool addPoint(int x, int y=0, int z = 0, int a=0, int b = 0, int c=0)
            {
                // reserve one place for the end point that is
                // assumed to exist for the algorithm to work.
                if(numTargets >= maxSize -1) return false;

                targetsX[numTargets].target = x;
                targetsY[numTargets].target = y;
                targetsZ[numTargets].target = z;
                targetsA[numTargets].target = a;
                targetsB[numTargets].target = b;
                targetsC[numTargets].target = c;
                numTargets++;
                return true;
            }

            /**
             * \brief Update segments between start(included) and end(excluded) in targets with pullin and pullout speed for each segment.
             */
            void updateSubSegments(Target* targets, unsigned start, unsigned end, int vPullOut, LinStepAccelerator& accelerator)
            {
                for(unsigned i = start; i < end; i++)
                {
                    targets[i].vPullIn  = accelerator.updateSpeed(targets[i - 1].target);
                    targets[i].vPullOut = i == end -1 ? vPullOut : accelerator.updateSpeed(targets[i].target); // updateSpeed returns zero to indicate end of movement, must
                                                                                                               // adjust it to the steppers pullout speed
                }
            }


            void updateSegments(Target* targets, unsigned len, int vMax, int pullIn, int pullOut, int acc)
            {
                if(len <= 1)
                {
                    return;
                }

                for(unsigned i = 0; i <= len; i++)
                {
                    targets[i].dir = (i == 0 || i == len) ? Target::Direction::NONE : Target::getDirection(targets[i], targets[i - 1]);
                }

                Target::Direction segDirection = Target::Direction::NONE;
                Target::Direction curDirection = Target::Direction::NONE;
                int segStart = -1, segEnd = 0;
                for(unsigned i = 0; i < len; i++)
                {
                    curDirection = targets[i + 1].dir;
                    if(segStart == -1 && curDirection == Target::Direction::NONE)
                    {
                        continue;
                    }

                    if(segStart == -1)
                    {
                        segStart = i;
                        segDirection = curDirection;
                        continue;
                    }

                    if(segDirection == curDirection)
                    {
                        continue;
                    }
                    segEnd = i;
                     // calculate the motion for the segment
                    accelerator.prepareMovement(targets[segStart].target, targets[segEnd].target, vMax, pullIn, pullOut, acc);
                    // split the motion over all points in the segment
                    updateSubSegments(targets, segStart + 1, segEnd + 1, pullOut, accelerator);
                    segStart = -1; // start scan of next segment start
                    i--; //current segment's end will be next segments start
                }
            }

            /**
             * \brief Calculate motion, must be done before call to move of steppers.
             */
            void calculate(Stepper &x, Stepper &y, Stepper &z, Stepper &a, Stepper &b, Stepper &c)
            {
                // Add start point
                targetsX[0].target = x.getPosition();
                targetsY[0].target = y.getPosition();
                targetsZ[0].target = z.getPosition();
                targetsA[0].target = a.getPosition();
                targetsB[0].target = b.getPosition();
                targetsC[0].target = c.getPosition();

                updateSegments(targetsX, numTargets, x.getMaxSpeed(), x.getMaxPullInSpeed(), x.getMaxPullOutSpeed(), x.getAcceleration());
                updateSegments(targetsY, numTargets, y.getMaxSpeed(), y.getMaxPullInSpeed(), y.getMaxPullOutSpeed(), y.getAcceleration());
                updateSegments(targetsZ, numTargets, z.getMaxSpeed(), z.getMaxPullInSpeed(), z.getMaxPullOutSpeed(), z.getAcceleration());
                updateSegments(targetsA, numTargets, a.getMaxSpeed(), a.getMaxPullInSpeed(), a.getMaxPullOutSpeed(), a.getAcceleration());
                updateSegments(targetsB, numTargets, b.getMaxSpeed(), b.getMaxPullInSpeed(), b.getMaxPullOutSpeed(), b.getAcceleration());
                updateSegments(targetsC, numTargets, c.getMaxSpeed(), c.getMaxPullInSpeed(), c.getMaxPullOutSpeed(), c.getAcceleration());

                x.setTargets(getX(), size());
                y.setTargets(getY(), size());
                z.setTargets(getZ(), size());
                a.setTargets(getA(), size());
                b.setTargets(getB(), size());
                c.setTargets(getC(), size());

            }
    };
}
