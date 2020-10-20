#pragma once

#include "Stepper.h"
#include "Target.h"

namespace TeensyStep
{
    template<unsigned N = 100>
    class MotionPlanner{
        protected:
            static constexpr unsigned maxSize = N;
            unsigned numTargets = 0;
            float speed = 1.0;
            Target targetsX[maxSize];
            Target targetsY[maxSize];
            Target targetsZ[maxSize];

        public:
            // Actual speed will be in the interval [speed, sqrt(2)*speed] as the speed in
            // separated in x and y component.
            // lowest speed is reached when only one axis run.
            // max speed is reached when both axis run equal length.
            unsigned size() { return numTargets; }
            void clear() { numTargets = 0; }

            Target* getX(){ return targetsX; }
            Target* getY(){ return targetsY; }
            Target* getZ(){ return targetsZ; }

            /**
             * \brief Add point to motion.
             */
            bool addPoint(int x, int y, int z){
                if(numTargets >= maxSize) return false;
                targetsX[numTargets].target = x;
                targetsY[numTargets].target = y;
                targetsZ[numTargets].target = z;
                numTargets++;
                return true;
            }

            /**
             * \brief Calculate angle between two vectors.
             * \return Angle in degrees
             */
            float getAngle(int x0, int y0, int x1, int y1){
                float v0_d = sqrtf(x0*x0 + y0*y0);
                float v1_d = sqrtf(x1*x1 + y1*y1);
                float dot = x0*x1 + y0*y1;
                float angle = acosf(dot / (v0_d * v1_d))*180/PI;
                return angle;
            }

            /**
             * \brief Calculate the x' and y' axis speed that combined gives desired speed given in
             *        constructor.
             *        NOTE: Not supported as there is no way to set individual speeds when moving in sync.
             * \param[in] delta_x X axis delta of target
             * \param[in] delta_y Y axis delta of target
             * \param[out] speed_x Output x axis speed
             * \param[out] speed_y Output y axis speed
             */
            void getSpeed(int delta_x, int delta_y, int &speed_x, int &speed_y)
            {
                float len = sqrtf(delta_x*delta_x + delta_y*delta_y);
                speed_x = std::abs(static_cast<int>(speed*(delta_x/len)));
                speed_y = std::abs(static_cast<int>(speed*(delta_y/len)));
            }

            /**
             * \brief Internal function to set target for the steppers with start/end speed depending
             *        on angle change in motion. If the angle of the motion is changed with more than 45
             *         degrees a full stop/start is done between the targets. If the angle is less than 45
             *         degrees full speed is kept.
             * \param[in] t_x X target
             * \param[in] t_y Y target
             * \param[in] angle Angle between the current and the next target vector
             * \param[in] speed_x X axis max speed
             * \param[in] speed_y Y axis max speed
             * \param[in/out] pullin_x Pullin X speed to use for the current target. Is also updated to be used for the next target.
             * \param[in/out] pullin_y Pullin Y speed to use for the current target. Is also updated to be used for the next target.
             */
            void updateTarget(Target& x,  Target& y, float angle, float speed_x, float speed_y, float &pullin_x, float &pullin_y)
            {
                //Serial.printf("Target (x: %d, y: %d), speed_x: %d, speed_y: %d, angle: ", x.target, y.target, speed_x, speed_y);
                //Serial.println(angle);
                if(angle < 45)
                {
                    // the change in motion angle is low, we can keep full speed
                    x.setSpeedFactors(speed_x, pullin_x, speed_x);
                    y.setSpeedFactors(speed_y, pullin_y, speed_y);
                    pullin_x = 1.0; // ask stepper for real ve instead of assuming that everything is fine?
                    pullin_y = 1.0; // ask stepper for real ve instead of assuming that everything is fine?
                }
                else
                {
                    // the change in motion angle is too large, we need to slow down
                    x.setSpeedFactors(speed_x, pullin_x, 0);
                    y.setSpeedFactors(speed_y, pullin_y, 0);
                    pullin_x = pullin_y = 0;
                }
            }

            /**
             * \brief Calculate motion, must be done before call to move of steppers. The motion should be re-calculated on
             *        every new call to move, even if no points were removed/added.
             *        NOTE:
             *        * There is currently no way to find out if the steppers managed to reach their end speed(ve) or not
             *          so the assumption is that they always reach it. This can cause undesired high acc/dec if the
             *          end speed wasn't reached for one segment.
             *          Example:
             *          pullin speed = 100
             *          Desired end speed, Ve = 500
             *          start position: (0,0)
             *          targets: 1,1), (10,10)
             *          => Ve won't be reached when moving from (0,0) -> (1,1) but is set as start speed(vs) when starting with movement from (1,1) ->
             *          (10,10). This causes the steppers to accelerate from pullin speed to 500 in one step.
             */
            void calculate(int posX, int posY, int posZ)
            {
                float next_pullin_x = 0;
                float next_pullin_y = 0;
                if(numTargets == 0){
                    return;
                }

                // First point, we must take the current stepper position into consideration.
                int x0 = targetsX[0].target - posX;
                int y0 = targetsY[0].target - posY;
                float angle = getAngle(posX, posY, x0, y0);
                //getSpeed(x0, y0, speed_x, speed_y);
                updateTarget(targetsX[0], targetsY[0], angle, 1.0, 1.0, next_pullin_x, next_pullin_y);

                // quit if there's only one target
                if(numTargets < 2){
                    return;
                }

                x0 = targetsX[1].target - targetsX[0].target;
                y0 = targetsY[1].target - targetsY[0].target;
                unsigned i;
                for(i = 1; i < numTargets - 1; i++){
                    int x1 = targetsX[i+1].target - targetsX[i].target;
                    int y1 = targetsY[i+1].target - targetsY[i].target;

                    angle = getAngle(x0, y0, x1, y1);
                    //getSpeed(x1, y1, speed_x, speed_y);
                    updateTarget(targetsX[i], targetsY[i], angle, 1.0, 1.0, next_pullin_x, next_pullin_y);
                    x0 = x1;
                    y0 = y1;
                }
                // Add last point, always slow down
                //getSpeed(targets[i]->x - targets[i - 1]->x, targets[i]->y - targets[i - 1]->y, speed_x, speed_y);
                updateTarget(targetsX[i], targetsY[i], 90, 1.0, 1.0, next_pullin_x, next_pullin_y);
            }
    };
}