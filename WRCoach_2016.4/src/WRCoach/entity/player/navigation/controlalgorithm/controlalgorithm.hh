/***
 * Warthog Robotics
 * University of Sao Paulo (USP) at Sao Carlos
 * http://www.warthog.sc.usp.br/
 *
 * This file is part of WRCoach project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***/

#ifndef CONTROLALGORITHM_HH
#define CONTROLALGORITHM_HH

#include <GEARSystem/gearsystem.hh>
#include <WRCoach/utils/wrtimer/wrtimer.hh>
#include <WRCoach/entity/player/navigation/controlalgorithm/controlalgorithmparameters.hh>

class ControlAlgorithm {
public:
    ControlAlgorithm();
    virtual ~ControlAlgorithm();
    virtual QString name() = 0;

    virtual float iterate(float distance, Velocity velocity);
    virtual void reset();

    virtual void accept(ControlAlgorithmParameters *setter) = 0;
private:
    virtual float _iterate(float error, Velocity velocity, float dt) = 0;

    // Timer to calc looptime
    WRTimer _dt;
};

#endif // CONTROLALGORITHM_HH
