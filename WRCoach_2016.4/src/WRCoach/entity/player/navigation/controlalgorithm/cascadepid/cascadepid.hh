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

#ifndef CASCADEPID_HH
#define CASCADEPID_HH

#include <WRCoach/entity/player/navigation/controlalgorithm/controlalgorithm.hh>

class CascadePID : public ControlAlgorithm {
public:
    CascadePID();
    CascadePID(double dkp, double dkd, double vkp, double vki, double vkd, double viLimit);
    ~CascadePID();

    QString name();

    void reset();

    void accept(ControlAlgorithmParameters *setter);
    // dkp, dkd: parameters for distance PID
    // skp, ski, skd, siLimit: parameters for speed PID
    void setParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed);
private:
    float _iterate(float distance, Velocity velocity, float dt);

    PID *_distancePID;
    PID *_speedPID;

    float _maxSpeed;
};

#endif // CASCADEPID_HH
