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

#ifndef PID_HH
#define PID_HH

#include <WRCoach/entity/player/navigation/controlalgorithm/controlalgorithm.hh>

class PID :public ControlAlgorithm {
public:
    PID();
    PID(float kp, float ki, float kd, float iLimit);
    ~PID();

    QString name();

    void reset();

    void accept(ControlAlgorithmParameters *setter);
    void setParameters(double kp, double ki, double kd, double iLimit);
private:
    float _iterate(float error, Velocity velocity, float dt);

    // PID parameters
    double _kp;
    double _ki;
    double _kd;
    double _iLimit;

    // Internal variables
    bool _hasLastError;
    float _lastError;
    float _iError;
};

#endif // PID_HH
