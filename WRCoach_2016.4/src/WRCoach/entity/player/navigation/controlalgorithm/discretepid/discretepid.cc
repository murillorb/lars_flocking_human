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

#include "discretepid.hh"
#include <WRCoach/utils/utils.hh>
#include <WRCoach/const/globalconstants.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString DiscretePID::name() {
    return "DiscretePID";
}

DiscretePID::DiscretePID() {
    setParameters(0.0, 0.0, 0.0);
    reset();
}

DiscretePID::DiscretePID(float kp, float ki, float kd) {
    setParameters(kp, ki, kd);
    reset();
}

DiscretePID::~DiscretePID() {

}

void DiscretePID::reset() {
    _lastError = 0;
    _lastlastError = 0;
    _lastIteration = 0;
    ControlAlgorithm::reset();
}

float DiscretePID::_iterate(float error, Velocity velocity, float dt) {

    // Scaling
    float q0_term = (_q0 * error);
    float q1_term = (_q1 * _lastError);
    float q2_term = (_q2 * _lastlastError);

    // Save current errors for next iteration
    _lastlastError = _lastError;
    _lastError = error;

    // Save iteration
    _lastIteration = _lastIteration + q0_term + q1_term + q2_term;

    return _lastIteration;
}

void DiscretePID::accept(ControlAlgorithmParameters *params) {
    params->visit(this);
}

void DiscretePID::setParameters(double kp, double ki, double kd) {
    const float T0 = GlobalConstants::threadLoopFrequency();

    // Convert K to Q
    _q0 = kp + kd/T0 + ki*T0;
    _q1 = -kp -2.0*(kd/T0);
    _q2 = kd/T0;
}
