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

#include "pid.hh"
#include <WRCoach/utils/utils.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString PID::name() {
    return "PID";
}

PID::PID() {
    setParameters(0.0, 0.0, 0.0, 0.0);
    reset();
}

PID::PID(float kp, float ki, float kd, float iLimit) {
    setParameters(kp, ki, kd, iLimit);
    reset();
}

PID::~PID() {

}

void PID::reset() {
    _iError = 0;
    _lastError = 0;
    _hasLastError = false;
    ControlAlgorithm::reset();
}

float PID::_iterate(float error, Velocity velocity, float dt) {
    // dt check
    if(dt > 0.250)
        std::cout << "[WARNING] " << name().toStdString() << ": Very high dt! Check cause.\n";

    // Integration, with windup guarding
    _iError += error * dt;
    WR::Utils::limitValue(&_iError, -_iLimit, _iLimit);

    // Differentiation
    float diff = _hasLastError? (error-_lastError)/dt : 0.0;

    // Scaling
    float p_term = (_kp * error);
    float i_term = (_ki * _iError);
    float d_term = (_kd * diff);

    // Save current error for next iteration
    _lastError = error;
    _hasLastError = true;

    return p_term + i_term + d_term;
}

void PID::accept(ControlAlgorithmParameters *params) {
    params->visit(this);
}

void PID::setParameters(double kp, double ki, double kd, double iLimit) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _iLimit = iLimit;
}


