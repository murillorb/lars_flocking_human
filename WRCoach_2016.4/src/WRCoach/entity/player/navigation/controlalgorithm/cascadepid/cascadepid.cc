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

#include "cascadepid.hh"
#include <WRCoach/entity/player/navigation/wrnavigation.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString CascadePID::name() {
    return "Cascade PID";
}

CascadePID::CascadePID() {
    _distancePID = new PID();
    _speedPID = new PID();
    _maxSpeed = 0.0;
    reset();
}

CascadePID::CascadePID(double dkp, double dkd, double vkp, double vki, double vkd, double viLimit) {
    _distancePID = new PID(dkp, 0.0, dkd, 0.0);
    _speedPID = new PID(vkp, vki, vkd, viLimit);
    _maxSpeed = 0.0;
    reset();
}

CascadePID::~CascadePID() {

}

void CascadePID::reset() {
    _distancePID->reset();
    _speedPID->reset();
}

void CascadePID::accept(ControlAlgorithmParameters *params) {
    params->visit(this);
}

void CascadePID::setParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed) {
    _distancePID->setParameters(dkp, 0.0, dkd, 0.0);
    _speedPID->setParameters(skp, ski, skd, siLimit);
    _maxSpeed = maxSpeed;
}

float CascadePID::_iterate(float distance, Velocity velocity, float dt) {
    // 1st PID
    float speedSetPoint = _distancePID->iterate(distance, velocity);
    WR::Utils::limitValue(&speedSetPoint, -_maxSpeed, _maxSpeed);

    // 2nd PID
    float speedError = speedSetPoint - velocity.abs();
    float speedOffset = _speedPID->iterate(speedError, velocity);
    WR::Utils::approximateToZero(&speedOffset, 0.0001);

    // Return velocity
    return velocity.abs() + speedOffset;
}



