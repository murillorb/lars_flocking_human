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

#include "controlalgorithmparameters.hh"
#include <WRCoach/entity/player/navigation/wrnavigation.hh>

ControlAlgorithmParameters::ControlAlgorithmParameters() {
    _isPidSet = false;
    _isDiscretePidSet = false;
    _isCascadePidSet = false;
}

void ControlAlgorithmParameters::setPIDParameters(double kp, double ki, double kd, double iLimit) {
    _pid.kp = kp;
    _pid.ki = ki;
    _pid.kd = kd;
    _pid.iLimit = iLimit;

    _isPidSet = true;
}

void ControlAlgorithmParameters::setDiscretePIDParameters(double kp, double ki, double kd) {
    _dpid.kp = kp;
    _dpid.ki = ki;
    _dpid.kd = kd;

    _isDiscretePidSet = true;
}

void ControlAlgorithmParameters::setCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed) {
    _cpid.dkp = dkp;
    _cpid.dkd = dkd;
    _cpid.skp = skp;
    _cpid.ski = ski;
    _cpid.skd = skd;
    _cpid.siLimit = siLimit;
    _cpid.maxSpeed = maxSpeed;

    _isCascadePidSet = true;
}

void ControlAlgorithmParameters::setTo(ControlAlgorithm *ctrAlg) {
    ctrAlg->accept(this);
}

void ControlAlgorithmParameters::visit(PID *pid) {
    if(_isPidSet)
        pid->setParameters(_pid.kp, _pid.ki, _pid.kd, _pid.iLimit);
}

void ControlAlgorithmParameters::visit(DiscretePID *dpid) {
    if(_isDiscretePidSet)
        dpid->setParameters(_dpid.kp, _dpid.ki, _dpid.kd);
}

void ControlAlgorithmParameters::visit(CascadePID *cpid) {
    if(_isCascadePidSet)
        cpid->setParameters(_cpid.dkp, _cpid.dkd, _cpid.skp, _cpid.ski, _cpid.skd, _cpid.siLimit, _cpid.maxSpeed);
}
