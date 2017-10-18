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

#ifndef CONTROLALGORITHMPARAMETERS_H
#define CONTROLALGORITHMPARAMETERS_H

#include <WRCoach/entity/player/navigation/controlalgorithm/wrcontrolalgorithmbase.hh>

class ControlAlgorithmParameters {
public:
    ControlAlgorithmParameters();

    void setTo(ControlAlgorithm *ctrAlg);

    void setPIDParameters(double kp, double ki, double kd, double iLimit);
    void setDiscretePIDParameters(double kp, double ki, double kd);
    void setCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed);

    void visit(PID *pid);
    void visit(DiscretePID *dpid);
    void visit(CascadePID *cpid);
private:
    // PID
    struct {
        double kp;      // Proportional
        double ki;      // Integrative
        double kd;      // Derivative
        double iLimit;  // Integral error limit
    } _pid;

    // DiscretePID
    struct {
        double kp;      // Proportional
        double ki;      // Integrative
        double kd;      // Derivative
    } _dpid;

    // Cascade PID
    struct {
        double dkp;     // Distance propotional
        double dkd;     // Distance derivative
        double skp;     // Speed proportional
        double ski;     // Speed integrative
        double skd;     // Speed derivative
        double siLimit; // Speed integral error limit
        float maxSpeed; // Maximum speed
    } _cpid;

    // Control type flag
    int _isPidSet;
    int _isDiscretePidSet;
    int _isCascadePidSet;
};

#endif // CONTROLALGORITHMPARAMETERS_H
