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

#include <iostream>
#include "controlalgorithm.hh"

ControlAlgorithm::ControlAlgorithm() {
    reset();
}

ControlAlgorithm::~ControlAlgorithm() {

}

void ControlAlgorithm::reset() {
    _dt.start();
}

float ControlAlgorithm::iterate(float distance, Velocity velocity) {
    _dt.stop();
    float dt = _dt.timesec();
    _dt.start();

    return _iterate(distance, velocity, dt);
}
