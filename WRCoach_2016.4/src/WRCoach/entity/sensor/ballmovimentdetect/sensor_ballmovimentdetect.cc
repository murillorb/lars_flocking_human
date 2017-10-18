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

#include "sensor_ballmovimentdetect.hh"
#include <WRCoach/utils/utils.hh>

Sensor_BallMovimentDetect::Sensor_BallMovimentDetect(SSLReferee *ref, WorldMap *wm) {
    _ref = ref;
    _wm = wm;

    _lastBallPos.setUnknown();
}

QString Sensor_BallMovimentDetect::name() {
    return "Sensor_BallMovimentDetect";
}

void Sensor_BallMovimentDetect::initialization() {
    std::cout << "[SENSOR_BallMovimentDetect] thread started.\n";
}

void Sensor_BallMovimentDetect::loop() {
    if(_lastBallPos.isUnknown())
        _lastBallPos = _wm->ballPosition(0);
    else {
        Position pos = _wm->ballPosition(0);

        float dist = WR::Utils::distance(pos, _lastBallPos);
        if(dist >= SensorConstants::distToConsiderBallMovement()) {
            _lastBallPos = pos;
            _ref->setBallKicked();
        }
    }
}

void Sensor_BallMovimentDetect::finalization() {
    std::cout << "[SENSOR_BallMovimentDetect] thread ended.\n";
}
