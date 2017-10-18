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

#ifndef SENSOR_BALLMOVIMENTDETECT_HH_
#define SENSOR_BALLMOVIMENTDETECT_HH_

#include <GEARSystem/worldmap.hh>
#include <WRCoach/entity/referee/sslreferee/sslreferee.hh>
#include <WRCoach/entity/sensor/sensor.hh>

class Sensor_BallMovimentDetect : public WRSensor {
private:
    // External access
	SSLReferee *_ref;
    WorldMap *_wm;

    // Internal
    Position _lastBallPos;

    void initialization();
    void loop();
    void finalization();
public:
    Sensor_BallMovimentDetect(SSLReferee *ref, WorldMap *wm);
    QString name();
};

#endif // SENSOR_BALLMOVIMENTDETECT_HH_
