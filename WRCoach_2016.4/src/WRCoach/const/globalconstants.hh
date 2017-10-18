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

#ifndef GLOBALCONSTANTS_HH_
#define GLOBALCONSTANTS_HH_

#include <WRCoach/entity/locations.hh>
#include <WRCoach/utils/basics/color.hh>
#include <WRCoach/utils/basics/competition.hh>

class GlobalConstants {
private:
    static Competitions::Competition _competition;
    static int _threadLoopFrequency;

    static bool _competitionSet;

    static float _maxKickPower;
    static float _maxPushDistance;
    static float _ballRadius;
    static float _minBallDistToKick;
    static float _gameOffMaxLSpeed;
    static float _highDistance;
public:
    static void setCompetition(Competitions::Competition competition);

    static Competitions::Competition competition() { return _competition; }
    static float threadLoopFrequency() { return _threadLoopFrequency; }

    static float maxKickPower() { return _maxKickPower; }
    static float maxPushDistance() { return _maxPushDistance; }
    static float ballRadius() { return _ballRadius; }
    static float minBallDistToKick() { return _minBallDistToKick; }
    static float gameOffMaxLSpeed() { return _gameOffMaxLSpeed; }
    static float highDistance() { return _highDistance; }
};

#endif /* GLOBALCONSTANTS_HH_ */
