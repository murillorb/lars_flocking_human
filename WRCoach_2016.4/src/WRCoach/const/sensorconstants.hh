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

#ifndef SENSORCONSTANTS_HH
#define SENSORCONSTANTS_HH

#include <WRCoach/utils/basics/competition.hh>

class SensorConstants {
private:
    static float _distToConsiderBallMovement;
    static float _maxDistPossession;

    static bool _competitionSet;
public:
    static void setCompetition(Competitions::Competition competition);

    static float distToConsiderBallMovement() { return _distToConsiderBallMovement; }
    static float maxDistPossession() { return _maxDistPossession; }
};

#endif // SENSORCONSTANTS_HH
