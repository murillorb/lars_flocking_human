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

#ifndef PFCONSTANTS_H_
#define PFCONSTANTS_H_

#include <WRCoach/utils/basics/competition.hh>

class PFConstants {
private:
    static float _constBallRepulsive;
    static float _constRepulsive;
	static float _constAreaRepulsive;
    static float _collisionCheckingRadius;
    static float _robotCollisionRadius;
    static float _ballCollisionRadius;
    static bool _collisionCheckingEnabled;
    static float _functionXshift;
    static float _functionYshift;
    static float _functionFactor;

    static bool _competitionSet;
public:
    static void setCompetition(Competitions::Competition competition);

    static float constBallRepulsive() { return PFConstants::_constBallRepulsive; }
    static float constRepulsive() { return PFConstants::_constRepulsive; }
	static float constAreaRepulsive() { return PFConstants::_constAreaRepulsive; }
    static float collisionCheckingRadius() { return PFConstants::_collisionCheckingRadius; }
    static float robotCollisionRadius() { return PFConstants::_robotCollisionRadius; }
    static float ballCollisionRadius() { return PFConstants::_ballCollisionRadius; }
    static bool collisionCheckingEnabled() { return PFConstants::_collisionCheckingEnabled; }
    static float functionXshift() { return PFConstants::_functionXshift; }
    static float functionYshift() { return PFConstants::_functionYshift; }
    static float functionFactor() { return PFConstants::_functionFactor; }
};

#endif // PFCONSTANTS_H_
