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

#ifndef SKILL_PUSHBALL_HH
#define SKILL_PUSHBALL_HH

#include <WRCoach/entity/player/skill/skill.hh>

class Skill_PushBall : public Skill {
private:
    // Parameters
    Position _destination;
    float _maxPushDistance;
    bool _avoidOpponents;
    bool _avoidTeammates;

    // State machine
    enum {STATE_POS, STATE_PUSH};
    int _state;

    // Internal
    Position _currPos;
    Position _lastPos;
    float _distPushed;

    void run();
public:
	Skill_PushBall();
	QString name();

    void setDestination(const Position &destination) { _destination = destination; }
    void setMaxPushDistance(float maxPushDistance) { _maxPushDistance = maxPushDistance; }
    void avoidOpponents(bool avoid) { _avoidOpponents = avoid; }
    void avoidTeammates(bool avoid) { _avoidTeammates = avoid; }
    void avoidRobots(bool avoid) { _avoidOpponents = _avoidTeammates = avoid; }
};

#endif // SKILL_PUSHBALL_HH
