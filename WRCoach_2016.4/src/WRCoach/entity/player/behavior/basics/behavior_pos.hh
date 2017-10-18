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

#ifndef BEHAVIOR_POS_HH
#define BEHAVIOR_POS_HH

#include <WRCoach/entity/player/behavior/behavior.hh>

class Behavior_Pos : public Behavior {
private:
    // Parameters
    Position _destination;
    Position _posToLook;
    bool _avoidBall;
    bool _avoidTeammates;
    bool _avoidOpponents;
	bool _avoidGoalArea;
    bool _limitFieldDimensions;

    // Internal
    bool _isPositioned;

    // Skills
    Skill_GoTo *_sk_goTo;

    void configure();
    void run();
public:
    Behavior_Pos();
    QString name();

    void setPosition(const Position &pos) { _destination = pos; }
    void setPositionToLook(const Position &pos) { _posToLook = pos; }

    void avoidBall(bool avoid) { _avoidBall = avoid; }
    void avoidTeammates(bool avoid) { _avoidTeammates = avoid; }
    void avoidOpponents(bool avoid) { _avoidOpponents = avoid; }
    void avoidAll(bool avoid) { _avoidBall = _avoidTeammates = _avoidOpponents = avoid; }
    void avoidRobots(bool avoid) { _avoidTeammates = _avoidOpponents = avoid; }
    void avoidGoalArea(bool avoid) { _avoidGoalArea = avoid; }
    void limitFieldDimensions(bool limit) { _limitFieldDimensions = limit; }

    bool isPositioned() const;
};

#endif // BEHAVIOR_POS_HH
