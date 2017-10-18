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

#ifndef SKILL_GOTO_HH
#define SKILL_GOTO_HH

#include <WRCoach/entity/player/skill/skill.hh>

class Skill_GoTo : public Skill {
private:
    // Parameters
    Position _destination;
    Position _positionToLook;
    bool _avoidBall;
    bool _avoidTeammates;
    bool _avoidOpponents;
    bool _avoidOurGoalArea;
    bool _avoidTheirGoalArea;
    bool _limitFieldDimensions;

    void run();
public:
    Skill_GoTo();
    QString name();

    void setDestination(const Position &destination) { _destination = destination; }
    void setPositionToLook(const Position &positionToLook) { _positionToLook = positionToLook; }
    void avoidBall(bool avoid) { _avoidBall = avoid; }
    void avoidTeammates(bool avoid) { _avoidTeammates = avoid; }
    void avoidOpponents(bool avoid) { _avoidOpponents = avoid; }
    void avoidRobots(bool avoid) { _avoidTeammates = _avoidOpponents = avoid; }
    void avoidGoalArea(bool avoid) { _avoidOurGoalArea = _avoidTheirGoalArea = avoid; }
    void avoidOurGoalArea(bool avoid) { _avoidOurGoalArea = avoid; }
    void avoidTheirGoalArea(bool avoid) { _avoidTheirGoalArea = avoid; }
    void avoidAll(bool avoid) { _avoidBall = _avoidTeammates = _avoidOpponents = avoid; }
    void limitFieldDimensions(bool limit) { _limitFieldDimensions = limit; }
};

#endif // SKILL_GOTO_HH
