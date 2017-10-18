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

#ifndef SKILL_KICK_HH
#define SKILL_KICK_HH

#include <WRCoach/entity/player/skill/skill.hh>

class Skill_Kick : public Skill {
private:
    // Parameters
    Position _posToKick;
    float _kickPower;
    bool _avoidTeammates;
    bool _avoidOpponents;

    // State machine
    enum {STATE_POS, STATE_KICK};
    int _state;

    void run();
public:
    Skill_Kick();
	QString name();

    void setPosToKick(const Position &posToKick) { _posToKick = posToKick; }
    void setKickPower(float kickPower) { _kickPower = kickPower; }
    void avoidTeammates(bool avoid) { _avoidTeammates = avoid; }
    void avoidOpponents(bool avoid) { _avoidOpponents = avoid; }
    void avoidRobots(bool avoid) { _avoidTeammates = _avoidOpponents = avoid; }
};

#endif // SKILL_KICK_HH_
