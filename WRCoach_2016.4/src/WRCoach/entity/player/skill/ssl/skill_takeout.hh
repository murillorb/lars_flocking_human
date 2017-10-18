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

#ifndef SKILL_TAKEOUT_HH
#define SKILL_TAKEOUT_HH

#include <WRCoach/entity/player/skill/skill.hh>
#include <WRCoach/const/globalconstants.hh>

class Skill_Takeout : public Skill {
private:
    // Parameters
    Position _posToAvoid;
	float _kickPower;

    // State machine
    enum {STATE_POS, STATE_KICK};
    int _state;

    void run();
public:
    Skill_Takeout();
    QString name();

    void setPositionToAvoid(const Position &posToAvoid) { _posToAvoid = posToAvoid; }
    void setKickPower(float kickPower) { _kickPower = kickPower; }
};

#endif // SKILL_TAKEOUT_HH
