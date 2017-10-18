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

#ifndef SKILL_INTERCEPTBALL_HH
#define SKILL_INTERCEPTBALL_HH

#include <WRCoach/entity/player/skill/skill.hh>

class Skill_InterceptBall : public Skill {
private:
    // Parameters
    bool _advance;
    Position _positionToLook;

    void run();
public:
    Skill_InterceptBall();
    QString name();

    void setAdvance(bool advance) { _advance = advance; }
    void setPositionToLook(const Position &positionToLook) { _positionToLook = positionToLook; }
};

#endif // SKILL_INTERCEPTBALL_HH
