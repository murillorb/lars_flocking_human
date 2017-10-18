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

#ifndef BEHAVIOR_GOTO_HH
#define BEHAVIOR_GOTO_HH

#include <WRCoach/entity/player/behavior/behavior.hh>

class Behavior_GoTo : public Behavior {
private:
    // Parameters
    Position _destination;
    Position _posToLook;

    // Skills
	Skill_GoTo *_sk_goTo;

    void configure();
    void run();
public:
    Behavior_GoTo();
    QString name();

    void setDestination(const Position &destination) { _destination = destination; }
    void setPositionToLook(const Position &posToLook) { _posToLook = posToLook; }
};

#endif // BEHAVIOR_GOTO_HH
