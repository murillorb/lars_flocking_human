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

#ifndef BEHAVIOR_FOLLOWBALL_HH
#define BEHAVIOR_FOLLOWBALL_HH

#include <WRCoach/entity/player/behavior/behavior.hh>

class Behavior_FollowBall : public Behavior {
private:
    // Parameters
    float _distBall;

    // Skills
	Skill_GoTo *_sk_goTo;

    void configure();
    void run();
public:
    Behavior_FollowBall();
    QString name();

    void setBallDistance(float ballDistance) { _distBall = ballDistance; }
};

#endif // BEHAVIOR_FOLLOWBALL_HH
