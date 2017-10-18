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

#ifndef BEHAVIOR_GKA_HH
#define BEHAVIOR_GKA_HH

#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/utils/fieldside/fieldside.hh>

class Behavior_GKA : public Behavior {
private:
    // Parameters
    float _radius;
    float _d;
    bool _takeoutEnabled;
    float _takeoutFactor;
    FieldSide _side;

    // Skills
    Skill_GoTo *_sk_goTo;
    Skill_InterceptBall *_sk_intBall;
    Skill_Kick *_sk_kickout;

    // Skills Transitions
    enum {SKT_KICKOUT, SKT_POS, SKT_INTERCEPT};

    void configure();
    void run();
public:
    Behavior_GKA();
    QString name();

    void setRadius(float radius) { _radius = radius; }
    void setD(float d) { _d = d; }
    void setTakeoutEnabled(bool enable) { _takeoutEnabled = enable; }
    void setTakeoutFactor(float takeoutFactor) { _takeoutFactor = takeoutFactor; }
    void setSide(FieldSide side) { _side = side; }
};

#endif // BEHAVIOR_GKA_HH
