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

#ifndef BEHAVIOR_GK_HH
#define BEHAVIOR_GK_HH

#include <WRCoach/entity/player/behavior/behavior.hh>

class Behavior_GK : public Behavior {
private:
    // Parameters
    float _radius;
    bool _takeoutEnabled;
    float _takeoutFactor;
    bool _useAttackerOri;

    // Skills
    Skill_GoTo *_sk_goTo;
    Skill_InterceptBall *_sk_intBall;
    Skill_Takeout *_sk_takeout;

    // Skills transitions
    enum {SKT_TAKEOUT, SKT_POS, SKT_INTERCEPT};

    // Auxiliary
    Position getAttackerInterceptPosition();
    Position calcAttackerBallImpact();

    void configure();
    void run();
public:
    Behavior_GK();
    QString name();

    void setRadius(float radius) { _radius = radius; }
    void setTakeoutEnabled(bool enable) { _takeoutEnabled = enable; }
    void setTakeoutFactor(float takeoutFactor) { _takeoutFactor = takeoutFactor; }
    void useAttackerOri(bool useAttackerOri) { _useAttackerOri = useAttackerOri; }
};

#endif // BEHAVIOR_GK_HH
