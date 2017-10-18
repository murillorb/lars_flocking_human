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

#ifndef BEHAVIOR_RECV_HH
#define BEHAVIOR_RECV_HH

#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/utils/basics/quadrant.hh>

class Behavior_Recv : public Behavior {
    Q_OBJECT
private:
    // Parameters
    Quadrants::Quadrant _quadrant;
    bool _followAttacker;
    qint8 _attackerId;
    float _incDistAttacker;
    float _minRadius;
    float _maxRadius;

    // State machine
    enum {STATE_POS, STATE_WAIT, STATE_INTERCEPT};
    int _state;

    // Skills
    Skill_GoTo *_sk_goTo;

    void configure();
    void run();
public:
    Behavior_Recv();
    QString name();

    void setQuadrant(Quadrants::Quadrant quadrant) { _quadrant = quadrant; }
    void setFollowAttacker(bool follow) { _followAttacker = follow; }
    void setAttacker(qint8 attackerID, float incDistAttacker) { _attackerId = attackerID; _incDistAttacker = incDistAttacker; }
    void setRadius(float minRadius, float maxRadius) { _minRadius = minRadius; _maxRadius = maxRadius; }
signals:
    void ballReceived(quint8 recvID);
    void ballNotReceived(quint8 recvID);
public slots:
    void attackerAboutToKick(quint8 recvID);
    void attackerKicked(quint8 recvID);
};

#endif // BEHAVIOR_RECV_HH
