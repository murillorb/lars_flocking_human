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

#ifndef PLAYBOOK_ATTACK_HH
#define PLAYBOOK_ATTACK_HH

#include <WRCoach/entity/controlmodule/coach/playbook/playbook.hh>
#include <WRCoach/utils/wrtimer/wrtimer.hh>

class Playbook_Attack : public Playbook {
    Q_OBJECT
private:
    // Parameters
    qint8 _kicker;
    bool _forcePass;

    // State machine
    enum {STATE_POS, STATE_GOINGTOPASS, STATE_PASS};
    int _state;

    // Internal
    quint8 _recvKicked;

    // Players assigned to behaviors
    quint8 _attacker;
    quint8 _recvM, _recvMR, _recvFW;

    // Behaviors
    Behavior_Attack *_bh_atk;
    Behavior_Recv *_bh_atkM;

    Behavior_Pos *_bh_recvPos;
    Behavior_Recv *_bh_recvM;
    Behavior_Recv *_bh_recvMR;
    Behavior_Recv *_bh_recvFW;

    // Internal methods
    void checkAssignedPlayers(int numPlayers);

    void configure(int numPlayers);
    void run(int numPlayers);
public:
    Playbook_Attack();
    QString name();
    int maxNumPlayer();

    void setKicker(quint8 kicker) { _kicker = kicker; }
    void setForcePass(bool forcePass) { _forcePass = forcePass; }

public slots:
    void ballKicked(quint8 recvID);
    void ballReceived();
    void ballNotReceived();
    void attackerAboutToKick(quint8 recvID);
};

#endif // PLAYBOOK_ATTACK_HH
