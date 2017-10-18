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

#ifndef BEHAVIOR_ATTACK_HH
#define BEHAVIOR_ATTACK_HH

#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/utils/wrtimer/wrtimer.hh>

class Behavior_Attack : public Behavior {
    Q_OBJECT
private:
    // Parameters
    float _maxDistToKick;
    bool _clearKickEnabled;
    bool _passEnabled;
    bool _forcePass;
    QList<quint8> _recvs;

    // State machine
    enum {STATE_WAIT, STATE_PUSH, STATE_KICK};
    int _state;
    WRTimer _timer;

    // Internal
    QMutex _mutex;
    qint8 _recvId;
    Position _posToKick;
    float _kickPower;
    qint8 getBestRecv();

    // Skills
    Skill_GoTo *_sk_goTo;
    Skill_PushBall *_sk_pushBall;

    // Skills Transitions
    enum {SKT_ATTACK, SKT_WAIT};

    void configure();
    void run();
public:
    Behavior_Attack();
    QString name();

    void setMinimumKickDistance(float maxDistToKick) { _maxDistToKick = maxDistToKick; }
    void setEnableClearKick(bool enable) { _clearKickEnabled = enable; }
    void setEnablePass(bool enable) { _passEnabled = enable; }
    void setForcePass(bool forcePass) { _forcePass = forcePass; }
    void lockRecvs() { _mutex.lock(); }
    void addRecv(quint8 recvID) { _recvs.push_back(recvID); }
    void clearRecvList() { _recvs.clear(); }
    void unlockRecvs() { _mutex.unlock(); }
signals:
    void ballKicked(quint8 recvID);
    void aboutToKick(quint8 recvID);
};

#endif // BEHAVIOR_ATTACK_HH
