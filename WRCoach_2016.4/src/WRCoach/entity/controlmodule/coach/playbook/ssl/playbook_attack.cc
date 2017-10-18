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

#include "playbook_attack.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#define DIST_RECVFW_FALLBACK 1.5f

QString Playbook_Attack::name() {
    return "Playbook_Attack";
}

Playbook_Attack::Playbook_Attack() {
    _bh_atk = NULL;
    _bh_atkM = NULL;
    _bh_recvM = NULL;
    _bh_recvMR = NULL;
    _bh_recvFW = NULL;
    _bh_recvPos = NULL;

    _kicker = -1;
    setForcePass(false);

    _recvKicked = 0;
    _attacker = _recvM = _recvMR = _recvFW = 0;

    _state = STATE_POS;
}

int Playbook_Attack::maxNumPlayer() {
   return 4;
}

void Playbook_Attack::configure(int numPlayers) {

    // Create behaviors
    switch(numPlayers) {
        default:
        case 4:
            usesBehavior(_bh_recvFW = new Behavior_Recv());
        case 3:
            usesBehavior(_bh_recvMR = new Behavior_Recv());
        case 2:
            usesBehavior(_bh_recvM = new Behavior_Recv());
        case 1:
            usesBehavior(_bh_atk = new Behavior_Attack());
            usesBehavior(_bh_atkM = new Behavior_Recv());
    }

    // Connect signals
    switch(numPlayers) {
        default:
        case 4:
            connect(_bh_atk, SIGNAL(ballKicked(quint8)), _bh_recvFW, SLOT(attackerKicked(quint8)), Qt::DirectConnection);
            connect(_bh_atk, SIGNAL(aboutToKick(quint8)), _bh_recvFW, SLOT(attackerAboutToKick(quint8)), Qt::DirectConnection);
            connect(_bh_recvFW, SIGNAL(ballReceived(quint8)), this, SLOT(ballReceived()), Qt::DirectConnection);
            connect(_bh_recvFW, SIGNAL(ballNotReceived(quint8)), this, SLOT(ballNotReceived()), Qt::DirectConnection);
        case 3:
            connect(_bh_atk, SIGNAL(ballKicked(quint8)), _bh_recvMR, SLOT(attackerKicked(quint8)), Qt::DirectConnection);
            connect(_bh_atk, SIGNAL(aboutToKick(quint8)), _bh_recvMR, SLOT(attackerAboutToKick(quint8)), Qt::DirectConnection);
            connect(_bh_recvMR, SIGNAL(ballReceived(quint8)), this, SLOT(ballReceived()), Qt::DirectConnection);
            connect(_bh_recvMR, SIGNAL(ballNotReceived(quint8)), this, SLOT(ballNotReceived()), Qt::DirectConnection);
        case 2:
            connect(_bh_atk, SIGNAL(ballKicked(quint8)), _bh_recvM, SLOT(attackerKicked(quint8)), Qt::DirectConnection);
            connect(_bh_atk, SIGNAL(aboutToKick(quint8)), _bh_recvM, SLOT(attackerAboutToKick(quint8)), Qt::DirectConnection);
            connect(_bh_recvM, SIGNAL(ballReceived(quint8)), this, SLOT(ballReceived()), Qt::DirectConnection);
            connect(_bh_recvM, SIGNAL(ballNotReceived(quint8)), this, SLOT(ballNotReceived()), Qt::DirectConnection);
        case 1:
            connect(_bh_atk, SIGNAL(ballKicked(quint8)), this, SLOT(ballKicked(quint8)), Qt::DirectConnection);

            connect(_bh_atk, SIGNAL(aboutToKick(quint8)), this, SLOT(attackerAboutToKick(quint8)), Qt::DirectConnection);
        break;
    }
}

void Playbook_Attack::run(int numPlayers) {
    if(_kicker==-1)
        std::cout << "[WARNING] " << name().toStdString() << ", kicker not set!\n";

    const int numRecvs = numPlayers-1;
    Behavior *bh_atk=NULL;
    Behavior *bh_recvM=NULL;

    switch(_state) {
        default:
        case STATE_POS: {
            // Attacker
            _attacker = _kicker;
            dist()->removePlayer(_kicker);
            bh_atk = _bh_atk;

            // RecvM
            if(numRecvs==1) {
                _recvM = dist()->getPlayer();
                bh_recvM = _bh_recvM;
            }

            // Recvs M + MR
            if(numRecvs==2) {
                QList<quint8> tmp = dist()->getKNN(2, loc()->theirGoal());
                _recvM = tmp.takeFirst();
                _recvMR = tmp.takeFirst();
                bh_recvM = _bh_recvM;
            }

            // Recvs M + MR + FW
            if(numRecvs>=3) {
                QList<quint8> tmp = dist()->getKNN(3, loc()->theirGoal());
                _recvFW = tmp.takeFirst();
                _recvM = tmp.takeFirst();
                _recvMR = tmp.takeFirst();
                bh_recvM = _bh_recvM;
            }

        } break;

        case STATE_GOINGTOPASS: {
//            checkAssignedPlayers(numPlayers); PARECE ser inútil, causa problema com passe

            // Attacker
            bh_atk = _bh_atk;

            // RecvM
            if(numRecvs>=1)
                bh_recvM = _bh_recvM;
        } break;

        case STATE_PASS: {
//            checkAssignedPlayers(numPlayers); PARECE ser inútil, causa problema com passe

            // Attacker always change to RecvM after kicking
            if(_bh_atkM != NULL)
                bh_atk = _bh_atkM;

            // RecvM
            if(numRecvs>=1)
                bh_recvM = _bh_recvM;

            // Configure
            quint8 newAttacker = _recvKicked; // recv that gets the ball becomes the next attacker
            Position posNewAttackerNext = PlayerBus::ourPlayer(newAttacker)->nextPosition();
            Quadrants::Quadrant quadrant = (posNewAttackerNext.y()>=0)? Quadrants::BOTMID : Quadrants::UPMID;
            _bh_atkM->setAttacker(newAttacker, 0.5f);
            _bh_atkM->setQuadrant(quadrant);
        } break;
    }

    quint8 currAttacker = (_state==STATE_PASS? _recvKicked : _attacker);
    const Position posAttackerNext = PlayerBus::ourPlayer(currAttacker)->nextPosition();

    /// SET BEHAVIORS
	// Attacker
	setPlayerBehavior(_attacker, bh_atk);

    // RecvM
    if(numRecvs>=1)
        setPlayerBehavior(_recvM, bh_recvM);
    // RecvMR
    if(numRecvs>=2)
        setPlayerBehavior(_recvMR, _bh_recvMR);
    // RecvFW
    if(numRecvs>=3)
        setPlayerBehavior(_recvFW, _bh_recvFW);

    /// CONFIGURE BEHAVIORS
    // Attacker
    _bh_atk->setEnableClearKick(true);
    _bh_atk->setEnablePass(numPlayers>1);
    _bh_atk->setForcePass(_forcePass);

    // Add receveirs to attacker
    _bh_atk->lockRecvs();
    _bh_atk->clearRecvList();
    switch(numRecvs) {
        case 3:
            _bh_atk->addRecv(_recvFW);
        case 2:
            _bh_atk->addRecv(_recvMR);
        case 1:
            _bh_atk->addRecv(_recvM);
        default:
        break;
    }
    _bh_atk->unlockRecvs();

    // RecvM
    if(numRecvs>=1) {
        Quadrants::Quadrant quadrant = (posAttackerNext.y()>=0)? Quadrants::BOTMID : Quadrants::UPMID;
        _bh_recvM->setAttacker(currAttacker, 0.5f);
        _bh_recvM->setFollowAttacker(true);
        _bh_recvM->setQuadrant(quadrant);
        _bh_recvM->setRadius(0.6f*loc()->fieldMaxX(), loc()->fieldMaxX());
    }

    // RecvMR
    if(numRecvs>=2) {
        Quadrants::Quadrant quadrant = Quadrants::UPMID+Quadrants::BOTMID;
        _bh_recvMR->setAttacker(currAttacker, 1.0);
        _bh_recvMR->setFollowAttacker(true);
        _bh_recvMR->setQuadrant(quadrant);
        _bh_recvMR->setRadius(1.2f*loc()->fieldMaxX(), 1.5f*loc()->fieldMaxX());
    }

    // RecvFW
    if(numRecvs>=3) {
        Quadrants::Quadrant quadrant = (PlayerBus::ourPlayer(_recvFW)->position().y()>=0)? Quadrants::UP : Quadrants::BOT;
        _bh_recvFW->setAttacker(_attacker, 0.5f);
        _bh_recvFW->setFollowAttacker(false);
        _bh_recvFW->setQuadrant(quadrant);
        _bh_recvFW->setRadius(loc()->fieldMaxX()/2.0f, loc()->fieldMaxX()/2.0f);
    }

}

void Playbook_Attack::ballKicked(quint8 recvID) {
    _recvKicked = recvID;
    _state = STATE_PASS;
}

void Playbook_Attack::ballReceived() {
    _state = STATE_POS;
}

void Playbook_Attack::ballNotReceived() {
    _state = STATE_POS;
}

void Playbook_Attack::attackerAboutToKick(quint8 recvID) {
    _recvKicked = recvID;
    _state = STATE_GOINGTOPASS;
}

void Playbook_Attack::checkAssignedPlayers(int numPlayers) {

    // Check if players still on playbook
    bool hasPlayers = true;
    switch(numPlayers) {
        default:
        case 4:
            hasPlayers &= this->hasPlayer(_recvFW);
        case 3:
            hasPlayers &= this->hasPlayer(_recvMR);
        case 2:
            hasPlayers &= this->hasPlayer(_recvM);
        case 1:
            hasPlayers &= this->hasPlayer(_attacker);
    }

    // Change state if doesn't have all players
    if(hasPlayers==false)
        _state = STATE_POS;

}
