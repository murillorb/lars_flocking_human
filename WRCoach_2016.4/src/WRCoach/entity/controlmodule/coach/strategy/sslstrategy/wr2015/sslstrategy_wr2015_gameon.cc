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

#include "sslstrategy_wr2015_gameon.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_GameOn::name() {
    return "SSLStrategy_WR2015_GameOn";
}

SSLStrategy_WR2015_GameOn::SSLStrategy_WR2015_GameOn() {
	_pb_atk = NULL;
	_pb_defense = NULL;
	_pb_goal = NULL;
}

void SSLStrategy_WR2015_GameOn::configure(int numOurPlayers) {
    _mode = AGGRESSIVE;
//    _mode = DEFENSIVE;

    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_atk = new Playbook_Attack());
    usesPlaybook(_pb_defense = new Playbook_Defense());
}

void SSLStrategy_WR2015_GameOn::run(int numOurPlayers) {

    // RESET PLAYBOOKS
    _pb_defense->setMarkDistance();
    _pb_defense->setAdvanceToBall();
    _pb_defense->setDisableMarkBall();
    _pb_defense->setMarkBetweenBall();

    // ADD PLAYERS
    quint8 gk = dist()->getGK();

    /// Attack (1, Attacker)
    quint8 attacker = (numOurPlayers == 1)?gk:dist()->getKicker(loc()->ball());
    _pb_atk->addPlayer(attacker);
    _pb_atk->setKicker(attacker);

    /// Goal (1, GK)
    if(numOurPlayers >= 2) {
        _pb_goal->addPlayer(gk);
        _pb_goal->setGK(gk);
    }

    /// Dynamic players
    // Their field
    if(loc()->isInsideTheirField(loc()->ball())) {

        if(dist()->playersAvailable() >= 2) { // 2 or more: MB/GKA and Recv
            // MB/GKA
            if(_mode == AGGRESSIVE) {
                quint8 mb = dist()->getOneKNN(loc()->ball());
                _pb_defense->addPlayer(mb);
                _pb_defense->setAdvanceToBall(false);
                _pb_defense->setMarkDistance(DEFENSE_MB_MARKDISTANCE);
            } else {
                quint8 gka1 = dist()->getOneKNN(loc()->ourGoal());
                _pb_goal->addPlayer(gka1);
            }

            // Recv
            quint8 recvM = dist()->getOneKNN(loc()->theirGoal());
            _pb_atk->addPlayer(recvM);

        } else { // Only one: Recv
            // Recv
            quint8 recvM = dist()->getOneKNN(loc()->theirGoal());
            _pb_atk->addPlayer(recvM);
        }

        // Recv
        quint8 recvMR = dist()->getOneKNN(loc()->theirGoal());
        _pb_atk->addPlayer(recvMR);

        // Recv/GKA
        if(_mode == AGGRESSIVE) {
            quint8 recvFW = dist()->getOneKNN(loc()->theirGoal());
            _pb_atk->addPlayer(recvFW);
        } else {

            // Check opponents
            bool oppNearOurGoal = false;
            QList<kNN::element> opps = utils()->getOpponentKNN(1, loc()->ourGoal());
            if(opps.size() != 0) {
                float oppDistOurGoal = PlayerBus::theirPlayer(opps.first().id)->distTheirGoal();
                if(oppDistOurGoal < loc()->fieldMaxX()*0.75f)
                    oppNearOurGoal = true;
            }

            // Opponent near our goal
            if(oppNearOurGoal) {
                Position oppPlayerPos = PlayerBus::theirPlayer(opps.first().id)->position();
                quint8 mp = dist()->getOneKNN(oppPlayerPos);
                _pb_defense->addPlayer(mp);
                _pb_defense->setDisableMarkBall(true);
                _pb_defense->setMarkBetweenBall(true);

            // Our goal is ""safe""
            } else {
                quint8 gka2 = dist()->getOneKNN(loc()->ball());
                _pb_goal->addPlayer(gka2);
            }

        }

    // Our field
    } else {
        const float ballDistOurGoal = WR::Utils::distance(loc()->ball(), loc()->ourGoal());

        // Check if has opponent near the ball
        bool oppNearBall = false;
        QList<kNN::element> opps = utils()->getOpponentKNN(1, loc()->ball());
        if(opps.size() != 0) {
            float oppDistBall = PlayerBus::theirPlayer(opps.first().id)->distanceTo(loc()->ball());
            if(oppDistBall < loc()->fieldMaxX()/2.0f)
                oppNearBall = true;
        }

        // Ball near our goal
        if(ballDistOurGoal < 3.0f*loc()->fieldDefenseRadius() && oppNearBall) {
            // GKA
            quint8 gka1 = dist()->getOneKNN(loc()->ourGoal());
            _pb_goal->addPlayer(gka1);

            // GKA
            quint8 gka2 = dist()->getOneKNN(loc()->ourGoal());
            _pb_goal->addPlayer(gka2);

            // MPs
            QList<quint8> mps = dist()->getKNN(2, loc()->ourGoal());
            _pb_defense->addPlayers(mps);
            _pb_defense->setDisableMarkBall(true);

        // Ball away from our goal
        } else {

            // Opponent near ball
            if(oppNearBall) {
                // MB
                quint8 mb = dist()->getOneKNN(loc()->ball());
                _pb_defense->addPlayer(mb);
                _pb_defense->setAdvanceToBall(false);
                _pb_defense->setMarkDistance(DEFENSE_MB_MARKDISTANCE);

                // MPs
                QList<quint8> mps = dist()->getKNN(3, loc()->ourGoal());
                _pb_defense->addPlayers(mps);

            // Anyone near the ball
            } else {

                // Aggressive mode or no opponents
                if(_mode == AGGRESSIVE || opps.size() == 0) {
                    // RecvM
                    quint8 recvM = dist()->getOneKNN(loc()->theirGoal());
                    _pb_atk->addPlayer(recvM);

                    // GKAs
                    QList<quint8> gkas = dist()->getKNN(2, loc()->ourGoal());
                    _pb_goal->addPlayers(gkas);

                    // RecvMR
                    quint8 recvMR = dist()->getOneKNN(loc()->ball());
                    _pb_atk->addPlayer(recvMR);

                // Defensive mode and has opponents
                } else {
                    QList<kNN::element> opps = utils()->getOpponentKNN(2, loc()->ourGoal());

                    // MB
                    quint8 mb = dist()->getOneKNN(loc()->ball());
                    _pb_defense->addPlayer(mb);

                    // MP
                    Position nearestOppPos1 = PlayerBus::theirPlayer(opps.takeFirst().id)->position();
                    quint8 mp1 = dist()->getOneKNN(nearestOppPos1);
                    _pb_defense->addPlayer(mp1);
                    _pb_defense->setMarkDistance(DEFENSE_MB_MARKDISTANCE);

                    // GKA
                    quint8 gka = dist()->getOneKNN(loc()->ourGoal());
                    _pb_goal->addPlayer(gka);

                    // MP
                    Position nearestOppPos2 = PlayerBus::theirPlayer(opps.first().id)->position();
                    quint8 mp2 = dist()->getOneKNN(nearestOppPos2);
                    _pb_defense->addPlayer(mp2);

                }

            }

        }

    }


}
