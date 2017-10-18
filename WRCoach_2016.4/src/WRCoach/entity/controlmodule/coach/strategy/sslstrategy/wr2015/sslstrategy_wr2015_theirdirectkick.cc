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

#include "sslstrategy_wr2015_theirdirectkick.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_TheirDirectKick::name() {
    return "SSLStrategy_WR2015_TheirDirectKick";
}

SSLStrategy_WR2015_TheirDirectKick::SSLStrategy_WR2015_TheirDirectKick() {
	_pb_goal = NULL;
	_pb_defense = NULL;
}

void SSLStrategy_WR2015_TheirDirectKick::configure(int numOurPlayers) {
    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_defense = new Playbook_Defense());
}

void SSLStrategy_WR2015_TheirDirectKick::run(int numOurPlayers) {
    // ADD PLAYERS
    // Goal (1, GK)
    quint8 gk = dist()->getGK();
    _pb_goal->addPlayer(gk);
    _pb_goal->setGK(gk);

    // Defense (1, MarkBall)
    quint8 mb = dist()->getOneKNN(loc()->ball());
    _pb_defense->addPlayer(mb);

    // Goal (1, GKA)
    quint8 gka1 = dist()->getOneKNN(loc()->ourGoal());
    _pb_goal->addPlayer(gka1);

    // Defense (MarkPlayers)
    QList<kNN::element> opps = utils()->getOpponentKNN(6, loc()->ourGoal());

    // Defense (1, MarkPlayer/MarkBallA)
    Position opPos1 = loc()->ball();
    while(opps.empty()==false && dist()->hasPlayersAvailable()) {
        quint8 oppId = opps.takeFirst().id;

        // Check markable and select player
        if(isDirectMarkable(oppId)) {
            opPos1 = PlayerBus::theirPlayer(oppId)->position();
            break;
        }
    }
    quint8 defense1 = dist()->getOneKNN(opPos1);
    _pb_defense->addPlayer(defense1);

    // Defense (1, MarkPlayer/MarkBallA)
    Position opPos2 = loc()->ball();
    while(opps.empty()==false && dist()->hasPlayersAvailable()) {
        quint8 oppId = opps.takeFirst().id;

        // Check markable and select player
        if(isDirectMarkable(oppId)) {
            opPos2 = PlayerBus::theirPlayer(oppId)->position();
            break;
        }
    }
    quint8 defense2 = dist()->getOneKNN(opPos2);
    _pb_defense->addPlayer(defense2);

    // Goal (1, GKA)
    quint8 gka2 = dist()->getOneKNN(loc()->ourGoal());
    _pb_goal->addPlayer(gka2);


    // CONFIGURE
    _pb_defense->setAdvanceToBall(false);
    _pb_defense->setCanGoToOpTeamFieldSide(true);
}

bool SSLStrategy_WR2015_TheirDirectKick::isDirectMarkable(quint8 oppId) {
    // Check too close to ball
    if(PlayerBus::theirPlayer(oppId)->distBall() < 0.70f)
        return false;

    // Check op defense player (GK, etc)
    if(PlayerBus::theirPlayer(oppId)->distOurGoal() <= 1.5*loc()->fieldDefenseRadius())
        return false;

    return true;
}
