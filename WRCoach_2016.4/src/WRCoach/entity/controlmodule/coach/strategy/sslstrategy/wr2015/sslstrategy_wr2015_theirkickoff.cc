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

#include "sslstrategy_wr2015_theirkickoff.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_TheirKickOff::name() {
    return "SSLStrategy_WR2015_TheirKickOff";
}

SSLStrategy_WR2015_TheirKickOff::SSLStrategy_WR2015_TheirKickOff() {
	_pb_defense = NULL;
	_pb_goal = NULL;
}

void SSLStrategy_WR2015_TheirKickOff::configure(int numOurPlayers) {
    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_defense = new Playbook_Defense());
}

void SSLStrategy_WR2015_TheirKickOff::run(int numOurPlayers) {
    // ADD PLAYERS
    // Goal (1, GK)
    quint8 gk = dist()->getGK();
    _pb_goal->addPlayer(gk);
    _pb_goal->setGK(gk);

    // Defense (1, MarkBall)
    quint8 mb = dist()->getOneKNN(loc()->fieldCenter());
    _pb_defense->addPlayer(mb);

    // Get opp. players to mark and parse
    QList<kNN::element> opps = utils()->getOpponentKNN(6, loc()->fieldCenter());
    for(int i=0; i<opps.size(); i++) {
        quint8 oppId = opps.at(i).id;
        if(isKickoffMarkable(oppId)==false) {
            opps.removeAt(i);
            i--;
        }
    }

    // Allocate defense players for opp. players
    while(opps.empty()==false && dist()->hasPlayersAvailable()) {
        quint8 oppId = opps.takeFirst().id;
        Position oppPos = PlayerBus::theirPlayer(oppId)->position();

        // Defense (MarkPlayers)
        quint8 mp = dist()->getOneKNN(oppPos);
        _pb_defense->addPlayer(mp);
    }

    // Goal (2, GKAs)
    QList<quint8> gkas = dist()->getKNN(2, loc()->ourGoal());
    _pb_goal->addPlayers(gkas);

    // Defense (All, MarkBallA)
    QList<quint8> mbas = dist()->getAllPlayers();
    _pb_defense->addPlayers(mbas);

    // CONFIGURE
    _pb_defense->setAdvanceToBall(false);
    _pb_defense->setCanGoToOpTeamFieldSide(false);
}

bool SSLStrategy_WR2015_TheirKickOff::isKickoffMarkable(quint8 oppId) {
    // Check position
    Position oppPos = PlayerBus::theirPlayer(oppId)->position();
    if(oppPos.isUnknown())
        return false;

    // Check near goal (defense players)
    float distOppGoal = PlayerBus::theirPlayer(oppId)->distOurGoal();
    if(distOppGoal <= loc()->fieldMaxX()/2.0)
        return false;

    // Check near field center
    float distCenter = PlayerBus::theirPlayer(oppId)->distanceTo(loc()->fieldCenter());
    if(distCenter <= 1.0f)
        return false;

    return true;
}
