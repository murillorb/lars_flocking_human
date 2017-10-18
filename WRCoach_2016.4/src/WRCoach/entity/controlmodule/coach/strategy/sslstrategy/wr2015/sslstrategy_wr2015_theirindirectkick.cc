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

#include "sslstrategy_wr2015_theirindirectkick.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_TheirIndirectKick::name() {
    return "SSLStrategy_WR2015_TheirIndirectKick";
}

SSLStrategy_WR2015_TheirIndirectKick::SSLStrategy_WR2015_TheirIndirectKick() {
    _pb_goal = NULL;
    _pb_defense = NULL;
}

void SSLStrategy_WR2015_TheirIndirectKick::configure(int numOurPlayers) {
    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_defense = new Playbook_Defense());
}

void SSLStrategy_WR2015_TheirIndirectKick::run(int numOurPlayers) {
    // ADD PLAYERS
    // Goal (1, GK)
    quint8 gk = dist()->getGK();
    _pb_goal->addPlayer(gk);
    _pb_goal->setGK(gk);

    // Defense (1, MarkBall)
    quint8 mb = dist()->getOneKNN(loc()->ball());
    _pb_defense->addPlayer(mb);

    // Get players to mark and set MarkPlayers
    QList<kNN::element> ops = utils()->getOpponentKNN(6, loc()->ourGoal());
    while(ops.empty()==false) {

        // Check if still has our team players available
        if(dist()->hasPlayersAvailable()==false)
            break;

        // Get player to mark
        quint8 toMark = ops.takeFirst().id;
        Position toMarkPos = PlayerBus::theirPlayer(toMark)->position();

        // Check if still exists
        if(toMarkPos.isUnknown())
            continue;

        // Check if "markable"
        if(isMarkable(toMark)==false)
            continue;

        // Allocate MarkPlayer
        _pb_defense->addPlayer(dist()->getOneKNN(toMarkPos));
    }

    // Goal (2, GKAs)
    QList<quint8> gkas = dist()->getKNN(2, loc()->ourGoal());
    _pb_goal->addPlayers(gkas);

    // Defense (2, MarkPlayer/MarkBallA)
    QList<quint8> defense = dist()->getAllPlayers();
    _pb_defense->addPlayers(defense);

    // CONFIGURE
    _pb_defense->setAdvanceToBall(false);
    _pb_defense->setCanGoToOpTeamFieldSide(true);
    _pb_defense->setMarkBetweenBall(true);
}

bool SSLStrategy_WR2015_TheirIndirectKick::isMarkable(quint8 toMarkID) {
    // Check too close to ball
    if(PlayerBus::theirPlayer(toMarkID)->distBall() < 0.70f)
        return false;

    // Check outside attack area (our field side)
    Position toMarkPos = PlayerBus::theirPlayer(toMarkID)->position();
    if(loc()->isInsideTheirField(toMarkPos))
        return false;

    return true;
}
