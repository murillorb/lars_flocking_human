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

#include "sslstrategy_wr2015_gameoff.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_GameOff::name() {
    return "SSLStrategy_WR2015_GameOff";
}

SSLStrategy_WR2015_GameOff::SSLStrategy_WR2015_GameOff() {
    _pb_goal = NULL;
    _pb_defense = NULL;
    _pb_pos = NULL;
}

void SSLStrategy_WR2015_GameOff::configure(int numOurPlayers) {
    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_defense = new Playbook_Defense());
    usesPlaybook(_pb_pos = new Playbook_Pos());
}

void SSLStrategy_WR2015_GameOff::run(int numOurPlayers) {

    // ADD PLAYERS
    // Goal (1, GK)
    quint8 gk = dist()->getGK();
    _pb_goal->addPlayer(gk);
    _pb_goal->setGK(gk);
    reduceSpeed(gk);

    // GKA or MB
    if(loc()->distBallOurGoal() <= 2.5*loc()->fieldDefenseRadius()) {
        // Goal (1, GKA)
        quint8 gka = dist()->getOneKNN(loc()->ourGoal());
        _pb_goal->addPlayer(gka);
        reduceSpeed(gka);
    } else {
        // Defense (1, MarkBall)
        quint8 mb = dist()->getOneKNN(loc()->ball());
        _pb_defense->addPlayer(mb);
        reduceSpeed(mb);
    }

    // Goal (1, GKA)
    quint8 gka = dist()->getOneKNN(loc()->ourGoal());
    _pb_goal->addPlayer(gka);
    reduceSpeed(gka);

    // Pos or Defense
    if(loc()->distBallOurGoal() <= 2.0*loc()->fieldDefenseRadius()) {
        // Add all to pos to avoid collisions
        _pb_pos->addPlayers(dist()->getAllPlayers());
        _pb_pos->setPosition(loc()->ball());
        _pb_pos->setDist(0.6f);
        _pb_pos->setOurSide(false);

    } else {
        // Defense (3, MarkPlayer/MarkBallA)
        QList<quint8> defense = dist()->getAllPlayers();
        _pb_defense->addPlayers(defense);

    }


    // CONFIGURE
    _pb_defense->setAdvanceToBall(false);
    _pb_defense->setCanGoToOpTeamFieldSide(true);
}

void SSLStrategy_WR2015_GameOff::reduceSpeed(quint8 id) {
    if(PlayerBus::ourPlayerAvailable(id))
        PlayerBus::ourPlayer(id)->setMaxLSpeed(GlobalConstants::gameOffMaxLSpeed());
}
