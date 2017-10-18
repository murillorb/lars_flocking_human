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

#include "sslstrategy_wr2015_ourkickoff.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_WR2015_OurKickOff::name() {
    return "SSLStrategy_WR2015_OurKickOff";
}

SSLStrategy_WR2015_OurKickOff::SSLStrategy_WR2015_OurKickOff() {
    _pb_ourkickoff = NULL;
	_pb_goal = NULL;
    _pb_pos = NULL;
}

void SSLStrategy_WR2015_OurKickOff::configure(int numOurPlayers) {
    usesPlaybook(_pb_ourkickoff = new Playbook_OurKickOff());
    usesPlaybook(_pb_goal = new Playbook_Goal());
    usesPlaybook(_pb_pos = new Playbook_Pos());
}

void SSLStrategy_WR2015_OurKickOff::run(int numOurPlayers) {
    // ADD PLAYERS
    const quint8 gk = dist()->getGK();

    // OurKickOff (1, Kicker)
    quint8 kicker = numOurPlayers==1? gk : dist()->getKicker(loc()->fieldCenter());
    _pb_ourkickoff->setKicker(kicker);
    _pb_ourkickoff->addPlayer(kicker);

    // Goal (1, GK)
    if(numOurPlayers >= 2) {
        _pb_goal->setGK(gk);
        _pb_goal->addPlayer(gk);
    }

    // Goal (1, GKA)
    quint8 gka1 = dist()->getOneKNN(loc()->ourGoal());
    _pb_goal->addPlayer(gka1);

    // OurKickOff (1, Asistant)
    quint8 assistant1 = dist()->getOneKNN(loc()->fieldCenter());
    if(canKickBall())
        _pb_ourkickoff->addPlayer(assistant1);
    else
        _pb_pos->addPlayer(assistant1);

    // OurKickOff (1, Asistant)
    quint8 assistant2 = dist()->getOneKNN(loc()->fieldCenter());
    if(canKickBall())
        _pb_ourkickoff->addPlayer(assistant2);
    else
        _pb_pos->addPlayer(assistant2);

    // Goal (1, GKA)
    quint8 gka2 = dist()->getOneKNN(loc()->ourGoal());
    _pb_goal->addPlayer(gka2);


    // CONFIGURE
    _pb_pos->setPosition(loc()->fieldCenter());
    _pb_pos->setDist(0.6f);
}
