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

#include "vssstrategy_wr2016_freeball.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSStrategy_WR2016_FreeBall::name() {
    return "VSSStrategy_WR2016_FreeBall";
}

VSSStrategy_WR2016_FreeBall::VSSStrategy_WR2016_FreeBall() {
	_pb_freeball = NULL;
	_pb_goal = NULL;
    _pb_pos = NULL;
}

void VSSStrategy_WR2016_FreeBall::configure(int numOurPlayers) {
	usesPlaybook(_pb_freeball = new VSSPlaybook_FreeBall());
	usesPlaybook(_pb_goal = new VSSPlaybook_Goal());
    usesPlaybook(_pb_pos = new Playbook_Pos());
}

void VSSStrategy_WR2016_FreeBall::run(int numOurPlayers) {
	// Solo (GK-attacker)
	if(numOurPlayers == 1) {
        quint8 fb = dist()->getPlayer();
        _pb_freeball->addPlayer(fb);
	}

	// 2 or more
	if(numOurPlayers >= 2) {
        // GK
        quint8 gk = dist()->getOneKNN(loc()->ourGoal());
		_pb_goal->addPlayer(gk);

        // FreeBall player
        quint8 fb = dist()->getOneKNN(loc()->ball());
        _pb_freeball->addPlayer(fb);

        // Assistant player
        quint8 pos = dist()->getOneKNN(loc()->theirGoal());
        _pb_pos->addPlayer(pos);

        // Calc and set position for assistant player
        Position position;
        if(loc()->isInsideOurField(loc()->ball()))
            position = WR::Utils::threePoints(loc()->ourGoal(), loc()->fieldCenter(), loc()->fieldMaxX()/4, 0.0);
        else
            position = WR::Utils::threePoints(loc()->theirGoal(), loc()->fieldCenter(), loc()->fieldMaxX()/4, 0.0);
        _pb_pos->setPosition(position);
        _pb_pos->setDist(0.0);
	}
}


