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

#include "vssstrategy_wr2015_gameoff.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString VSSStrategy_WR2015_GameOff::name() {
	return "VSSStrategy_WR2015_GameOff";
}

VSSStrategy_WR2015_GameOff::VSSStrategy_WR2015_GameOff() {
	_pb_pos = NULL;
}

void VSSStrategy_WR2015_GameOff::configure(int numOurPlayers) {
	usesPlaybook(_pb_pos = new Playbook_Pos());
}

void VSSStrategy_WR2015_GameOff::run(int numOurPlayers) {
    // Stay at field center (all)
    _pb_pos->addPlayers(dist()->getAllPlayers());
	_pb_pos->setDist(0.30);
	_pb_pos->setPosition(loc()->fieldCenter());
	_pb_pos->setOurSide(true);
}
