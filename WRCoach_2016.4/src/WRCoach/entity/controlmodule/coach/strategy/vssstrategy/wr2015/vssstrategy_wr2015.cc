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

#include "vssstrategy_wr2015.hh"
#include "vssstrategy_wr2015_states.hh"

VSSStrategy_WR2015::VSSStrategy_WR2015() {

}

QString VSSStrategy_WR2015::name() {
	return "VSSStrategy_WR2015";
}

void VSSStrategy_WR2015::configure() {
	setStrategyState(HALT, new VSSStrategy_WR2015_Halt());
	setStrategyState(GAMEON, new VSSStrategy_WR2015_GameOn());
	setStrategyState(GAMEOFF, new VSSStrategy_WR2015_GameOff());
}
