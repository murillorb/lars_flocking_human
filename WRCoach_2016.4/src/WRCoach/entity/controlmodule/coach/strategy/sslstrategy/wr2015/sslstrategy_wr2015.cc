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

#include "sslstrategy_wr2015.hh"
#include "sslstrategy_wr2015_states.hh"

QString SSLStrategy_WR2015::name() {
    return "SSLStrategy_WR2015";
}

SSLStrategy_WR2015::SSLStrategy_WR2015() {

}

void SSLStrategy_WR2015::configure() {
	setStrategyState(HALT, new SSLStrategy_WR2015_Halt());
    setStrategyState(GAMEOFF, new SSLStrategy_WR2015_GameOff());
	setStrategyState(GAMEON, new SSLStrategy_WR2015_GameOn());
	setStrategyState(OURPENALTY, new SSLStrategy_WR2015_OurPenalty());
	setStrategyState(THEIRPENALTY, new SSLStrategy_WR2015_TheirPenalty());
	setStrategyState(OURINDIRECTKICK, new SSLStrategy_WR2015_OurIndirectKick());
	setStrategyState(OURDIRECTKICK, new SSLStrategy_WR2015_OurDirectKick());
	setStrategyState(OURKICKOFF, new SSLStrategy_WR2015_OurKickOff());
    setStrategyState(THEIRKICKOFF, new SSLStrategy_WR2015_TheirKickOff());
	setStrategyState(THEIRDIRECTKICK, new SSLStrategy_WR2015_TheirDirectKick());
	setStrategyState(THEIRINDIRECTKICK, new SSLStrategy_WR2015_TheirIndirectKick());
}
