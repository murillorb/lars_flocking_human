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

#include "sslstrategy_test.hh"
#include "sslstrategy_test_states.hh"

QString SSLStrategy_Test::name() {
    return "SSLStrategy_Test";
}

SSLStrategy_Test::SSLStrategy_Test() {

}

void SSLStrategy_Test::configure() {
    setStrategyState(HALT, new SSLStrategy_Test_Halt());
    setStrategyState(GAMEOFF, new SSLStrategy_Test_GameOff());
    setStrategyState(GAMEON, new SSLStrategy_Test_GameOn());
}
