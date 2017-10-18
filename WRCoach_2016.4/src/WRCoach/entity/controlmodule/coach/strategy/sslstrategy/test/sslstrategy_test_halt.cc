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

#include "sslstrategy_test_halt.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_Test_Halt::name() {
    return "SSLStrategy_Test_Halt";
}

SSLStrategy_Test_Halt::SSLStrategy_Test_Halt() {
    _pb_doNothing = NULL;
}

void SSLStrategy_Test_Halt::configure(int numOurPlayers) {
    usesPlaybook(_pb_doNothing = new Playbook_DoNothing());
}

void SSLStrategy_Test_Halt::run(int numOurPlayers) {
    _pb_doNothing->addPlayers(dist()->getAllPlayers());
}
