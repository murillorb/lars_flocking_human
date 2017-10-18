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

#include "sslstrategy_template_theirindirectkick.hh"
#include <WRCoach/entity/controlmodule/coach/strategy/wrstrategystate.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

QString SSLStrategy_Template_TheirIndirectKick::name() {
    return "SSLStrategy_Template_THEIRINDIRECTKICK";
}

SSLStrategy_Template_TheirIndirectKick::SSLStrategy_Template_TheirIndirectKick() {

}

void SSLStrategy_Template_TheirIndirectKick::configure(int numOurPlayers) {
    std::cout << "SSLStrategy_Template_THEIRINDIRECTKICK::configurate(" << numOurPlayers << ")\n";
}

void SSLStrategy_Template_TheirIndirectKick::run(int numOurPlayers) {
    std::cout << "SSLStrategy_Template_THEIRINDIRECTKICK::run()\n";
}
