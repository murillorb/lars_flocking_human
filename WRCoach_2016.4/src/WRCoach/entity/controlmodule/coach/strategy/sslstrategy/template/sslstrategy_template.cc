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

#include "sslstrategy_template.hh"

/// Include your strategy states header file here
#include "sslstrategy_template_states.hh"

QString SSLStrategy_Template::name() {
    return "SSLStrategy_Template";
}

SSLStrategy_Template::SSLStrategy_Template() {

}

void SSLStrategy_Template::configure() {
    /// Set your strategy states here
    setStrategyState(HALT, new SSLStrategy_Template_Halt());
    setStrategyState(GAMEOFF, new SSLStrategy_Template_GameOFF());
    setStrategyState(GAMEON, new SSLStrategy_Template_GameON());
    setStrategyState(OURKICKOFF, new SSLStrategy_Template_OurKickoff());
    setStrategyState(OURDIRECTKICK, new SSLStrategy_Template_OurDirectKick());
    setStrategyState(OURINDIRECTKICK, new SSLStrategy_Template_OurIndirectKick());
    setStrategyState(OURPENALTY, new SSLStrategy_Template_OurPenalty());
    setStrategyState(THEIRKICKOFF, new SSLStrategy_Template_TheirKickoff());
    setStrategyState(THEIRDIRECTKICK, new SSLStrategy_Template_TheirDirectKick());
    setStrategyState(THEIRINDIRECTKICK, new SSLStrategy_Template_TheirIndirectKick());
    setStrategyState(THEIRPENALTY, new SSLStrategy_Template_TheirPenalty());

    /// ATTENTION:
    /// Please remember to remove #pragma directives from strategy states
    /// if you copy template files for a new strategy
}
