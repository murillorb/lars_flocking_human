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

#include "coach.hh"
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>
#include <WRCoach/entity/controlmodule/coach/strategy/strategy.hh>
#include <WRCoach/entity/controlmodule/coach/coachutils.hh>
#include <WRCoach/entity/player/playerbus.hh>

QString Coach::name() {
    return "Coach";
}

Coach::Coach(SSLReferee *ref, WRTeam *ourTeam, WRTeam *opTeam) {
    _ref = ref;
    _ourTeam = ourTeam;
    _opTeam = opTeam;
	_strat = NULL;

    // Initialize PlayerBus
    PlayerBus::initialize(ourTeam, opTeam);

    // Coach utils
    _utils = new CoachUtils(ourTeam);
}

Coach::~Coach() {
    // Delete strategy
    if(_strat != NULL)
        delete _strat;

    // Delete coach utils
    delete _utils;
}


void Coach::run() {

    // Check if has players
    if(_ourTeam->avPlayersSize()==0) {
        std::cout << "[COACH] No players available!\n";
        return;
    }

    // Get Strategy
    Strategy *strat = strategy();

    // Run
    if(strat != NULL) {
        if(strat->isInitialized()==false)
            strat->initialize(_ref, _ourTeam, _opTeam, _utils);
        strat->runStrategy();
    }
}

void Coach::setStrategy(Strategy *strat) {
	_mutexStrategy.lock();
    // Delete old
    if(_strat != NULL)
        delete _strat;
    // Set new
	_strat = strat;
	_mutexStrategy.unlock();
}

Strategy* Coach::strategy() {
	_mutexStrategy.lock();
    Strategy *s = _strat;
	_mutexStrategy.unlock();
	return s;
}
