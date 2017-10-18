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

#include "vssbehavior_mark.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

VSSBehavior_Mark::VSSBehavior_Mark() {
	_markId = -1;
	_selectedMarkId = -1;

	_sk_goTo = NULL;

	_state = STATE_MARK;
}
QString VSSBehavior_Mark::name() {
	return "VSSBehavior_Mark";
}

void VSSBehavior_Mark::configure() {
	usesSkill(_sk_goTo = new Skill_GoTo);
}

void VSSBehavior_Mark::run() {
	if(_markId == -1)
		std::cout << "[WARNING] VSSBehavior_Mark: _markId not set.";

	switch(_state) {
        default:
		case STATE_MARK: {
            // Accept to mark id while running
            _selectedMarkId = _markId;

            // Check if opp. to mark is available
            if(PlayerBus::theirPlayerAvailable(_markId)==false)
                return;

            // Re-enable robot stuck check
			player()->setRobotStuckEnabled(true);

            Position desiredPosition = WR::Utils::threePoints(PlayerBus::theirPlayer(_markId)->position(), loc()->ball(), 0.15, 0.0);
			_sk_goTo->setDestination(desiredPosition);
            _sk_goTo->avoidAll(true);
			_sk_goTo->avoidGoalArea(true);

            // Switch state condition: in position
			if(player()->isNearbyPosition(desiredPosition, player()->lError()))
				_state = STATE_STOP;

		} break;

		case STATE_STOP: {
            // Reset state condition: player doesn't exists anymore
            if(PlayerBus::theirPlayerAvailable(_selectedMarkId)==false) {
				_state = STATE_MARK;
				break;
			}

            // Disable robot stuck check
			player()->setRobotStuckEnabled(false);

            // Calc position to intercept robot
            Position desiredPosition = WR::Utils::threePoints(loc()->ball(), PlayerBus::theirPlayer(_selectedMarkId)->position(), player()->distBall(), 0.0);
			_sk_goTo->setDestination(desiredPosition);
			_sk_goTo->avoidBall(false);
            _sk_goTo->avoidTeammates(true);
			_sk_goTo->avoidOpponents(false);
			_sk_goTo->avoidGoalArea(true);

            // Switch state condition: opponent escaped from interception
            if(PlayerBus::theirPlayer(_selectedMarkId)->distanceTo(player()->position()) > 0.20)
				_state = STATE_MARK;

		}

	}
}
