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

#include "behavior_followplayer.hh"
#include <WRCoach/entity/player/behavior/wrbehavior.hh>

QString Behavior_FollowPlayer::name() {
    return "Behavior_FollowPlayer";
}

Behavior_FollowPlayer::Behavior_FollowPlayer() {
	_radius = 0.5;
    _playerID = -1;
    _playerTeam = -1;
}

void Behavior_FollowPlayer::configure() {
    usesSkill(_sk_goTo = new Skill_GoTo());
}

void Behavior_FollowPlayer::run() {
    if(_playerID==-1 || _playerTeam==-1)
        std::cout << "[WARNING] Behavior_FollowPlayer, playerID or playerTeam not set!\n";

    // Check if player to follow is available
    if(PlayerBus::ourPlayerAvailable(_playerID)==false) {
        std::cout << "[WARNING] Player to follow ID not available!\n";
        return;
    }

    Position posToGo = player()->position();
    Position posToLook = loc()->ball();
    bool avoidRobots=true;

    Position posFollowNext;
    if(_playerTeam==player()->teamId())
        posFollowNext = PlayerBus::ourPlayer(_playerID)->nextPosition();
    else
        posFollowNext = PlayerBus::theirPlayer(_playerID)->nextPosition();

    // Follow player
    float dx=0.0, dy=0.0;
    if(posFollowNext.y()>=0)
        dy = _radius;
    else
        dy = -_radius;
    if(loc()->theirGoal().x()>=0)
        dx = 2*_radius;
    else
        dx = -2*_radius;
    posToGo.setPosition(posFollowNext.x()+dx, posFollowNext.y()+dy, 0.0);

    // Skill GoTo
    _sk_goTo->setDestination(posToGo);
    _sk_goTo->setPositionToLook(posToLook);
    _sk_goTo->avoidTeammates(avoidRobots);
    _sk_goTo->avoidOpponents(avoidRobots);
    _sk_goTo->avoidBall(false);
	_sk_goTo->avoidGoalArea(true);

}
