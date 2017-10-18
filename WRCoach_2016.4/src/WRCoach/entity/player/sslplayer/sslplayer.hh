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

#ifndef SSLPLAYER_HH_
#define SSLPLAYER_HH_

#include <WRCoach/entity/controlmodule/coach/basecoach.hh>
#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/entity/player/player.hh>

class SSLPlayer : public Player {
public:
    SSLPlayer(World *world, WRTeam *team, quint8 playerId, Controller *ctr, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg, Behavior *defaultBehavior, SSLReferee *ref);
    virtual ~SSLPlayer();

    // Player config
    float robotRadius() const;
    float robotLength() const;

    bool cameraSpeedCheckEnabled() const;
    float cameraSpeedCheckFactor() const;
    bool curveLSpeedReductionEnabled() const;

    void _goToLookTo(const Position &destination, const Position &positionToLook, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea);
    Position limitFieldDimensions(Position destination);
private:
    // Player reset
    void reset();

    // Reimplements getNavDirectionDistance() to fix SSL different coordinate system
    std::pair<Angle,float> getNavDirectionDistance(const Position &destination, const Angle &positionToLook, bool avoidTeammates, bool avoidOpponents, bool avoidBall, bool avoidGoalArea, bool avoidTheirGoalArea);
};

#endif /* SSLPLAYER_HH_ */
