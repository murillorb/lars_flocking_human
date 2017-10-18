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

#ifndef VSSPLAYER_HH_
#define VSSPLAYER_HH_

#include <WRCoach/entity/controlmodule/coach/basecoach.hh>
#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/entity/player/player.hh>

class VSSPlayer: public Player {
public:
    VSSPlayer(World *world, WRTeam *team, quint8 playerId, Controller *ctr, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg, Behavior *defaultBehavior, SSLReferee *ref);
    virtual ~VSSPlayer();

    // Player config
    float robotRadius() const;
    float robotLength() const;

	// Orientation VSS implementation
	Angle orientation() const;

    // isLookintTo VSS implementation
    bool isLookingTo(const Position &pos);
    bool isLookingTo(const Position &pos, float error);

	// Robot movement
    void _goToLookTo(const Position &destination, const Position &positionToLook, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea);
	Position limitFieldDimensions(Position destination);
    bool cameraSpeedCheckEnabled() const;
    float cameraSpeedCheckFactor() const;
    bool curveLSpeedReductionEnabled() const;

	// Robot reversion
	void setReversionAngle(float angle) { if(angle>Angle::pi/2 && angle<Angle::pi) _reversionAngle = angle; }

    // Anti-stuck
    void setFieldWallStuckEnabled(bool value = true) { _fieldWallStuckEnabled = value; }
    void setRobotStuckEnabled(bool value = true) { _robotStuckEnabled = value; }
    void setGoalWallsStuckEnabled(bool value = true) { _goalWallsStuckEnabled = value; }

private:
    // Player reset
    void reset();

    // Robot reversion
	bool _reversed;
	float _reversionAngle;

    void processRobotReversion(const Angle &direction);
    void revertRobot();

	// Robot anti-stuck
    bool _fieldWallStuckEnabled;
    bool _robotStuckEnabled;
    bool _goalWallsStuckEnabled;

    bool _avoidingGoalWalls;

    void processAntiStuck();
    bool isFieldWallStuck();
    bool isRobotStuck();
    bool isGoalWallsStuck();
};

#endif /* VSSPLAYER_HH_ */
