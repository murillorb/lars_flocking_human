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

#ifndef PLAYERACCESS_H
#define PLAYERACCESS_H

#include <WRCoach/entity/player/baseplayer.hh>

class PlayerAccess {
private:
    bool _self;
    Player *_player;
    PlayerUtils *_utils;
    Locations *_loc;
public:
    PlayerAccess(bool self, Player *player, PlayerUtils *utils, Locations *loc);
    virtual ~PlayerAccess();

    // Kick
    void kick(float power = 8.0);

    // Utils
    PlayerUtils* utils() { return _utils; }

    // General player info
    quint8 playerId() const;
    quint8 teamId() const;
    quint8 opTeamId() const;

    // Robot auxiliary methods
    // Dimensions
    float robotRadius() const;
    float robotLength() const;
    // Position
    Position position() const;
    Position nextPosition() const;
    // Orientation
    virtual Angle orientation() const;
    Angle nextOrientation() const;
    // Velocity
    Velocity velocity() const;
	float lastSpeed() const;
	// Direction
	Angle nextDirection() const;
    // Booleans
    bool isLookingTo(const Position &pos) const;
    bool isLookingTo(const Position &pos, float error) const;
    bool isAtPosition(const Position &pos) const;
    bool isNearbyPosition(const Position &pos, float error) const;
    bool hasBallPossession() const;
    bool canKickBall() const;
    // Distances
	float distanceTo(const Position &pos) const;
    float distBall() const;
    float distOurGoal() const;
    float distTheirGoal() const;
	// Angles
	Angle angleTo(const Position &pos) const;
    // Errors
    float lError() const;
    float aError() const;
	// PID
    void setLinearPIDParameters(double kp, double ki, double kd, double ilimit);
    void setAngularPIDParameters(double kp, double ki, double kd, double ilimit);
    // Maximum Linear Speed
	void setMaxLSpeed(float maxLSpeed);
    // Maximum Linear Acceleration
	void setMaxLAcceleration(float maxLAcceleration);

    // Anti-stuck (VSS only)
    void setFieldWallStuckEnabled(bool value = true);
    void setRobotStuckEnabled(bool value = true);
    void setGoalWallsStuckEnabled(bool value = true);
};

#endif // PLAYERACCESS_H
