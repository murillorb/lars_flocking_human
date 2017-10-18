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

#ifndef PLAYER_HH
#define PLAYER_HH

#include <WRCoach/entity/controlmodule/coach/basecoach.hh>
#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/entity/entity.hh>
#include <WRCoach/utils/wrtimer/wrtimer.hh>

#define IDLE_COUNT 10

class Player : public Entity {
public:
    Player(World *world, WRTeam *team, quint8 playerId, Controller *ctr, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg, Behavior *defaultBehavior, SSLReferee *ref);
    virtual ~Player();

    QString name();

    /// Player info
    // General player info
    quint8 playerId() const;
    WRTeam* team() const;
    quint8 teamId() const;
    quint8 opTeamId() const;
    PlayerAccess* access() const;

    // Robot dimensions
    virtual float robotRadius() const = 0;
    virtual float robotLength() const = 0;

    /// Navigation
    // Path
    QLinkedList<Position> getPath() const;

    // Errors
    void setLError(float lError);
    float lError() const;
    void setAError(float aError);
    float aError() const;

    // Control algorithms
    // PID
    void setLinearPIDParameters(double kp, double ki, double kd, double iLimit);
    void setAngularPIDParameters(double kp, double ki, double kd, double iLimit);

    // DiscretePID
    void setLinearDiscretePIDParameters(double kp, double ki, double kd);
    void setAngularDiscretePIDParameters(double kp, double ki, double kd);

    // Cascade PID
    void setLinearCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed);
    void setAngularCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed);

    // Maximum speed
    void setMaxLSpeed(float maxLSpeed);
    void setMaxASpeed(float maxASpeed);
    float maxLSpeed();
    float maxASpeed();
    virtual bool cameraSpeedCheckEnabled() const = 0;
    virtual float cameraSpeedCheckFactor() const = 0;
    virtual bool curveLSpeedReductionEnabled() const = 0;

    // Maximum acceleration
    void setMaxLAcceleration(float maxLAcceleration);
    float maxLAcceleration();

    /// Behavior
    void setBehavior(Behavior *b);

    // Actions
    void kick(float power = 8.0);
    void chipKick(float power);
    void holdBall(bool value = true);

    /// Robot locomotion
    // Locomotion layer 2
    void goTo(const Position &destination, bool avoidBall=false, bool avoidTeammates=false, bool avoidOpponents=false, bool avoidOurGoalArea=false, bool avoidTheirGoalArea=false, bool limitFieldDimensions=true);
    void goToLookTo(const Position &destination, const Position &positionToLook=Position(false,0,0,0), bool avoidBall=false, bool avoidTeammates=false, bool avoidOpponents=false, bool avoidOurGoalArea=false, bool avoidTheirGoalArea=false, bool limitFieldDimensions=true);
	void lookTo(const Position &positionToLook);
	void spin(bool clockwise, bool fast);
    void idle();

    /// Auxiliary methods
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
    virtual bool isLookingTo(const Position &pos) const;
    virtual bool isLookingTo(const Position &pos, float error) const;
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

protected:
    /// Inheritance access
    PlayerUtils* utils() const { return _utils; }
    Locations* loc() const;

    /// Robot locomotion
    // Locomotion layer 1
    void move(float speedX, float speedY);
    void rotate(float angularSpeed);
    void moveAndRotate(float speedX, float speedY, float angularSpeed);

    // Robot route and speed calculation
    virtual std::pair<Angle,float> getNavDirectionDistance(const Position &destination, const Angle &positionToLook, bool avoidTeammates, bool avoidOpponents, bool avoidBall, bool avoidGoalArea, bool avoidTheirGoalArea);
    double getLinearSpeed(float distError);
    double getAngularSpeed(Angle angleToLook);

private:
    // Entity inherit virtual methods
    void initialization();
    void loop();
    void finalization();

    // Player reset on behavior changes
    virtual void reset();

    /// Robot locomotion
    // Locomotion base
    virtual void _goToLookTo(const Position &destination, const Position &positionToLook, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea) = 0;
	virtual Position limitFieldDimensions(Position destination) = 0;

    // Locomotion layer 0
    void setSpeed(float x, float y, float theta = 0);

	// Next state
	Position _nextPosition;
	Angle _nextOrientation;
	Angle _nextDirection;

    /// Player info
    // General player info
    WRTeam *_team;
    quint8 _playerId;

    // Player related objects
    Controller *_ctr;
    Navigation *_nav;
    PlayerUtils *_utils;
    SSLReferee *_ref;
    PlayerAccess *_playerAccessSelf;
    PlayerAccess *_playerAccessBus;

    // Behavior
    Behavior *_behavior;
    Behavior *_defaultBehavior;
    QMutex _mutexBehavior;

    // Acceleration control
    WRTimer _speedTimer;
	float _lastSpeedAbs;

    // World access for synchronization
    World *_world;

    // Idle control
	quint8 _setIdle;
};

#endif // PLAYER_HH
