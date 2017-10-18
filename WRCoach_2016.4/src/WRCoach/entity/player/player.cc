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

#include "player.hh"
#include <WRCoach/entity/referee/sslreferee/sslreferee.hh>
#include <WRCoach/entity/referee/sslreferee/sslgameinfo.hh>
#include <WRCoach/entity/player/navigation/navigation.hh>
#include <WRCoach/entity/player/behavior/behavior.hh>
#include <WRCoach/entity/player/playerutils.hh>
#include <WRCoach/entity/player/playeraccess.hh>
#include <WRCoach/entity/controlmodule/coach/wrteam.hh>
#include <WRCoach/entity/locations.hh>
#include <WRCoach/entity/world/world.hh>

QString Player::name() {
    return "Player #"+QString::number((int)_team->teamId())+":"+QString::number((int)_playerId);
}

Player::Player(World *world, WRTeam *team, quint8 playerId, Controller *ctr, NavigationAlgorithm *navAlg, ControlAlgorithm *linCtrAlg, ControlAlgorithm *angCtrAlg, Behavior *defaultBehavior, SSLReferee *ref)
 : Entity(Entity::ENT_PLAYER) {

    _world = world;
    _team = team;
    _playerId = playerId;
    _behavior = NULL;
    _defaultBehavior = defaultBehavior;
    _ref = ref;
    _nav = new Navigation(this, navAlg, linCtrAlg, angCtrAlg);
    _utils = new PlayerUtils(this, team->loc());
    _playerAccessSelf = new PlayerAccess(true, this, _utils, team->loc());
    _playerAccessBus = new PlayerAccess(false, this, _utils, team->loc());
    _ctr = ctr;

    // Acceleration control
    _lastSpeedAbs = 0;

    // Idle control
    _setIdle = 0;

    // Reset player
    reset();
}

Player::~Player() {
    if(_defaultBehavior!=NULL)
        delete _defaultBehavior;
    if(_nav!=NULL)
        delete _nav;
    if(_utils!=NULL)
        delete _utils;
    if(_playerAccessSelf!=NULL)
        delete _playerAccessSelf;
    if(_playerAccessBus!=NULL)
        delete _playerAccessBus;
}

void Player::reset() {
    /// Default reset for "Player"
    // Maximum speed
    setMaxLSpeed(0.25);
    setMaxASpeed(Angle::toRadians(180));

    // Maximum acceleration
    setMaxLAcceleration(9.82);

    // Errors
    setLError(0.015);
    setAError(Angle::toRadians(4.0));
}

/* Player info
 *
 */
quint8 Player::playerId() const {
    return _playerId;
}

WRTeam* Player::team() const {
    return _team;
}

quint8 Player::teamId() const {
    return _team->teamId();
}

quint8 Player::opTeamId() const {
    return _team->opTeam()->teamId();
}

PlayerAccess* Player::access() const {
    return _playerAccessBus;
}

QLinkedList<Position> Player::getPath() const {
    return _nav->getPath();
}

/* Entity inherit virtual methods
 *
 */
void Player::initialization() {
    printf("[TEAM %d PLAYER %2d] thread started.\n", (int)teamId(), (int)playerId());
}

void Player::loop() {

    // Lock for reading WorldMap
    _world->wmLockRead();

    // If robot was lost or isn't available, just send an idle()
    if(position().isUnknown()) {
        setBehavior(NULL);
        if(_setIdle < IDLE_COUNT) {
            _setIdle++;
            idle();
            kick(0.0);
        }

    // Known position
    } else {
        _setIdle = 0;

        // Set kick on touch off by default
        _ctr->kickOnTouch(teamId(), playerId(), false, 0.0);

        // Run behavior, if exists; if not, run default behavior
        _mutexBehavior.lock();
        if(_behavior != NULL) {
            if(_behavior->isInitialized()==false)
                _behavior->initialize(_team->loc());
            _behavior->setPlayer(this, _utils, _playerAccessSelf);
            _behavior->runBehavior();
        } else if(_defaultBehavior != NULL) {
            if(_defaultBehavior->isInitialized()==false)
                _defaultBehavior->initialize(_team->loc());
            _defaultBehavior->setPlayer(this, _utils, _playerAccessSelf);
            _defaultBehavior->runBehavior();
        } else {
            std::cout << "[ERROR] No behavior found for player #" << (int)playerId() << "!\n";
        }
        _mutexBehavior.unlock();

    }

    // Unlock
    _world->wmUnlock();
}

void Player::finalization() {
    if(isEnabled()) {
        for(int i=0; i<IDLE_COUNT; i++) {
            this->idle();
            _ctr->kickOnTouch(teamId(), playerId(), false, 0.0);
            this->msleep(this->loopTime()/4);
        }
    }
    printf("[TEAM %d PLAYER %2d] thread ended.\n", (int)teamId(), (int)playerId());
}

/* Behavior
 *
 */
void Player::setBehavior(Behavior* b) {
    // Check same behavior
    if(b==_behavior)
        return;

    // Set new
    _mutexBehavior.lock();
    _behavior = b;
    this->reset();
    _mutexBehavior.unlock();
}

void Player::kick(float power) {
    _ctr->kickOnTouch(teamId(), _playerId, (power==0.0f?false:true), power);
}

void Player::chipKick(float power) {
    _ctr->chipKickOnTouch(teamId(), _playerId, true, power);
}

void Player::holdBall(bool value) {
    _ctr->holdBall(teamId(), playerId(), value);
}

/* Auxiliary methods
 *
 */
Position Player::position() const {
    return _team->wm()->playerPosition(teamId(), _playerId);
}

Position Player::nextPosition() const {
    return this->_nextPosition;
}

Angle Player::orientation() const {
    return _team->wm()->playerOrientation(teamId(), _playerId);
}

Angle Player::nextOrientation() const {
    return this->_nextOrientation;
}

Velocity Player::velocity() const {
    return _team->wm()->playerVelocity(teamId(), _playerId);
}

float Player::lastSpeed() const {
    return this->_lastSpeedAbs;
}

Angle Player::nextDirection() const {
    return this->_nextDirection;
}

bool Player::isLookingTo(const Position &pos) const {
    return this->isLookingTo(pos, aError());
}

bool Player::isLookingTo(const Position &pos, float error) const {
    // Get angle that should be looking at
    Angle angle(true, WR::Utils::getAngle(position(), pos));

    // Calc dif
    float dif = fabs(WR::Utils::angleDiff(orientation(), angle));
    return (dif <= error);
}

bool Player::isAtPosition(const Position &pos) const {
    // Get distance
    const float distance = WR::Utils::distance(position(), pos);
    return (distance <= lError());
}

bool Player::isNearbyPosition(const Position &pos, float error) const {
    // Get distance
    const float distance = WR::Utils::distance(position(), pos);
    return (distance <= error);
}

bool Player::hasBallPossession() const {
    return _team->wm()->ballPossession(teamId(), _playerId);
}

bool Player::canKickBall() const {
    return _ref->getGameInfo(_team->teamColor())->canKickBall();
}

float Player::distanceTo(const Position &pos) const {
    return WR::Utils::distance(position(), pos);
}

float Player::distBall() const {
    return WR::Utils::distance(position(), _team->loc()->ball());
}

float Player::distOurGoal() const {
    return WR::Utils::distance(position(), _team->loc()->ourGoal());
}

float Player::distTheirGoal() const {
    return WR::Utils::distance(position(), _team->loc()->theirGoal());
}

Angle Player::angleTo(const Position &pos) const {
    return Angle(true, WR::Utils::getAngle(position(), pos));
}

/*
 *
 */
Locations* Player::loc() const {
    return _team->loc();
}

/* Navigation
 *
 */
void Player::setLError(float lError) {
    _nav->setLError(lError);
}

float Player::lError() const {
    return _nav->getLError();
}

void Player::setAError(float aError) {
    _nav->setAError(aError);
}

float Player::aError() const {
    return _nav->getAError();
}

void Player::setLinearPIDParameters(double kp, double ki, double kd, double iLimit) {
    _nav->setLinearPIDParameters(kp, ki, kd, iLimit);
}

void Player::setAngularPIDParameters(double kp, double ki, double kd, double iLimit) {
    _nav->setAngularPIDParameters(kp, ki, kd, iLimit);
}

void Player::setLinearDiscretePIDParameters(double kp, double ki, double kd) {
    _nav->setLinearDiscretePIDParameters(kp, ki, kd);
}

void Player::setAngularDiscretePIDParameters(double kp, double ki, double kd) {
    _nav->setAngularDiscretePIDParameters(kp, ki, kd);
}

void Player::setLinearCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed) {
    _nav->setLinearCascadePIDParameters(dkp, dkd, skp, ski, skd, siLimit, maxSpeed);
}

void Player::setAngularCascadePIDParameters(double dkp, double dkd, double skp, double ski, double skd, double siLimit, float maxSpeed) {
    _nav->setAngularCascadePIDParameters(dkp, dkd, skp, ski, skd, siLimit, maxSpeed);
}

void Player::setMaxLSpeed(float maxLSpeed) {
    _nav->setMaxLSpeed(maxLSpeed);
}

void Player::setMaxASpeed(float maxASpeed) {
    _nav->setMaxASpeed(maxASpeed);
}

float Player::maxLSpeed() {
    return _nav->getMaxLSpeed();
}

float Player::maxASpeed() {
    return _nav->getMaxASpeed();
}

void Player::setMaxLAcceleration(float maxLAcceleration) {
    _nav->setMaxLAcceleration(maxLAcceleration);
}

float Player::maxLAcceleration() {
    return _nav->getmaxLAcceleration();
}

/* Robot locomotion
 *
 */
void Player::goTo(const Position &destination, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea, bool limitFieldDimensions) {
    // goTo destination, look to anywhere
    goToLookTo(destination, Position(false, 0.0, 0.0, 0.0), avoidBall, avoidTeammates, avoidOpponents, avoidOurGoalArea, avoidTheirGoalArea, limitFieldDimensions);
}

void Player::goToLookTo(const Position &destination, const Position &positionToLook, bool avoidBall, bool avoidTeammates, bool avoidOpponents, bool avoidOurGoalArea, bool avoidTheirGoalArea, bool limitFieldDimensions) {
    // Check destination
    if(destination.isUnknown()) {
        std::cout << "[Player] ERROR: goToLookTo(destination, ...), destination is unknown!\n";
        idle();
        return;
    }

    // Limit field dimensions
    Position limitedDestination = (limitFieldDimensions? this->limitFieldDimensions(destination) : destination);

    // Set next state
    _nextPosition = limitedDestination;
    _nextOrientation.setValue(WR::Utils::getAngle(_nextPosition, positionToLook));

    // Call virtual GoTo
    _goToLookTo(limitedDestination, positionToLook, avoidBall, avoidTeammates, avoidOpponents, avoidOurGoalArea, avoidTheirGoalArea);
}

void Player::lookTo(const Position &positionToLook) {
    // goTo current position, look to positionToLook
	goToLookTo(position(), positionToLook);
}

void Player::spin(bool clockwise, bool fast) {
    float speed = GEARSystem::Angle::twoPi;
    if(clockwise)
		speed *= -1.0;
	if(fast)
        speed *= 4.0;
    rotate(speed);
}

void Player::idle() {
    // Set current position/orientation as desired
    _nextPosition = position();
    _nextOrientation = orientation();

    // Set speed
    this->setSpeed(0.0, 0.0, 0.0);

    // Reset navigation
    _nav->reset();
}

void Player::move(float speedX, float speedY) {
    this->setSpeed(speedX, speedY, 0.0);
}

void Player::rotate(float angularSpeed) {
    this->setSpeed(0.0, 0.0, angularSpeed);
}

void Player::moveAndRotate(float speedX, float speedY, float angularSpeed) {
    this->setSpeed(speedX, speedY, angularSpeed);
}

std::pair<Angle,float> Player::getNavDirectionDistance(const Position &destination, const Angle &positionToLook, bool avoidTeammates, bool avoidOpponents, bool avoidBall, bool avoidOurGoalArea, bool avoidTheirGoalArea) {
    _nav->setGoal(destination, positionToLook, avoidTeammates, avoidOpponents, avoidBall, avoidOurGoalArea, avoidTheirGoalArea);
    Angle direction = _nav->getDirection();
    float distance = _nav->getDistance();

    //std::cout << "Player, direction: " << GEARSystem::Angle::toDegrees(direction) << "\n";
    //std::cout << "Player, distance: " << distance << "\n";
    _nextDirection = direction;

    return std::make_pair(direction, distance);
}

double Player::getLinearSpeed(float distError) {
    return _nav->getLinearSpeed(distError, velocity());
}

double Player::getAngularSpeed(Angle angleToLook) {
    return _nav->getAngularSpeed(angleToLook.value() - orientation().value(), velocity());
}

void Player::setSpeed(float x, float y, float theta) {
    // Check curves: reduce speed
    if(curveLSpeedReductionEnabled()) {
        if(fabs(_nextDirection.value())>0 && (WR::Utils::angleDiff(_nextDirection, orientation()) > GEARSystem::Angle::toRadians(25)))
            _lastSpeedAbs *= 0.975;
    }

    // Check high speed request than real speed
    if(cameraSpeedCheckEnabled()) {
        if(_lastSpeedAbs>maxLSpeed()/2.0 && velocity().abs()>0.10 && _lastSpeedAbs > 2.0*velocity().abs())
            _lastSpeedAbs *= cameraSpeedCheckFactor();
    }

    // Max linear acceleration
    _speedTimer.stop();
    const float maxAcc = (maxLAcceleration()/1E3) * _speedTimer.timemsec();
    _speedTimer.start();

    // Iterate
    float currSpeedAbs = sqrt(pow(x,2)+pow(y,2));
    float incSpeedAbs = currSpeedAbs - _lastSpeedAbs;
    if(fabs(incSpeedAbs)>maxAcc && incSpeedAbs>0) { // Limit positive acceleration
        float newSpeed = _lastSpeedAbs + maxAcc;
        float angle = atan2(y, x);
        x = newSpeed*cos(angle);
        y = newSpeed*sin(angle);
    }

    // Save state
    _lastSpeedAbs = sqrt(pow(x,2)+pow(y,2));

    // Watchdog on speed
    WR::Utils::limitValue(&x, -maxLSpeed(), maxLSpeed());
    WR::Utils::limitValue(&y, -maxLSpeed(), maxLSpeed());

    // Controller setSpeed
    _ctr->setSpeed(teamId(), playerId(), x, y, theta);
}
