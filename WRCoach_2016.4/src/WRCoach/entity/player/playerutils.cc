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

#include "playerutils.hh"
#include <WRCoach/entity/player/player.hh>
#include <WRCoach/entity/player/playerbus.hh>
#include <WRCoach/entity/player/playeraccess.hh>
#include <WRCoach/entity/locations.hh>
#include <WRCoach/utils/utils.hh>
#include <WRCoach/utils/line/line.hh>
#include <WRCoach/const/globalconstants.hh>

PlayerUtils::PlayerUtils(Player *player, Locations *loc) {
    _player = player;
    _loc = loc;
}

QList<kNN::element> PlayerUtils::getKNN(int k, const Position &origin) {
    // kNN algorithm
    QList<kNN::element> knn = kNN::getKNN(k+1, origin);

    // Exclude self player
    QList<kNN::element>::iterator it;
    for(it=knn.begin(); it!=knn.end(); it++) {
        kNN::element element = *it;
        if(element.id==_player->playerId() && element.team==_player->teamId()) {
            knn.erase(it);
            break;
        }
    }

    // Return list
    return knn;
}

QList<kNN::element> PlayerUtils::getKNN(int k) {
    return this->getKNN(k, _player->position());
}

QList<kNN::element> PlayerUtils::getTeammateKNN(int k, const Position &origin) {
    // kNN algorithm
    QList<kNN::element> knn = this->getKNN(100, origin);

    // Exclude opponent team
    int c=0;
    int ss = knn.size();
    for(int i=0; i<ss; i++) {
        kNN::element element = knn.at(c);
        if(element.team==_player->opTeamId())
            knn.removeAt(c);
        else
            c++;
    }

    // Get 'k'
    QList<kNN::element> retn;
    if(k>=knn.size())
        retn = knn;
    else {
        for(int i=0; i<k; i++)
            retn.push_back(knn.at(i));
    }

    // Return list
    return retn;
}

QList<kNN::element> PlayerUtils::getTeammateKNN(int k) {
    return this->getTeammateKNN(k, _player->position());
}

QList<kNN::element> PlayerUtils::getOpponentKNN(int k, const Position &origin) {
    // kNN algorithm
    QList<kNN::element> knn = this->getKNN(100, origin);

    // Exclude my team
    int c=0;
    int ss = knn.size();
    for(int i=0; i<ss; i++) {
        kNN::element element = knn.at(c);
        if(element.team==_player->teamId())
            knn.removeAt(c);
        else
            c++;
    }

    // Get 'k'
    QList<kNN::element> retn;
    if(k>=knn.size())
        retn = knn;
    else {
        for(int i=0; i<k; i++)
            retn.push_back(knn.at(i));
    }

    // Return list
    return knn;
}

QList<kNN::element> PlayerUtils::getOpponentKNN(int k) {
    return this->getOpponentKNN(k, _player->position());
}

Position PlayerUtils::getClearKickPosition() {
    const Position theirGoalRightPost = _loc->theirGoalRightPost();
    const Position theirGoalLeftPost = _loc->theirGoalLeftPost();
    const Position posTheirGoal = _loc->theirGoal();

    QList<FreeAngles::Interval> freeAngles = FreeAngles::getFreeAngles(_loc->ball(), theirGoalRightPost, theirGoalLeftPost, _player->robotRadius());

	// Get the largest
	float largestAngle=0, largestMid=0;
	if(freeAngles.size()==0) { // Without free angles
        return Position(false, 0.0, 0.0, 0.0);
	} else {
		for(int i=0; i<freeAngles.size(); i++) {
			float angI = freeAngles.at(i).angInitial();
			float angF = freeAngles.at(i).angFinal();

            WR::Utils::angleLimitZeroTwoPi(&angI);
            WR::Utils::angleLimitZeroTwoPi(&angF);
            float dif = angF - angI;
            WR::Utils::angleLimitZeroTwoPi(&dif);
			if(dif>largestAngle) {
				largestAngle = dif;
				largestMid = angF - dif/2;
			}
		}
	}

	// With free angles
	// Triangle
    float x = posTheirGoal.x() - _loc->ball().x();
	float tg = tan(largestMid);
	float y = tg*x;

	// Impact point
    float pos_y = _loc->ball().y() + y;
    Position impactPos(true, posTheirGoal.x(), pos_y, 0.0);

    // Check if impact pos has enough space for the ball
    const float distImpactPos = WR::Utils::distance(_loc->ball(), impactPos);
    const float radiusAngle = largestAngle/2;
    const float distR = radiusAngle*distImpactPos;
    if(distR < 1.5*GlobalConstants::ballRadius())
        return Position(false, 0.0, 0.0, 0.0);

    return impactPos;
}

std::pair<Position,Position> PlayerUtils::getQuadrantPositions(Quadrants::Quadrant quadrant, FieldSide side) {
    Position initialPos, finalPos;

    // Calc some points
    const float x = fabs(_loc->ourGoal().x());
    const float y = fabs(_loc->ourFieldTopCorner().y());
    const Position upL(true, -x, y, 0.0);
    const Position up(true, 0.0, y, 0.0);
    const Position upR(true, x, y, 0.0);
    const Position botL(true, -x, -y, 0.0);
    const Position bot(true, 0.0, -y, 0.0);
    const Position botR(true, x, -y, 0.0);
    const Position cen(true, 0.0, 0.0, 0.0);

    // Set initial position
    if(side.isRight()) {
        if(quadrant&Quadrants::BOT)
            initialPos = bot;
        if(quadrant&Quadrants::BOTMID)
            initialPos = cen;
        if(quadrant&Quadrants::UPMID)
            initialPos = up;
        if(quadrant&Quadrants::UP)
            initialPos = upR;
    } else {
        if(quadrant&Quadrants::UP)
            initialPos = up;
        if(quadrant&Quadrants::UPMID)
            initialPos = cen;
        if(quadrant&Quadrants::BOTMID)
            initialPos = bot;
        if(quadrant&Quadrants::BOT)
            initialPos = botL;
    }

    // Set final position
    if(side.isRight()) {
        if(quadrant&Quadrants::UP)
            finalPos = up;
        if(quadrant&Quadrants::UPMID)
            finalPos = cen;
        if(quadrant&Quadrants::BOTMID)
            finalPos = bot;
        if(quadrant&Quadrants::BOT)
            finalPos = botR;
    } else {
        if(quadrant&Quadrants::BOT)
            finalPos = bot;
        if(quadrant&Quadrants::BOTMID)
            finalPos = cen;
        if(quadrant&Quadrants::UPMID)
            finalPos = up;
        if(quadrant&Quadrants::UP)
            finalPos = upL;
    }

    return std::make_pair(initialPos, finalPos);
}

FreeAngles::Interval PlayerUtils::getGoalAngles(Quadrants::Quadrant quadrant) {

	// Calc pos angles
    std::pair<Position,Position> positions = getQuadrantPositions(quadrant, _loc->theirSide());
    Position initialPos = positions.first;
    Position finalPos = positions.second;

	// Calc angles
    float minPosAngle = WR::Utils::getAngle(_loc->theirGoal(), initialPos);
    float maxPosAngle = WR::Utils::getAngle(_loc->theirGoal(), finalPos);
	return FreeAngles::Interval(minPosAngle, maxPosAngle, false);
}

QList<FreeAngles::Interval> PlayerUtils::getGoalFreeAngles(quint8 quadrant, float radius) {
    const Position posGoal = _loc->theirGoal();

    // Calc pos angles
    std::pair<Position,Position> positions = getQuadrantPositions(quadrant, _loc->theirSide());
    Position initialPos = positions.first;
    Position finalPos = positions.second;

    // Generates obstacle list, removing the calling player
    QList<Obstacle> obstacles = FreeAngles::getObstacles(posGoal, radius);
    for(int i=0; i<obstacles.size(); i++) {
        Obstacle obst = obstacles.at(i);
        if(obst.team()==_player->teamId() && obst.id()==_player->playerId())
            obstacles.removeAt(i);
    }

    // Calc free angles
    return FreeAngles::getFreeAngles(posGoal, initialPos, finalPos, obstacles);
}

Position PlayerUtils::getGoalPosition(Quadrants::Quadrant quadrant, float radius) {
	const Position posTheirGoal = _loc->theirGoal();

	// Get angles in quadrant (1, 2 or 3)
	FreeAngles::Interval goalAngles = getGoalAngles(quadrant);

	// Get the mid angle
	float angI = goalAngles.angInitial();
	float angF = goalAngles.angFinal();
	float dif = WR::Utils::angleDiff(angI, angF);
	float midAngle = angF - dif/2;

	// Get position
	float posAngle = GEARSystem::Angle::pi - midAngle;
	float posX = radius*cos(posAngle);
	float posY = radius*sin(posAngle);

	return Position(true, posTheirGoal.x()-posX, posY, 0.0);
}

Position PlayerUtils::getClearGoalPosition(quint8 quadrant, float radius) {
    const Position posTheirGoal = _loc->theirGoal();

    // Get free angles in quadrant (1, 2 or 3)
    QList<FreeAngles::Interval> goalFreeAngles = getGoalFreeAngles(quadrant, radius+0.2);

    // Get the largest angle
    float largestAngle=0, largestMid=0;
    if(goalFreeAngles.empty()) { // Without free angles
        return Position(false, 0.0, 0.0, 0.0);
    } else {
        for(int i=0; i<goalFreeAngles.size(); i++) {
            float angI = goalFreeAngles.at(i).angInitial();
            float angF = goalFreeAngles.at(i).angFinal();
            float dif = WR::Utils::angleDiff(angI, angF);
            if(dif>largestAngle) {
                largestAngle = dif;
                largestMid = angF - dif/2;
            }
        }
    }

    // Get position
    float posAngle = GEARSystem::Angle::pi - largestMid;
    float posX = radius*cos(posAngle);
    float posY = radius*sin(posAngle);

    return Position(true, posTheirGoal.x()-posX, posY, 0.0);
}

bool PlayerUtils::hasClearPathTo(const Position &pos, const QList<std::pair<quint8,quint8> > &exceptions) {
    // Get angles of destination
    Obstacle destination;
    destination.position() = pos;
    destination.calcAnglesFrom(_player->position(), _player->robotRadius());

    // Generate obstacles and remove exceptions
    float distanceRadius = 1.10*WR::Utils::distance(_player->position(), pos);
    QList<Obstacle> obstacles = FreeAngles::getObstacles(_player->position(), _player->robotRadius(), distanceRadius);

    for(int i=0; i<exceptions.size(); i++) {
        const quint8 exceptionTeam = exceptions.at(i).first;
        const quint8 exceptionId = exceptions.at(i).second;

        for(int j=0; j<obstacles.size(); j++) {
            Obstacle obst = obstacles.at(j);
            const quint8 obstId = obst.id();
            const quint8 obstTeam = obst.team();

            if(obstTeam==exceptionTeam && obstId==exceptionId) {
                obstacles.removeAt(j);
                j--;
            }
        }
    }

    // Calc free angles and check if its clear
    QList<FreeAngles::Interval> freeAngles = FreeAngles::getFreeAngles(destination.initialAngle(), destination.finalAngle(), obstacles);
    if(freeAngles.size() == 1) {
        if(freeAngles.first().angInitial() == destination.initialAngle() && freeAngles.first().angFinal() == destination.finalAngle())
            return true;
    }
    return false;
}

bool PlayerUtils::hasClearPathToTeammate(quint8 id) {
    // Check teammate available
    if(PlayerBus::ourPlayerAvailable(id)==false)
        return false;

    // Get teammate position
    const Position teammatePos = PlayerBus::ourPlayer(id)->position();

    // Create exception list
    QList<std::pair<quint8,quint8> > exceptions;
    exceptions.push_back(std::make_pair(_player->teamId(), id));

    // Return
    return this->hasClearPathTo(teammatePos, exceptions);
}

bool PlayerUtils::hasAnyPathTo(const Position &pos, const QList<quint8> &exceptions) {
    // Get angles of destination
    Obstacle destination;
    destination.position() = pos;
    destination.calcAnglesFrom(_player->position(), _player->robotRadius());

    // Generate obstacles and remove exceptions
    QList<Obstacle> obstacles = FreeAngles::getObstacles(_player->position(), _player->robotRadius());

    for(int i=0; i<exceptions.size(); i++) {
        const quint8 exception = exceptions.at(i);

        for(int j=0; j<obstacles.size(); j++) {
            Obstacle obst = obstacles.at(j);
            if(obst.id() == exception) {
                obstacles.removeAt(j);
                j--;
            }
        }
    }

    // Calc free angles and check if has any free angles
    QList<FreeAngles::Interval> freeAngles = FreeAngles::getFreeAngles(destination.initialAngle(), destination.finalAngle(), obstacles);
    return (freeAngles.empty()==false);
}

Position PlayerUtils::getClearGoalAndAttackerPosition(quint8 quadrant, quint8 attackerID, float minRadius, float maxRadius) {
    const Position posTheirGoal = _loc->theirGoal();
    const Position posAttacker = PlayerBus::ourPlayer(attackerID)->position();
    const float distAttacker = WR::Utils::distance(_player->position(), posAttacker);

    // Radius
    float radius = minRadius + (float)(maxRadius-minRadius)/2;

    // Get free angles in goal
    QList<FreeAngles::Interval> goalFreeAngles = getGoalFreeAngles(quadrant, radius+0.2);
    float largestGoalAngle = 0;
    if(goalFreeAngles.empty()) { // Without free angles
        return Position(false, 0.0, 0.0, 0.0);
    } else {
        float largestAngle=0, largestMid=0;
        for(int i=0; i<goalFreeAngles.size(); i++) {
            float angI = goalFreeAngles.at(i).angInitial();
            float angF = goalFreeAngles.at(i).angFinal();
            float dif = WR::Utils::angleDiff(angI, angF);
            if(dif>largestAngle) {
                largestAngle = dif;
                largestMid = angF - dif/2;
            }
        }
        largestGoalAngle = largestMid;
    }

    // Get position
    float posAngle = GEARSystem::Angle::pi - largestGoalAngle;
    float posX = radius*cos(posAngle);
    float posY = radius*sin(posAngle);
    Position goalLinePos(true, posTheirGoal.x()-posX, posY, 0.0);

    const Position posMinRadius = WR::Utils::threePoints(posTheirGoal, goalLinePos, minRadius, 0.0);
    const Position posMaxRadius = WR::Utils::threePoints(posTheirGoal, goalLinePos, maxRadius, 0.0);

    // Get obstacles from attacker
    QList<Obstacle> atkObstacles = FreeAngles::getObstacles(_loc->ball(), _player->robotRadius(), distAttacker);
    for(int i=0; i<atkObstacles.size(); i++) {
        Obstacle obst = atkObstacles.at(i);
        if(obst.team()==_player->teamId() && obst.id()==_player->playerId()) {
            atkObstacles.removeAt(i);
            i--;
        }
    }

    // Get free angles from attacker
    Position initialPos, finalPos;
    float posMinAngle = WR::Utils::getAngle(_loc->ball(), posMinRadius);
    float posMaxAngle = WR::Utils::getAngle(_loc->ball(), posMaxRadius);
    WR::Utils::angleLimitZeroTwoPi(&posMinAngle);
    WR::Utils::angleLimitZeroTwoPi(&posMaxAngle);
    if(posMaxAngle>posMinAngle) {
        initialPos = posMinRadius;
        finalPos = posMaxRadius;
    } else {
        initialPos = posMaxRadius;
        finalPos = posMinRadius;
    }

    QList<FreeAngles::Interval> atkFreeAngles = FreeAngles::getFreeAngles(_loc->ball(), initialPos, finalPos, atkObstacles);
    float largestAtkAngle=0;
    if(atkFreeAngles.empty()) {
        return goalLinePos;
    } else {
        float largestAngle=0, largestMid=0;
        for(int i=0; i<atkFreeAngles.size(); i++) {
            float angI = atkFreeAngles.at(i).angInitial();
            float angF = atkFreeAngles.at(i).angFinal();
            float dif = WR::Utils::angleDiff(angI, angF);
            if(dif>largestAngle) {
                largestAngle = dif;
                largestMid = angF - dif/2;
            }
        }
        largestAtkAngle = largestMid;
    }

    // Get intercept position
    Line goalLine = Line::getLine(posTheirGoal, largestGoalAngle);
    Line atkLine = Line::getLine(_loc->ball(), largestAtkAngle);
    return goalLine.interceptionWith(atkLine);
}

bool PlayerUtils::isBallComing(float minVelocity, float radius) {
    const Position posBall = _loc->ball();
    const Position posPlayer = _player->position();

    // Check ball velocity
    if(_loc->ballVelocity().abs() < minVelocity)
        return false;

    // Angle player
    float angVel = _loc->ballVelocity().arg().value();
    float angPlayer = WR::Utils::getAngle(posBall, posPlayer);

    // Check angle difference
    float angDiff = WR::Utils::angleDiff(angVel, angPlayer);
    float angError = atan2(radius, _player->distBall());

    return (fabs(angDiff) < fabs(angError));
}

bool PlayerUtils::isBallComingToGoal(float minVelocity, float postsFactor) {
    const Position posBall = _loc->ball();
	const Position posRightPost(true, _loc->ourGoalRightPost().x(), _loc->ourGoalRightPost().y()*postsFactor, 0.0);
	const Position posLeftPost(true, _loc->ourGoalLeftPost().x(), _loc->ourGoalLeftPost().y()*postsFactor, 0.0);

    // Check ball velocity
    if(_loc->ballVelocity().abs()<minVelocity)
        return false;

    // Angle ball velocity
    float angVel = _loc->ballVelocity().arg().value();
    float angRightPost = WR::Utils::getAngle(posBall, posRightPost);
    float angLeftPost = WR::Utils::getAngle(posBall, posLeftPost);
    float angDiffPosts = fabs(WR::Utils::angleDiff(angRightPost, angLeftPost));

    // Check angle difference with posts
    float angDiffRight = fabs(WR::Utils::angleDiff(angVel, angRightPost));
    float angDiffLeft = fabs(WR::Utils::angleDiff(angVel, angLeftPost));

    return (angDiffRight<angDiffPosts && angDiffLeft<angDiffPosts);
}

bool PlayerUtils::isBallAlignedToGoal() {
    const Position posRightPost = _loc->theirGoalRightPost();
    const Position posLeftPost = _loc->theirGoalLeftPost();

	Angle angPlayerBall = _player->angleTo(_loc->ball());
	Angle angRightPost = _player->angleTo(posRightPost);
	Angle angLeftPost = _player->angleTo(posLeftPost);

	float angDiffPosts = WR::Utils::angleDiff(angRightPost, angLeftPost);

	// Check angle difference with posts
	float angDiffRight = WR::Utils::angleDiff(angPlayerBall, angRightPost);
	float angDiffLeft = WR::Utils::angleDiff(angPlayerBall, angLeftPost);
	return (fabs(angDiffRight)<angDiffPosts && fabs(angDiffLeft)<angDiffPosts);
}

Position PlayerUtils::getBallInterception() {
    const Velocity ballVel = _loc->ballVelocity();

    // Check ball near-zero velocity
    if(ballVel.abs()<=_player->lError())
        return _loc->ball();

    // Calc ball unitary velocity vector
    const Position posBall = _loc->ball();
    const Position ballVelUni(true, ballVel.x()/ballVel.abs(), ballVel.y()/ballVel.abs(), 0.0);

    // Calc ball velocity line (by taking two positions)
    const Position s1 = posBall;
    const Position s2 = Position(true, posBall.x()+ballVelUni.x(), posBall.y()+ballVelUni.y(), 0.0);

    // Return player nearest position to ball velocity line
    return WR::Utils::projectPointAtLine(s1, s2, _player->position());
}

Position PlayerUtils::getBallImpactPosition(const Angle &incomingAngle) {
    return getBallImpactPosition(incomingAngle, _loc->ourGoal().x());
}

Position PlayerUtils::getBallImpactPosition(const Angle &incomingAngle, float posX) {
    // Get incoming angle value
    float angle = incomingAngle.value();

    // Side inversion
    if(_loc->ourSide().isLeft())
        angle = GEARSystem::Angle::pi - angle;

    // Check 2pi
    if(angle==GEARSystem::Angle::twoPi)
        angle = 0.0f;

    // Check high angle
    if(fabs(angle)>=GEARSystem::Angle::pi)
        angle += (angle>=0? -GEARSystem::Angle::twoPi : GEARSystem::Angle::twoPi);

    // Check impossible impact (infinite tg)
    if(fabs(angle)>=GEARSystem::Angle::pi/2.0f)
        return Position(false, 0.0, 0.0, 0.0);

    // Triagulate
    float x = fabs(posX - _loc->ball().x());
    float tg = tan(angle);
    float y = tg*x;

    // Impact point
    float impact_y = _loc->ball().y() + y;
    const Position posImpact(true, posX, impact_y, 0.0);

    return posImpact;
}

bool PlayerUtils::isBehindBall(const Position &destination) {
    const Position posBall = _loc->ball();
    const Position posPlayer = _player->position();

    float anglePlayer = WR::Utils::getAngle(posBall, posPlayer);
    float angleDest = WR::Utils::getAngle(posBall, destination);
    float diff = WR::Utils::angleDiff(anglePlayer, angleDest);
    return (diff>GEARSystem::Angle::pi/2.0f);
}

/// [TODO] Usar um valor para o ângulo que dependa da competição?
bool PlayerUtils::isBallInFront() {
	Angle anglePlayerBall = _player->angleTo(_loc->ball());
	float diff = WR::Utils::angleDiff(anglePlayerBall, _player->orientation());
    return (diff <= atan(0.7)); // atan(0.7) aprox = 35 degree
}
