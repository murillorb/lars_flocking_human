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

#ifndef PLAYERUTILS_HH
#define PLAYERUTILS_HH

#include <WRCoach/entity/player/baseplayer.hh>
#include <WRCoach/utils/knn/knn.hh>
#include <WRCoach/utils/freeangles/freeangles.hh>
#include <WRCoach/utils/basics/quadrant.hh>
#include <WRCoach/utils/fieldside/fieldside.hh>

class PlayerUtils {
public:
    PlayerUtils(Player *player, Locations *loc);

    // kNN
    QList<kNN::element> getKNN(int k);
    QList<kNN::element> getKNN(int k, const Position &origin);
    QList<kNN::element> getTeammateKNN(int k);
    QList<kNN::element> getTeammateKNN(int k, const Position &origin);
    QList<kNN::element> getOpponentKNN(int k);
    QList<kNN::element> getOpponentKNN(int k, const Position &origin);

    // Free angles
	Position getClearKickPosition();

    FreeAngles::Interval getGoalAngles(Quadrants::Quadrant quadrant);
    Position getGoalPosition(Quadrants::Quadrant quadrant, float radius);

    QList<FreeAngles::Interval> getGoalFreeAngles(quint8 quadrant, float radius = 999);
    Position getClearGoalPosition(quint8 quadrant, float radius);
    Position getClearGoalAndAttackerPosition(quint8 quadrant, quint8 attackerID, float minRadius, float maxRadius);

    bool hasClearPathTo(const Position &pos, const QList<std::pair<quint8, quint8> > &exceptions = QList<std::pair<quint8,quint8> >());
    bool hasClearPathToTeammate(quint8 id);
    bool hasAnyPathTo(const Position &pos, const QList<quint8> &exceptions = QList<quint8>());

    bool isBallComing(float minVelocity, float radius);
	bool isBallComingToGoal(float minVelocity, float postsFactor = 1.0);
    Position getBallInterception();

    Position getBallImpactPosition(const Angle &incomingAngle);
    Position getBallImpactPosition(const Angle &incomingAngle, float posX);

    bool isBehindBall(const Position &destination);

    bool isBallInFront();

    bool isBallAlignedToGoal();
private:
    Player *_player;
    Locations *_loc;

    std::pair<Position,Position> getQuadrantPositions(Quadrants::Quadrant quadrant, FieldSide side);
};

#endif // PLAYERUTILS_HH
