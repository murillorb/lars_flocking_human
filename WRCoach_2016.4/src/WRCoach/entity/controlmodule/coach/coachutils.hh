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

#ifndef COACHUTILS_HH
#define COACHUTILS_HH

#include <WRCoach/utils/knn/knn.hh>

class CoachUtils {
private:
    WRTeam *_team;
public:
    CoachUtils(WRTeam *team);

    // kNN
    QList<kNN::element> getKNN(int k, const Position &origin);
	QList<kNN::element> getAllKNN(const Position &origin);
    QList<kNN::element> getTeammateKNN(int k, const Position &origin);
    QList<kNN::element> getOpponentKNN(int k, const Position &origin);
};

#endif // COACHUTILS_HH
