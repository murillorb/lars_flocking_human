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

#ifndef UVF_HH
#define UVF_HH

#include <WRCoach/entity/player/navigation/navigationalgorithm/navigationalgorithm.hh>
#include "uvfobstacle.hh"

class UVF : public NavigationAlgorithm {
public:
    UVF();
	~UVF();

	void reset();

	// Add obstacles
	void addBall(const Position &pos, const Velocity &vel);
	void addGoalArea(const Position &pos);
	void addOwnRobot(const Position &pos, const Velocity &vel);
	void addEnemyRobot(const Position &pos, const Velocity &vel);

	// Return results
	Angle getDirection() const;
	float getDistance() const;
private:
    // Inheritance methods
    UVF* clone() const;

	// Run
	void run();

	// Obstacle avoidance
	void addObstacle(const Position &pos, const Velocity &vel);
	QList<UVFObstacle> _obstacles;

	// Auxiliar functions
	double getPhi(double pX, double pY, bool ccw) const;
	double getTheta(double pX, double pY) const;
	double gauss(double r) const;

	// Results
	Angle _resultantDirection;
};


#endif // UVF_HH

