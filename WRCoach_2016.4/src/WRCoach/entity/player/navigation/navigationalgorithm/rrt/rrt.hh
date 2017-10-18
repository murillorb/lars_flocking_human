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

#ifndef RRT_HH
#define RRT_HH

#include <WRCoach/entity/player/navigation/navigationalgorithm/navigationalgorithm.hh>
#include <WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtobstacle.hh>
#include <WRCoach/entity/player/navigation/navigationalgorithm/rrt/rrtpoint.hh>

class RRT : public NavigationAlgorithm {
public:
    RRT();
    ~RRT();

    // Reset algorithm
    void reset();

    // Add obstacles
    void addBall(const Position &pos, const Velocity &vel);
    void addGoalArea(const Position &pos);
    void addOwnRobot(const Position &pos, const Velocity &vel);
    void addEnemyRobot(const Position &pos, const Velocity &vel);

    // Run
    void run();

    // Return results
    Angle getDirection() const;
    QLinkedList<Position> getPath() const;
    float getDistance() const;

private:
    // Calibration variables
    int   _maxIterations;
    int   _maxSmoothingInt;
    float _smoothStep;
    float _minDistToGoal;
    float _smoothPathResolution;
    float _smoothPathMaxStep;
    float _vectorMaxLength;
    float _fieldError;
    float _error;
    float _discResolution;
    QVector<QVector<bool> > _grid;

    // List of the path points and obstacles
    QList<RRTPoint*> _tree;
    QList<Position> _path;
    QList<RRTObstacle> _obsList;
    mutable QMutex _pathMutex;

    // Point where the path was break
    Position _breakPoint;

    // Resultant angle
    float _resultantAngle;

    // Create a randomic point
    Position randPoint();

    // Finds the nearest point in the tree to randomic point
    RRTPoint* nearestPoint(const Position &randPoint);

    // Allocates a new RRTPoint
    void addRRTPoint(const Position &posNew, RRTPoint pointNear, QList<RRTPoint> &RRTPath);

    // Allocates a new obstacle
    RRTObstacle* createObstacles(const Position &pos, float radius);

    // Internal addObstacles
    void addObstacles(const Position &pos, float radius);

    // Checks if two point can connect
    bool canConnect(const Position &posA, const Position &posB);

    // Gets a unit vector in the direction and orientation of two points
    Position getVector(const Position &near, const Position &rand, float vectorLength);

    // Minimizes number of points of the list
    void minimizePath();

    // Smooths path by add new points at list
    void smoothPath();

    // Finds a path
    QList<Position> findPath(Position origin, Position goal);

    // Check if path has been blocked
    bool isPathBlocked();

    // Update current and goal position on list
    void updatePositions();

    // Simple distance (without sqrt)
    static float simpleDistance(const Position &pos1, const Position &pos2);

    // Copy, append and remove methods for path
    void copyToFrom(QList<Position>* newPath, QList<Position> oldPath);
    void appendPaths(QList<Position>* firstPath, QList<Position> secondPath);
    void removeSubPath(QList<Position>* path);
};

#endif // RRT_H
