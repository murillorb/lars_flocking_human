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

#include "rrt.hh"
#include "rrtvertex.hh"
#include <WRCoach/const/globalconstants.hh>
#include <WRCoach/utils/graph/graph.hh>

#pragma GCC diagnostic ignored "-Wunused-parameter"

#define ROBOT_RADIUS 0.12

RRT::RRT() {
    srand (time(NULL));

    _minDistToGoal = 0.3;
    _smoothPathResolution = 0.1;
    _smoothPathMaxStep = 1.0;
    _smoothStep = 0.5;
    _vectorMaxLength = 0.3;
    _maxIterations = 3000;
    _maxSmoothingInt = 3;
    _fieldError = 0.2;
    _error = 0.0;
    _discResolution = 0.04;
}

RRT::~RRT() {

}

// Reset algorithm
void RRT::reset() {
    _obsList.clear();

    while(_tree.empty()==false)
        delete _tree.takeFirst();
}

// Add objects
void RRT::addBall(const Position &pos, const Velocity &vel) {
    addObstacles(pos, GlobalConstants::ballRadius() );
}

void RRT::addGoalArea(const Position &pos) {

}

void RRT::addOwnRobot(const Position &pos, const Velocity &vel) {
    addObstacles(pos, ROBOT_RADIUS);
}

void RRT::addEnemyRobot(const Position &pos, const Velocity &vel) {
    addObstacles(pos, ROBOT_RADIUS);
}

void RRT::run() {

    /// TODO: check smooth function
    /// TODO: check fix when goalPos() is inside obstacle

    _pathMutex.lock();

    if(_path.isEmpty()) {
        // Find RRT path
        copyToFrom(&_path,findPath(originPos(), goalPos()));

        // Minimize
        minimizePath();

        // Smooth path
//        smoothPath();
    } else {
        // If the current path has been blocked, find a new path, else try to minimize and smooth the current path
        if(isPathBlocked()) {
            // Find RRT path
            appendPaths(&_path, findPath(_breakPoint, goalPos()));

            // Update current and goal position on list
            updatePositions();

            // Minimize
            minimizePath();

            // Smooth path
//            smoothPath();
        } else {
//            std::cout << "Path OK, following (just minimizing)...\n";

            // Update current and goal position on list
            updatePositions();

            int last = _path.size() - 1;
            Position nextPos = _path.at(last-1);
            float dist = WR::Utils::distance(originPos(), nextPos);

            // Minimize
            if(dist > 0.40) {
                minimizePath();
//                smoothPath();
            }

            // Remove points that were already used
            if(dist < 0.20) {
               if(_path.size() > 2) {
                   _path.removeAt(last-1);
               }
            }

        }
    }

    _pathMutex.unlock();

    // Return resultant angle
    if(_path.size()>=2) {
        int last = _path.size()-1;
        _resultantAngle = WR::Utils::getAngle(_path.at(last), _path.at(last-1));
    } else {
        std::cout << "[ERROR] RRT::run(), generated path has only one point!\n";
        _resultantAngle = WR::Utils::getAngle(originPos(), goalPos());
    }
}

// Internal function for add obstacles in the obstacles list
void RRT::addObstacles(const Position &pos, float radius) {
    // If obstacle isn't inside origin area, append to obstacle list
    if(WR::Utils::distance(originPos(), pos) > radius + ROBOT_RADIUS)
        _obsList.append(RRTObstacle(pos, radius));
}

// Create a randomic point
Position RRT::randPoint() {
    // Generate two diferente numbers between 0.0 and 1.0
    float xSeed = (float) rand()/RAND_MAX;
    float ySeed = (float) rand()/RAND_MAX;

    // Generate a randomic x and y inside the field
    float xRand = ( xSeed*loc()->fieldLength() ) - (loc()->fieldLength()/2);
    float yRand = ( ySeed*loc()->fieldWidth() ) - (loc()->fieldWidth()/2);

    return Position(true, xRand, yRand, 0.0);
}

// Finds the nearest point in the tree to the randomic point
RRTPoint* RRT::nearestPoint(const Position &randPoint) {
    float minorDist = 100;
    int minorIndex = 0;

    // Looking for the minor distance
    for(int i = 0; i < _tree.size(); i++) {
        RRTPoint *point = _tree.at(i);

        float dist = RRT::simpleDistance(point->getPos(), randPoint);
        if (dist < minorDist) {
            minorDist = dist;
            minorIndex = i;
        }
    }

    // Return the nearest point
    return _tree.at(minorIndex);
}

// Allocates a new RRTPoint
void RRT::addRRTPoint(const Position &posNew, RRTPoint pointNear, QList<RRTPoint> &RRTPath) {
    RRTPath.append(RRTPoint(posNew, &pointNear));
}

// Checks if two point can connect
bool RRT::canConnect(const Position &posA, const Position &posB) {
    for(int i = 0; i < _obsList.size(); i++) {
        RRTObstacle obst = _obsList.at(i);
        float obstRadius = obst.getRadius() + ROBOT_RADIUS;
        float dist = WR::Utils::distanceToSegment(posA, posB, obst.getPos());
        if(dist < obstRadius)
            return false;
    }

    return true;
}

// Gets a vector in the direction and orientation of two points
Position RRT::getVector(const Position &near, const Position &rand, float vectorLength) {
    Position vec(true, rand.x()-near.x(), rand.y()-near.y(), 0.0);
    float theta = atan2(vec.y(), vec.x());

    return Position(true, vectorLength*cos(theta), vectorLength*sin(theta), 0.0);
}

// Minimizes number of points of the list
void RRT::minimizePath() {
    // Generate Graph to minimize path
    Graph graph;

    QList<RRTVertex*> vertices;

    // Generate vertices
    RRTVertex *origin=NULL;
    RRTVertex *goal=NULL;

    for(int i=0; i<_path.size(); i++) {
        // Create
        RRTVertex *vertex = new RRTVertex(_path.at(i));
        vertices.append(vertex);

        // Add to graph
        graph.addVertex(vertex);

        // Create original path connection
        if(i>0) {
            RRTVertex *last = vertices.at(i-1);
            graph.connect(vertex, last, WR::Utils::distance(vertex->getPosition(), last->getPosition()));
        }

        // Store origin and goal
        if(i==0)
            goal = vertex;
        if(i==_path.size()-1)
            origin = vertex;
    }

    // Generate edges
    for(int i=0; i<vertices.size(); i++) {
        RRTVertex *v1 = vertices.at(i);
        Position pos1 = v1->getPosition();

        for(int j=0; j<vertices.size(); j++) {
            RRTVertex *v2 = vertices.at(j);
            Position pos2 = v2->getPosition();

            // Create
            if(canConnect(pos1, pos2)) {
                float weight = WR::Utils::distance(pos1, pos2);
                graph.connect(v1, v2, weight);
            }
        }
    }

    // Get shortest path
    QLinkedList<Vertex*> shortestPath = graph.getShortestPath(goal, origin);

    // Clear old path
    _path.clear();

    // Append minimized path
    QLinkedList<Vertex*>::iterator it;
    for(it=shortestPath.begin(); it!=shortestPath.end(); it++) {
        RRTVertex *vertex = static_cast<RRTVertex*>(*it);
        _path.append(vertex->getPosition());
    }

}

// Smooths path by add new points at list
void RRT::smoothPath() {
    QList<Position> smoothPath;

    // Aply the smoothing algorithm maxSmoothingInt times
    for(int j=0;j<_maxSmoothingInt;j++) {
        // Append first position to smoothPath
        smoothPath.append(_path.first());

        // Go through all point except the extremes
        for(int i=1; i<_path.size()-1; i++) {
            float distA = WR::Utils::distance(_path.at(i), _path.at(i-1));
            float distB = WR::Utils::distance(_path.at(i), _path.at(i+1));

            // If the segments are too small, keep them
            if(distA <= _smoothPathResolution || distB <= _smoothPathResolution) {
                smoothPath.append(_path.at(i));
                continue;
            }

            Position posTmp;

            // If the segment is too long, limit by smoothStep, else limit by half of distance
            if (distA > _smoothPathMaxStep) {
                posTmp = getVector(_path.at(i), _path.at(i-1), _smoothStep);
            } else {
                posTmp = getVector(_path.at(i), _path.at(i-1), distA/2);
            }

            // Create a point between the point i and i-1, and append to smPath
            posTmp.setPosition(_path.at(i).x() + posTmp.x(), _path.at(i).y() + posTmp.y(), 0.0);
            smoothPath.append(posTmp);

            // If the segment is too long, limit by smoothStep, else limit by half of distance
            if(distB > _smoothPathMaxStep) {
                posTmp = getVector(_path.at(i), _path.at(i+1), _smoothStep);
            } else {
                posTmp = getVector(_path.at(i), _path.at(i+1), distB/2);
            }

            // Create a point between the point i and i+1, and append to smPath
            posTmp.setPosition(_path.at(i).x() + posTmp.x(), _path.at(i).y() + posTmp.y(), 0.0);
            smoothPath.append(posTmp);
        }

        // Append the last point to tmp
        smoothPath.append(_path.last());

        // Remove all points of path
        _path.clear();

        // Copy smPath to Path
        for(int i=0;i<smoothPath.length();i++) {
            _path.append(smoothPath.at(i));
        }

        // Clear smPath
        smoothPath.clear();
    }
}

QList<Position> RRT::findPath(Position origin, Position goal) {
    // Check trivial case
    if(canConnect(origin, goal)) {

        // Generate path
        QList<Position> path;
        path.append(goal);
        path.append(origin);

        return path;
    }

    // Initialize matrix of discrete points created
    int _xMaxIndex = (int) (loc()->fieldLength() / _discResolution) + 1;
    int _yMaxIndex = (int) (loc()->fieldWidth()  / _discResolution) + 1;

    // Reset grid
    _grid.resize(_xMaxIndex);
    for(int i=0; i<_grid.size(); i++) {
        _grid[i].fill(false, _yMaxIndex);
    }

    // Append the origin to the tree
    _tree.append(new RRTPoint(origin));

    // Creates random points until get max intaration or founds a path through plan to goal
    for(int i=0; i<_maxIterations; i++) {
        // Generate a discrete rand point
        Position randPosition = randPoint(); //discretize(randPoint());

        // Convert randPosition to index
        int xIndex = (randPosition.x()/_discResolution) + (_xMaxIndex)/2;
        int yIndex = (randPosition.y()/_discResolution) + (_yMaxIndex)/2;

        // Checks if randPosition was arealdy taken
        if(_grid[xIndex][yIndex]) {
            continue;
        } else {
            _grid[xIndex][yIndex] = true;
        }

        // Get nearest point
        RRTPoint *nearPoint = nearestPoint(randPosition);

        // Find a vector Near-Rand
        Position unitVec = getVector(nearPoint->getPos(), randPosition, _vectorMaxLength);

        // Sum vector to the near point to found the new point
        Position newPos(true, nearPoint->getPos().x() + unitVec.x(), nearPoint->getPos().y() + unitVec.y(), 0.0);

        // Check if can connect to new point
        if(canConnect(nearPoint->getPos(), newPos)) {

            // Append new point to the tree
            _tree.append(new RRTPoint(newPos, nearPoint));

            // Check if found a path
            float dist = WR::Utils::distance(_tree.last()->getPos(), goal);
            if(dist <= _minDistToGoal)
                break;
        }

    }

    // Initialize path
    QList<Position> path;

    // Append goal
    path.append(goal);

    // Copy the generated path from tree to path list
    RRTPoint *tmpPoint = _tree.last();
    while(tmpPoint != NULL) {
        path.append(tmpPoint->getPos());
        tmpPoint = tmpPoint->getPreviousPoint();
    }

    return path;
}

// Check if path has been blocked
bool RRT::isPathBlocked() {
    for(int i=1;i < _path.size(); i++) {
        // If you found a obstacle in the path, copy the last point before the obstacle and
        // remove all point starting from obstacle
        if(canConnect(_path.at(i-1), _path.at(i))==false) {
            _breakPoint = _path.at(i);
            removeSubPath(&_path);
            return true;
        }
    }
    return false;
}

// Update current and goal position on list
void RRT::updatePositions() {
    // Update current position on list
    _path.removeLast();
    _path.push_back(originPos());

    // Update goal position on list
    _path.removeFirst();
    _path.push_front(goalPos());

//    // Initialize shifted
//    Position shiftedGoal = goalPos();

//    bool goalIsInsideObstacle = true;
//    for(int i = _obsList.size()-1; i >= 0 && goalIsInsideObstacle; i--) {
//        // Reset
//        goalIsInsideObstacle = false;

//        // Check for all obstacles on list
//        for(int i = _obsList.size()-1; i >= 0; i--) {
//            RRTObstacle obstacle = _obsList.at(i);

//            // If goal is too close to origin, remove obstacle
//            if(WR::Utils::distance(shiftedGoal, originPos()) < ROBOT_RADIUS) {
//                _obsList.removeAt(i);
//                continue;
//            }

//            // If distance between goal and obstacle is less than both radius, then goal is inside obstacle area
//            float distGoalToObstacle = WR::Utils::distance(shiftedGoal, obstacle.getPos());
//            float obstacleRadius = obstacle.getRadius() + ROBOT_RADIUS;
//            if(distGoalToObstacle < obstacleRadius) {
//                // Shift goal in origin direction
////                Position vector = getVec(shiftedGoal, obstacle.getPos(), obstacleRadius);
////                vector.setPosition(shiftedGoal.x()+vector.x(), shiftedGoal.y()+vector.y(), 0.0);
////                shiftedGoal = vector;
//                shiftedGoal = WR::Utils::threePoints(shiftedGoal, originPos(), obstacleRadius, 0.0);

//                std::cout << "Shifted!!!\n";

//                // Set to true for a new loop
//                goalIsInsideObstacle = true;
//                break;
//            }
//        }
//    }

//    // Update goal on list
//    _path.removeFirst();
    //    _path.push_front(shiftedGoal);
}

// Copy all elements from oldPath to newPath after cleaning newPath
void RRT::copyToFrom(QList<Position>* newPath, QList<Position> oldPath) {
    if(!(newPath->isEmpty())) {
        newPath->clear();
    }

    for(int i=0; i < oldPath.size(); i++) {
        newPath->append(oldPath[i]);
    }

//    std::cout << "\n\nOh shit dat old path size: " << oldPath.size() << "\n";
//    std::cout << "Oh shit dat new path size: " << newPath->size() << "\n\n\n";
}

// Append all elements of secondPath in firstPath
void RRT::appendPaths(QList<Position> *firstPath, QList<Position> secondPath) {
    // While there is elements in secondPath, move to firstPath
    while(!secondPath.isEmpty()) {
        firstPath->prepend(secondPath.takeLast());
    }
}

// Remove all points that can not be used anymore
void RRT::removeSubPath(QList<Position> *path) {
    // Removes all points between goal and breakPoint from path
    while( !(path->first().x() == _breakPoint.x() && path->first().y() == _breakPoint.y()) ) {
        path->takeFirst();
    }
    // Removes breakPoint from path
    path->takeFirst();
}

float RRT::simpleDistance(const Position &pos1, const Position &pos2) {
    return pow(pos2.x()-pos1.x(), 2) + pow(pos2.y()-pos1.y(), 2);
}

// Return results
Angle RRT::getDirection() const {
    return Angle(true, _resultantAngle);
}

QLinkedList<Position> RRT::getPath() const {
    QLinkedList<Position> path;
    _pathMutex.lock();
    for(int i=0; i<_path.size(); i++)
        path.append(_path.at(i));
    _pathMutex.unlock();
    return path;
}

float RRT::getDistance() const {
    float dist = 0;
    for(int i=0;i<_path.size()-1;i++) { dist += WR::Utils::distance(_path.at(i),_path.at(i+1)); }
    return dist;
}
