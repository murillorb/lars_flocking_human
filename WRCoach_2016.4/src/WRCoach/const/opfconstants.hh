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

#ifndef OPFCONSTANTS_H_
#define OPFCONSTANTS_H_

#include <WRCoach/utils/basics/competition.hh>

class OPFConstants {
private:
    static float _gridWidth;
    static float _gridHeight;
    static float _gridWidthRes;
    static float _gridHeightRes;
    static float _maxError;
    static float _maxIterations;
    static float _epsilon;

    static bool _competitionSet;
public:
    static void setCompetition(Competitions::Competition competition);

    static float gridWidth() { return _gridWidth; }
    static float gridHeight() { return _gridHeight; }
    static float gridWidthRes() { return _gridWidthRes; }
    static float gridHeightRes() { return _gridHeightRes; }
    static float maxError() { return _maxError; }
    static float maxIterations() { return _maxIterations; }
    static float epsilon() { return _epsilon; }
};

#endif // OPFCONSTANTS_H_
