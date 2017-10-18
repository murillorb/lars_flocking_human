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

#include "opfconstants.hh"

bool OPFConstants::_competitionSet = false;

// Competition based values
float OPFConstants::_gridWidth = 0.0;
float OPFConstants::_gridHeight = 0.0;
float OPFConstants::_gridWidthRes = 0.0;
float OPFConstants::_gridHeightRes = 0.0;
float OPFConstants::_maxError = 0.0;
float OPFConstants::_maxIterations = 0.0;
float OPFConstants::_epsilon = 0.0;

void OPFConstants::setCompetition(Competitions::Competition competition) {
    using namespace Competitions;

    // Avoid setting competition two times
    if(OPFConstants::_competitionSet==true)
        return;

    // Set constants based on competition
    switch(competition) {
        default:
        case SSL:
            OPFConstants::_gridWidth = 8.50;
            OPFConstants::_gridHeight = 6.50;
            OPFConstants::_gridWidthRes = 0.10f;
            OPFConstants::_gridHeightRes = 0.10f;
            OPFConstants::_maxError = 1e-4;
            OPFConstants::_maxIterations = 25e3;
            OPFConstants::_epsilon = 0.0;
        break;
        case VSS:
            OPFConstants::_gridWidth = 1.55;
            OPFConstants::_gridHeight = 1.35;
            OPFConstants::_gridWidthRes = 0.02f;
            OPFConstants::_gridHeightRes = 0.02f;
            OPFConstants::_maxError = 1e-4;
            OPFConstants::_maxIterations = 1e3;
            OPFConstants::_epsilon = 0.2;
        break;
    }

    OPFConstants::_competitionSet = true;
}
