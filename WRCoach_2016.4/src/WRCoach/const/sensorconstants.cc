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

#include "sensorconstants.hh"

bool SensorConstants::_competitionSet = false;

// Competition based values
float SensorConstants::_maxDistPossession = 0.0;
float SensorConstants::_distToConsiderBallMovement = 0.0;

void SensorConstants::setCompetition(Competitions::Competition competition) {
    using namespace Competitions;

    // Avoid setting competition two times
    if(SensorConstants::_competitionSet==true)
        return;

    // Set constants based on competition
    switch(competition) {
        default:
        case SSL:
            SensorConstants::_maxDistPossession = 0.60;
            SensorConstants::_distToConsiderBallMovement = 0.15;
        break;
        case VSS:
            SensorConstants::_maxDistPossession = 0.3;
            SensorConstants::_distToConsiderBallMovement = 0.10;
        break;
    }

    SensorConstants::_competitionSet = true;
}
