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

#include "pfconstants.hh"

bool PFConstants::_competitionSet = false;

// Competition based values
float PFConstants::_constBallRepulsive = 0.0;
float PFConstants::_constRepulsive = 0.0;
float PFConstants::_constAreaRepulsive = 0.0;
float PFConstants::_collisionCheckingRadius = 0.0;
float PFConstants::_robotCollisionRadius = 0.0;
float PFConstants::_ballCollisionRadius = 0.0;
bool PFConstants::_collisionCheckingEnabled = false;
float PFConstants::_functionXshift = 0.0;
float PFConstants::_functionYshift = 0.0;
float PFConstants::_functionFactor = 0.0;

void PFConstants::setCompetition(Competitions::Competition competition) {
    using namespace Competitions;

    // Avoid setting competition two times
    if(PFConstants::_competitionSet==true)
        return;

    // Set constants based on competition
    switch(competition) {
        default:
        case SSL:
//            PFConstants::_constBallRepulsive = 0.18;
            PFConstants::_constBallRepulsive = 0.45;
            PFConstants::_constRepulsive = 0.11;
            PFConstants::_constAreaRepulsive = 0.15;
            PFConstants::_collisionCheckingRadius = 1.5;
            PFConstants::_robotCollisionRadius = 0.40;
            PFConstants::_ballCollisionRadius = 0.16;
            PFConstants::_collisionCheckingEnabled = true;
            PFConstants::_functionXshift = -0.5;
            PFConstants::_functionYshift = 0.0;
            PFConstants::_functionFactor = 1.0;
        break;
        case VSS:
            PFConstants::_constBallRepulsive = 0.2;
            PFConstants::_constRepulsive = 0.010;
			PFConstants::_constAreaRepulsive = 0.3;
            PFConstants::_collisionCheckingRadius = 1.5;
            PFConstants::_robotCollisionRadius = 0.10;
            PFConstants::_ballCollisionRadius = 0.25;
            PFConstants::_collisionCheckingEnabled = true;
            PFConstants::_functionXshift = -0.5;
            PFConstants::_functionYshift = 0.0;
            PFConstants::_functionFactor = 1.0;
        break;
    }

    PFConstants::_competitionSet = true;
}
