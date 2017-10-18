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

#include "uvfobstacle.hh"
#include <WRCoach/const/uvfconstants.hh>
#include <WRCoach/utils/utils.hh>

UVFObstacle::UVFObstacle(const Position &pos, const Velocity &vel) {
	_pos = pos;
	_vel = vel;
}

Position UVFObstacle::getObstaclePos() const {
	return _pos;
}

Position UVFObstacle::getVirtualObstaclePos(const Position &posRobot, const Velocity &velRobot) const {
	double k0 = UVFConstants::k0();

	// Calculate s vector
	float sx = k0*(_vel.x()-velRobot.x());
	float sy = k0*(_vel.y()-velRobot.y());
	Velocity s(true, sx, sy);

	// Calculate virtual obstacle position
	float dist = WR::Utils::distance(posRobot, _pos);
	float vox, voy;
	if(dist < s.abs()) {
		vox = _pos.x() + ((dist/s.abs())*s.x());
		voy = _pos.y() + ((dist/s.abs())*s.y());
	} else {
		vox = _pos.x() + s.x();
		voy = _pos.y() + s.y();
	}

	// Returns virtual obstacle position
	return Position(true, vox, voy, 0.0);
}

