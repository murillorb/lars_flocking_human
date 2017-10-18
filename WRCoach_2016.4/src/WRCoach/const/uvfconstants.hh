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

#ifndef UVFCONSTANTS_HH
#define UVFCONSTANTS_HH

class UVFConstants {
private:
	// Hyperbolic spiral univector field
	static double _de;
	static double _kr;

	// Avoid-obstacle univector field
	static double _k0;

	// Composition of move-to-goal univector field and avoid-obstacle univector field
	static double _dmin;
	static double _delta;
public:
	static double de() { return UVFConstants::_de; }
	static double kr() { return UVFConstants::_kr; }

	static double k0() { return UVFConstants::_k0; }

	static double dmin() { return UVFConstants::_dmin; }
	static double delta() { return UVFConstants::_delta; }
};

#endif // UVFCONSTANTS_HH
