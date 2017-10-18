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

#ifndef PLAYERBUS_HH
#define PLAYERBUS_HH

#include "baseplayer.hh"

class PlayerBus {
private:
    static WRTeam *_ourTeam;
    static WRTeam *_opTeam;
public:
    static void initialize(WRTeam *ourTeam, WRTeam *opTeam);

    static bool ourPlayerAvailable(quint8 id);
    static bool theirPlayerAvailable(quint8 id);
    static PlayerAccess* ourPlayer(quint8 id);
    static PlayerAccess* theirPlayer(quint8 id);
};

#endif // PLAYERBUS_HH
