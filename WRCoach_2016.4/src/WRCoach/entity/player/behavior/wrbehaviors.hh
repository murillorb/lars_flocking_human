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

#ifndef WRBEHAVIORS_HH
#define WRBEHAVIORS_HH

/// IA 2015
// Basics
#include "basics/behavior_donothing.hh"
#include "basics/behavior_followball.hh"
#include "basics/behavior_pos.hh"

// Slacs
#include "slacs/behavior_followentity.hh"
#include "slacs/behavior_goto.hh"

// SSL
#include "ssl/behavior_attack.hh"
#include "ssl/behavior_gk.hh"
#include "ssl/behavior_gka.hh"
#include "ssl/behavior_markball.hh"
#include "ssl/behavior_markballa.hh"
#include "ssl/behavior_markplayer.hh"
#include "ssl/behavior_penaltyassistant.hh"
#include "ssl/behavior_penaltygk.hh"
#include "ssl/behavior_penaltykick.hh"
#include "ssl/behavior_recv.hh"

// VSS
#include "vss/vssbehavior_attack.hh"
#include "vss/vssbehavior_mark.hh"
#include "vss/vssbehavior_defense.hh"
#include "vss/vssbehavior_gk.hh"
#include "vss/vssbehavior_attacka.hh"
#include "vss/vssbehavior_gka.hh"
#include "vss/vssbehavior_freeball.hh"

// Test
#include "test/behavior_followplayer.hh"

#endif // WRBEHAVIORS_HH
