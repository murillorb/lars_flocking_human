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

#ifndef WRPLAYBOOK_H_
#define WRPLAYBOOK_H_

// Basics
#include "basics/playbook_donothing.hh"
#include "basics/playbook_followball.hh"
#include "basics/playbook_pos.hh"

// Slacs
#include "slacs/playbook_slacs.hh"

// Flock
#include "flock/playbook_flock.hh"


// SSL
#include "ssl/playbook_attack.hh"
#include "ssl/playbook_defense.hh"
#include "ssl/playbook_goal.hh"
#include "ssl/playbook_ourkickoff.hh"
#include "ssl/playbook_ourpenalty.hh"
#include "ssl/playbook_theirpenalty.hh"

// VSS
#include "vss/vssplaybook_attack.hh"
#include "vss/vssplaybook_defense.hh"
#include "vss/vssplaybook_goal.hh"
#include "vss/vssplaybook_attacka.hh"
#include "vss/vssplaybook_freeball.hh"

#endif // WRPLAYBOOK_H_
