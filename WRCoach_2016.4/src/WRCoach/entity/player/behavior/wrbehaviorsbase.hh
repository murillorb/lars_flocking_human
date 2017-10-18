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

#ifndef WRBEHAVIORSBASE_HH
#define WRBEHAVIORSBASE_HH

/// IA 2015
// Basics
class Behavior_DoNothing;
class Behavior_FollowBall;
class Behavior_Pos;

// Slacs
class Behavior_FollowEntity;
class Behavior_GoTo;

// SSL
class Behavior_Attack;
class Behavior_GK;
class Behavior_GKA;
class Behavior_MarkBall;
class Behavior_MarkBallA;
class Behavior_MarkPlayer;
class Behavior_PenaltyAssistant;
class Behavior_PenaltyGK;
class Behavior_PenaltyKick;
class Behavior_Recv;

// VSS
class VSSBehavior_Attack;
class VSSBehavior_Mark;
class VSSBehavior_Defense;
class VSSBehavior_GK;
class VSSBehavior_AttackA;
class VSSBehavior_GKA;
class VSSBehavior_FreeBall;

// Test
class Behavior_FollowPlayer;

#endif // WRBEHAVIORSBASE_HH
