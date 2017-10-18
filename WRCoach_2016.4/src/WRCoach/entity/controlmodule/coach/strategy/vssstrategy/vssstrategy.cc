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

#include "vssstrategy.hh"

VSSStrategy::VSSStrategy() {

}

VSSStrategy::~VSSStrategy() {

}

void VSSStrategy::setStrategyState(VSSGameState gameState, StrategyState *strategyState) {
    Strategy::setStrategyState(gameState, strategyState);
}

void VSSStrategy::runStrategy() {
    SSLGameInfo::RefProcessedState refState = getGameState();

    // Convert SSLGameInfo state to VSSGameState
    VSSGameState gameState = refState2VSSGameState(refState);
    if(gameState==UNDEFINED)
        return;

    Strategy::runStrategy(gameState, refState);
}

VSSStrategy::VSSGameState VSSStrategy::refState2VSSGameState(SSLGameInfo::RefProcessedState refState) const {
    switch(refState) {
		// TODO: VSSGameInfo (prioridade: baixa)
		case SSLGameInfo::STATE_CANTMOVE:           return HALT;
		case SSLGameInfo::STATE_GAMEON:             return GAMEON;
		case SSLGameInfo::STATE_GAMEOFF:			return GAMEOFF;
		case SSLGameInfo::STATE_THEIRDIRECTKICK:
		case SSLGameInfo::STATE_OURDIRECTKICK:		return FREE_BALL;
        case SSLGameInfo::STATE_UNDEFINED:
        default:
            std::cout << "[ERROR] VSSStrategy: undefined state, should never reach here!\n";
            return UNDEFINED;
    }
}

QString VSSStrategy::state2str(int gameState) {
    switch(gameState) {
        case HALT:              return "HALT";
        case GAMEON:            return "GAME ON";
		case GAMEOFF:			return "GAME OFF";
		case FREE_BALL:			return "FREE BALL";
        case UNDEFINED:
        default:
            return "UNDEFINED";
    }
}
