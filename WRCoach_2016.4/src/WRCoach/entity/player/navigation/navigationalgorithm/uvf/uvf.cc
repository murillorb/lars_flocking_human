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

#include "uvf.hh"
#include <WRCoach/const/uvfconstants.hh>

UVF::UVF() {
	reset();
}

UVF::~UVF() {

}

void UVF::reset() {
	_resultantDirection.setUnknown();
}


void UVF::addBall(const Position &pos, const Velocity &vel) {
	addObstacle(pos, vel);
}

void UVF::addGoalArea(const Position &pos) {
	// TODO
}

void UVF::addOwnRobot(const Position &pos, const Velocity &vel) {
	addObstacle(pos, vel);
}

void UVF::addEnemyRobot(const Position &pos, const Velocity &vel) {
	addObstacle(pos, vel);
}

Angle UVF::getDirection() const {
	return _resultantDirection;
}

float UVF::getDistance() const {
	if(originPos().isUnknown() || goalPos().isUnknown())
		return 0.0;
	return WR::Utils::distance(originPos(), goalPos());
}

UVF* UVF::clone() const {
    return new UVF(*this);
}

void UVF::run() {
	if(originPos().isUnknown() || goalPos().isUnknown() || goalOri().isUnknown()) {
		std::cout << "[WARNING] UVF: originPos, goalPos or goalOri is unknown!\n";
		return;
	}

	// Translate x-y axis
	double xTranslate = originPos().x() - goalPos().x();
	double yTranslate = originPos().y() - goalPos().y();

	// Get orientation and position
	double ori = goalOri().value();
	double x = (xTranslate*cos(ori)) + (yTranslate*sin(ori));
	double y = (-xTranslate*sin(ori)) + (yTranslate*cos(ori));

	// Move-to-goal parameters
	double yLeft = y + UVFConstants::de();
	double yRight = y - UVFConstants::de();

	// Calculate phiTUF
	double phiTUF;
	if(y < -UVFConstants::de()) {
		phiTUF = getPhi(x, yLeft, false);
	} else if(y > UVFConstants::de()) {
		phiTUF = getPhi(x, yRight, true);
	} else {
		double phiHCW = getPhi(x, yLeft, false);
		double phiHCCW = getPhi(x, yRight, true);

		double NhCWx = cos(phiHCW);
		double NhCWy = sin(phiHCW);
		double NhCCWx = cos(phiHCCW);
		double NhCCWy = sin(phiHCCW);

		double phix = ((fabs(yLeft)*NhCCWx)+(fabs(yRight)*NhCWx))/(2*UVFConstants::de());
		double phiy = ((fabs(yLeft)*NhCCWy)+(fabs(yRight)*NhCWy))/(2*UVFConstants::de());

		phiTUF = atan2(phiy, phix);
	}

//	// Calculate phiAUF
//	//TODO iterate obstancles
//	double xAUF = 0;//((UVFObstacle)_obstacles.removeLast()).getVirtualObstaclePos().x();
//	double yAUF = 0;//((UVFObstacle)_obstacles.removeLast()).getVirtualObstaclePos().y();
//	double phiAUF = getTheta(xAUF, yAUF);
//	double dist = sqrt(pow(xAUF,2.0) + pow(yAUF,2.0));

//	// Calculate phi
	double phi;
//	double dmin = UVFConstants::dmin();
//	if(dist <= dmin) {
//		phi = phiAUF;
//	} else {
//		double g = gauss(dist-dmin);
//		phi = (phiAUF*g) + (phiTUF*(1.0-g));
//	}
	phi = phiTUF;

	// Store result
	_resultantDirection.setValue(phi+ori);
}

void UVF::addObstacle(const Position &pos, const Velocity &vel) {
	_obstacles.append(UVFObstacle(pos, vel));
}

double UVF::getPhi(double pX, double pY, bool ccw) const {
	double signal = (ccw) ? 1.0:-1.0;
	double phi = 0.0;
	double de = UVFConstants::de();
	double kr = UVFConstants::kr();

	double theta = atan2(pY, pX);
	double ro = sqrt(pow(pX,2.0) + pow(pY,2.0));
	if(ro > de)
		phi = theta + signal*(GEARSystem::Angle::pi/2.0)*(2.0-(de+kr)/(ro+kr));
	else
		phi = theta + signal*(GEARSystem::Angle::pi/2.0)*sqrt(ro/de);
	return phi;
}

double UVF::getTheta(double pX, double pY) const {
	return atan2(pY, pX);
}

double UVF::gauss(double r) const {
	double delta = UVFConstants::delta();
	return exp(-(pow(r,2.0))/(2*pow(delta,2.0)));
}


