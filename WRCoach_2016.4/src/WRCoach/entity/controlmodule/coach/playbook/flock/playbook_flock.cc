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

#include "playbook_flock.hh"
#include <WRCoach/entity/controlmodule/coach/playbook/wrplaybook.hh>

#include <math.h>

QString Playbook_Flock::name() {
    return "Playbook_Flock";
}

Playbook_Flock::Playbook_Flock() {
    setEntityDistance(0.6);
    setMaxBeam(20);
    setSamplesDistance(0.1);
    setWGNoiseGain(0.0);
    setEntityId(0);
}

int Playbook_Flock::maxNumPlayer() {
    return INT_MAX;
}

void Playbook_Flock::configure(int numPlayers) {
    _bh_followentity.clear();
    _bh_donothing.clear();
    for(quint8 i=0; i<numPlayers; i++) {
        Behavior_FollowEntity *bh_followentity = new Behavior_FollowEntity();
        usesBehavior(bh_followentity);
        _bh_followentity.push_back(bh_followentity);
    }
    _bh_goto.clear();
    for(quint8 i=0; i<numPlayers; i++) {
        Behavior_GoTo *bh_goto = new Behavior_GoTo();
        usesBehavior(bh_goto);
        _bh_goto.push_back(bh_goto);
    }
}

double Playbook_Flock::getDx(Position robot, Position target, double targAng) {
    double Dx = ((robot.x()-target.x())*cos(targAng) + (robot.y()-target.y())*sin(targAng)); //front
    return Dx;
}

double Playbook_Flock::getDy(Position robot, Position target, double targAng) {
    double Dy = ((robot.x()-target.x())*cos(targAng + M_PI/2.0) + (robot.y()-target.y())*sin(targAng + M_PI/2.0)); //front
    return Dy;
}

int Playbook_Flock::findNearest(int id, int numPlayers){
    int nid;
    int foundID = -1;
    double vx = 0.0;
    double vy = 0.0;
    double dist_squ; // Only the squared distance is used. This is the reason the square root is omitted on distance calculation.
    double minDist = getNeighborReach();
    for (nid = 0; nid < numPlayers; nid++){
        if (id != nid){
            dist_squ = ((pow(PlayerBus::ourPlayer(id)->position().x() - PlayerBus::ourPlayer(nid)->position().x(),2.0)+
                         pow(PlayerBus::ourPlayer(id)->position().y() - PlayerBus::ourPlayer(nid)->position().y(),2.0)));
            if ((dist_squ < minDist)){

               minDist = dist_squ;
               foundID = nid;
            }
        }
    }
    return foundID;
}

int Playbook_Flock::findNearestOpponent(int id, int numPlayers){
    int nid;
    int foundID = -1;
    double vx = 0.0;
    double vy = 0.0;
    double dist_squ; // Only the squared distance is used. This is the reason the square root is omitted on distance calculation.
    double minDist = getNeighborReach();
    for (nid = 0; nid < numPlayers; nid++){
        if (id != nid){
            dist_squ = ((pow(PlayerBus::ourPlayer(id)->position().x() - PlayerBus::theirPlayer(nid)->position().x(),2.0)+
                         pow(PlayerBus::ourPlayer(id)->position().y() - PlayerBus::theirPlayer(nid)->position().y(),2.0)));
            if ((dist_squ > 0.01)&&(dist_squ < minDist)){

               dist_squ = minDist;
               foundID = nid;
            }
        }
    }
    return foundID;
}

int sameQdr(double a1, double a2){
    int s1,s2,c1,c2;
    s1 = sin(a1)>0?1:0;
    s2 = sin(a2)>0?1:0;
    c1 = cos(a1)>0?1:0;
    c2 = cos(a2)>0?1:0;
    if ((s1==s2)&&(c1==c2))
        return 1;
    else
        return 0;
}

void Playbook_Flock::normalizeForces(){
    float sumOfForces = this->_cohesionForce + this->_gaussForce + this->_avoidForce + this->_avoidHumanForce;
    setCohesionForce(this->_cohesionForce/sumOfForces);
    setGaussForce(this->_gaussForce/sumOfForces);
    setAvoidForce(this->_avoidForce/sumOfForces);
    setAvoidHumanForce(this->_avoidHumanForce/sumOfForces);


}

void Playbook_Flock::squareObstacle(int id, int nid, Position *avoidPos){
    //Define the square's points
    double coordX[4], coordY[4], diff;

    coordX[0] = PlayerBus::theirPlayer(nid)->position().x() - this->_sqObstacleSize;
    coordX[1] = PlayerBus::theirPlayer(nid)->position().x() - this->_sqObstacleSize;
    coordX[2] = PlayerBus::theirPlayer(nid)->position().x() + this->_sqObstacleSize;
    coordX[3] = PlayerBus::theirPlayer(nid)->position().x() + this->_sqObstacleSize;
    coordY[0] = PlayerBus::theirPlayer(nid)->position().y() - this->_sqObstacleSize;
    coordY[1] = PlayerBus::theirPlayer(nid)->position().y() + this->_sqObstacleSize;
    coordY[2] = PlayerBus::theirPlayer(nid)->position().y() - this->_sqObstacleSize;
    coordY[3] = PlayerBus::theirPlayer(nid)->position().y() + this->_sqObstacleSize;

    //lado vertical esquerdo (0,1)

    //if (PlayerBus::ourPlayer(id)->position().x() < coordX[0]){
        diff = PlayerBus::theirPlayer(nid)->position().y() - PlayerBus::ourPlayer(id)->position().y();
        if (fabs(diff) < this->_sqObstacleSize){
            *avoidPos = Position(true, avoidPos->x() + 1.0/pow(PlayerBus::ourPlayer(id)->position().x() - coordX[0],2), avoidPos->y(), 0.0);
        }

    //}

    //lado vertical direito (2,3)

    //if (PlayerBus::ourPlayer(id)->position().x() > coordX[2]){
        diff = PlayerBus::theirPlayer(nid)->position().y() - PlayerBus::ourPlayer(id)->position().y();
        if (fabs(diff) < this->_sqObstacleSize){
            *avoidPos = Position(true, avoidPos->x() + 1.0/pow(PlayerBus::ourPlayer(id)->position().x() - coordX[2],2), avoidPos->y(), 0.0);

        }
    //}

    //lado horizontal inferior (0,2)

    //if (PlayerBus::ourPlayer(id)->position().y() < coordY[0]){
        diff = PlayerBus::theirPlayer(nid)->position().x() - PlayerBus::ourPlayer(id)->position().x();
        if (fabs(diff) < this->_sqObstacleSize){
            *avoidPos = Position(true, avoidPos->x(), avoidPos->y() + 1.0/pow(PlayerBus::ourPlayer(id)->position().y() - coordY[2],2), 0.0);
        }
    //}

    //lado horizontal superior (3,0)

    //if (PlayerBus::ourPlayer(id)->position().y() > coordY[1]){
        diff = PlayerBus::theirPlayer(nid)->position().x() - PlayerBus::ourPlayer(id)->position().x();
        if (fabs(diff) < this->_sqObstacleSize){
            *avoidPos = Position(true, avoidPos->x(), avoidPos->y() + 1.0/pow(PlayerBus::ourPlayer(id)->position().y() - coordY[3],2), 0.0);
        }
    //}



    return;
}

void Playbook_Flock::run(int numPlayers) {
    // Parameters
    float optimalPath;
    int nearestNeighbor, nearestOpponent, nid;
    double nnDist, noDist;
    double sideDirection, reverseSD, Dx, Dxv, Dy, Dyv, alpha, chkAngle;
    //normalizeForces();

    // For each robot...

    //AGS.setTargetPosition(PlayerBus::theirPlayer(_entityId)->position().x(),PlayerBus::theirPlayer(_entityId)->position().y(),PlayerBus::theirPlayer(_entityId)->orientation().value());

    //Treat these as VECTORS, not as positions!
    Position posToGo(true,0,0,0);
    Position posMine(true,0,0,0);
    Position posToAvoid(true,0,0,0);
    Position targetPosition(true,0,0,0);
    Position nullPosition(true,0,0,0);
    Position targetLookPosition(true,gauss_chk_range*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()),gauss_chk_range*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()),0);
    double posAvoidSize = 0.0;
    double vectorModule;
    double vectorXY[2];
    double distAttr, dist_squ, distAdv;
    AGS.setTargetPosition(PlayerBus::theirPlayer(_entityId)->position().x(),PlayerBus::theirPlayer(_entityId)->position().y(),PlayerBus::theirPlayer(_entityId)->orientation().value());
    QList<Position> samples[numPlayers][_maxBeam];
    QList<Position> allocatedSamples[numPlayers];

    float beamStep = 2*M_PI/_maxBeam;
    if (this->_enableSlacs){
    // Generate samples
    for(int i=0; i<numPlayers; i++) {
        for(int j=0; j<_maxBeam; j++) {
            for(int k=1; true; k++) {
                Position samplePos(true,
                                   PlayerBus::ourPlayer(i)->position().x() + k*_samplesDistance*cos(j*beamStep + PlayerBus::ourPlayer(i)->orientation().value()) + whiteGaussianNoise(_noiseGain),
                                   PlayerBus::ourPlayer(i)->position().y() + k*_samplesDistance*sin(j*beamStep + PlayerBus::ourPlayer(i)->orientation().value()) + whiteGaussianNoise(_noiseGain),
                                   0.0);

                // If there's an obstacle, break!
                /*if (k > 50)*/if(sampleIsValid(i, samplePos, numPlayers)==false)
                    break;

                samples[i][j].push_back(samplePos);

            }
        }
    }
    for(int i=0; i<numPlayers; i++) {
        //allocatedSamples[i].push_back(PlayerBus::ourPlayer(i)->position());
        for(int j=0; j<_maxBeam; j++) {
            for(int k=0; k<samples[i][j].size(); k++) {
                float minDistance = WR::Utils::distance(samples[i][j].at(k), PlayerBus::ourPlayer(0)->position());
                int minDistanceIndex = 0;
                for(int l=1; l<numPlayers; l++) {

                    float distance = WR::Utils::distance(samples[i][j].at(k), PlayerBus::ourPlayer(l)->position());
                    if(distance<minDistance) {
                        minDistance = distance;
                        minDistanceIndex = l;
                    }
                }
                allocatedSamples[minDistanceIndex].push_back(samples[i][j].at(k));
            }
        }
    }
    }
    // Calculate samples' averages and set behaviors
    /*for(int i=0; i<numPlayers; i++) {


        float distance = WR::Utils::distance(PlayerBus::ourPlayer(i)->position(), PlayerBus::theirPlayer(_entityId)->position());

        float lineAtan = atan2((PlayerBus::ourPlayer(i)->position().y()) - PlayerBus::theirPlayer(_entityId)->position().y(),((PlayerBus::ourPlayer(i)->position().x()) - PlayerBus::theirPlayer(_entityId)->position().x()));

        float referencePosition[2], distToDest[2];
        referencePosition[0] = PlayerBus::theirPlayer(_entityId)->position().x() + _distEntity*cos(lineAtan);
        referencePosition[1] = PlayerBus::theirPlayer(_entityId)->position().y() + _distEntity*sin(lineAtan);
    }*/


    for(int i=0; i<numPlayers; i++) {

        posMine = Position(true, PlayerBus::ourPlayer(i)->position().x(), PlayerBus::ourPlayer(i)->position().y(), 0.0); //For reference, my own position

        if(_enableSlacs){
            float xSum = 0.0;
            float ySum = 0.0;
            for(int j=0; j<allocatedSamples[i].size(); j++) {
                xSum += allocatedSamples[i].at(j).x();
                ySum += allocatedSamples[i].at(j).y();
            }

            posToAvoid = Position(true, xSum/(float)allocatedSamples[i].size() - PlayerBus::ourPlayer(i)->position().x(), ySum/(float)allocatedSamples[i].size() - PlayerBus::ourPlayer(i)->position().y(), 0.0);
        }
        //if (pow(posToAvoid.x()*posToAvoid.x() + posToAvoid.y()*posToAvoid.y(),0.5) < 1)
        //    posToAvoid = Position(true,0,0,0);

        AGS.setMyPosition(PlayerBus::ourPlayer(i)->position().x(),PlayerBus::ourPlayer(i)->position().y(),PlayerBus::ourPlayer(i)->orientation().value());
       /* gradientValues[0] = calcGaussForce(pose[0],pose[1]+checkGradientDist,1);
        gradientValues[1] = calcGaussForce(pose[0],pose[1]-checkGradientDist,1);
        gradientValues[2] = calcGaussForce(pose[0]-checkGradientDist,pose[1],1);
        gradientValues[3] = calcGaussForce(pose[0]+checkGradientDist,pose[1],1);
        */

        //up
        vectorXY[0] = 0;
        vectorXY[1] = AGS.getValue(posMine.x(),posMine.y()+0.001,0);
        //down
        vectorXY[0] += 0;
        vectorXY[1] += -AGS.getValue(posMine.x(),posMine.y()-0.001,0);
        //left
        vectorXY[0] += -AGS.getValue(posMine.x()-0.001,posMine.y(),0);
        vectorXY[1] += 0;
        //right
        vectorXY[0] += AGS.getValue(posMine.x()+0.001,posMine.y(),0);
        vectorXY[1] += 0;
        //radiusDir = atan2(buffer2-pose[1],buffer1-pose[0]);

        optimalPath = atan2(-vectorXY[1],-vectorXY[0]);

      /*  if (fabs(fabs(optimalPath)-M_PI) < 0.1){
            /*if (optimalPath > 0)
                optimalPath = M_PI-0.25;
            else
                optimalPath = M_PI+0.25;
            optimalPath = M_PI;
        }*/



        if (i == 9)
                printf("%.4f\n",optimalPath);

        distAttr = /*distance**/AGS.getValue(posMine.x(),posMine.y(),0); //get Gaussian value
        //distAttr = (log2(distAttr) - log2(gauss_curve_value));//*exp(1.0/(1.0-distAttr));//get logarithmic force adjustment

        //double cte = -log2(pow(gauss_curve_value-1,2.0)); //adjusts so that the value in the curve is 0 at gauss_curve_value
        //distAttr = (log2(pow(distAttr-1,2.0))+cte)/4.0;
        if (distAttr < gauss_curve_value)
            distAttr = tan((distAttr-gauss_curve_value)*(M_PI/2.0));
        else
            distAttr = tan((distAttr-gauss_curve_value)*(M_PI*5.0));


        if (distAttr > 2)
            distAttr = 2;

        //optimalPath = AGS.getBestRange();


        //if (distAttr < 0){
            //distAttr = (log2(distAttr) - log2(gauss_curve_value));//get logarithmic force adjustment

             //optimalPath = -optimalPath;

       // if (((distAttr)>0)||(distAttr < -10))
             posToGo = Position(true, distAttr*cos(optimalPath),distAttr*sin(optimalPath), 0.0);
       // else
        //    posToGo = Position(true,0,0,0.0);
             ////atan2(posToGo.x(),posToGo.y());
        //}
        //else{
            //distAttr = (distAttr-gauss_curve_value)*exp(1.0/(1.0-distAttr));
         //   posToGo = Position(true,(5*distAttr)*cos(optimalPath),(5*distAttr)*sin(optimalPath), 0.0); //Path to go on regard to the Gaussian curve
       // }

        /*if (fabs(distAttr) < 0.07){
            posToGo = Position(true, 0,0, 0.0);
        }*/


        if ((_sideMode)&&(i < 2)){
            int toLeft = 0;
            double ang0, ang1, angH;
            int zShift = 0;
            int oShift = 0;
            int sameQuadrant;
            angH = PlayerBus::theirPlayer(3)->orientation().value();
            if (angH > M_PI)
                angH -= 2*M_PI;
            if (angH < -M_PI)
                angH += 2*M_PI;
            ang0 = -angH + atan2(PlayerBus::ourPlayer(0)->position().y() - PlayerBus::theirPlayer(3)->position().y(),
                         PlayerBus::ourPlayer(0)->position().x() - PlayerBus::theirPlayer(3)->position().x());
            ang1 = -angH + atan2(PlayerBus::ourPlayer(1)->position().y() - PlayerBus::theirPlayer(3)->position().y(),
                         PlayerBus::ourPlayer(1)->position().x() - PlayerBus::theirPlayer(3)->position().x());

            if (ang0 > M_PI)
                ang0 -= 2*M_PI;
            if (ang0 < -M_PI)
                ang0 += 2*M_PI;
            if (ang1 > M_PI)
                ang1 -= 2*M_PI;
            if (ang1 < -M_PI)
                ang1 += 2*M_PI;
            sameQuadrant = sameQdr(ang0,ang1);
            if (ang0 < ang1)
                toLeft = 1;
            else
                toLeft = 0;
           if (((ang0 > 0)&&(ang1 > 0))||((ang0 < 0)&&(ang1 < 0))){
                toLeft = 1 - toLeft;

                if ((fabs(ang0) > M_PI/2.0))
                    zShift = 1;
                else if ((fabs(ang1) > M_PI/2.0))
                    oShift = 1;

                if (sameQuadrant){
                    zShift = 0;
                    oShift = 0;
                    if ((fabs(ang0) > fabs(ang1)))
                        zShift = 1;
                    else //if ((fabs(ang1) > fabs(ang0)))
                        oShift = 1;
                }
            }
            //sideDirection = (i==0?0.5*M_PI:-0.5*M_PI)+PlayerBus::theirPlayer(_entityId)->orientation().value();


            /*sideDirection = (i==toLeft?0.5*M_PI:-0.5*M_PI)+PlayerBus::theirPlayer(_entityId)->orientation().value();
            reverseSD = (i==toLeft?-0.5*M_PI:0.5*M_PI)+PlayerBus::theirPlayer(_entityId)->orientation().value();

            double distAdv = 1; //distância lateral
            double distFront = 1; //distância frontal

            if ((zShift==1)&&(i==0))
                posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang0<0)&&(sameQdr(ang0,ang1)==0))?M_PI/2.0:-M_PI/2.0))- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang0<0)&&(sameQdr(ang0,ang1)==0))?M_PI/2.0:-M_PI/2.0)) - posMine.y(), 0.0);
            else if ((oShift==1)&&(i==1))
                posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang1<0)&&(sameQdr(ang0,ang1)==0))?M_PI/2.0:-M_PI/2.0))- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang1<0)&&(sameQdr(ang0,ang1)==0))?M_PI/2.0:-M_PI/2.0)) - posMine.y(), 0.0);
            else
                posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);


            if ((sameQuadrant)){
                if ((zShift==1)&&(i==0))
                    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 - distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang0>0))?M_PI/2.0:-M_PI/2.0))- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() +  distAdv*sin(sideDirection)*1 - distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang0>0))?M_PI/2.0:-M_PI/2.0)) - posMine.y(), 0.0);
                else if ((oShift==1)&&(i==1))
                    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 - distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang1>0))?M_PI/2.0:-M_PI/2.0))- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 - distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()+(((ang1>0))?M_PI/2.0:-M_PI/2.0)) - posMine.y(), 0.0);
                else
                    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);

            }*/

            /*if ((zShift==1)&&(i==0))
                posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + (((ang0<0))?1:-1)*distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()+M_PI/2)- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + (((ang0<0))?1:-1)*distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()+M_PI/2) - posMine.y(), 0.0);
            else if ((oShift==1)&&(i==1))
                posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + (((ang0<0))?1:-1)*distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value()+M_PI/2)- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + (((ang0<0))?1:-1)*distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()+M_PI/2) - posMine.y(), 0.0);
            else
                posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);


            if ((sameQuadrant)){
                if ((zShift==1)&&(i==0))
                    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() - distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() - distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);
                else if ((oShift==1)&&(i==1))
                    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() - distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() - distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);
                else
                    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);

            }*/

            alpha = angH + atan2(PlayerBus::ourPlayer(i)->position().y() - PlayerBus::theirPlayer(_entityId)->position().y(),
                          PlayerBus::ourPlayer(i)->position().x() - PlayerBus::theirPlayer(_entityId)->position().x())- M_PI/2.0;

            if (alpha > M_PI)
                alpha-=2*M_PI;
            else if (alpha < -M_PI)
                alpha+=2*M_PI;
            //sideDirection = AGS.getBestRange();
            //reverseSD = PlayerBus::theirPlayer(_entityId)->orientation().value() - (sideDirection - PlayerBus::theirPlayer(_entityId)->orientation().value());

            Dx = getDx(PlayerBus::ourPlayer(i)->position(),PlayerBus::theirPlayer(_entityId)->position(),angH);
            if (i == 0)
                Dxv = getDx(PlayerBus::ourPlayer(1)->position(),PlayerBus::theirPlayer(_entityId)->position(),angH);
            else
                Dxv = getDx(PlayerBus::ourPlayer(0)->position(),PlayerBus::theirPlayer(_entityId)->position(),angH);


            Dy = getDy(PlayerBus::ourPlayer(i)->position(),PlayerBus::theirPlayer(_entityId)->position(),angH);

            if (i == 0)
                Dyv = getDy(PlayerBus::ourPlayer(1)->position(),PlayerBus::theirPlayer(_entityId)->position(),angH);
            else
                Dyv = getDy(PlayerBus::ourPlayer(0)->position(),PlayerBus::theirPlayer(_entityId)->position(),angH);


            Dx -= 0.5; //ajuste para que os robôs naveguem um pouco à frente da pessoa. Se somar, estimula a ficar mais pra trás
            Dxv -= 0.5;

            if (i==0)
                printf("Dx = %.4f, Dy = %.4f",Dx,Dy);

            if ((fabs(Dx) < 0.1)&&(Dx < 0))
                Dx = 0;

                    /*
            if (fabs(Dx) > 1)
                Dx = Dx/fabs(Dx);*/

            //Dx = -Dx;
            //Dx = fabs(Dx);

            distAdv = 1;

            /*if (alpha > 0)
                sideDirection = AGS.getBestRangeLR(0);
            else
                sideDirection = AGS.getBestRangeLR(1);*/

            chkAngle = 0;
            //if (Dx < 0) //ajuste de deslocamento atrás da pessoa
                //chkAngle += M_PI;


            chkAngle += atan2(PlayerBus::ourPlayer(i)->position().y() - PlayerBus::theirPlayer(_entityId)->position().y(),
                              PlayerBus::ourPlayer(i)->position().x() - PlayerBus::theirPlayer(_entityId)->position().x()) - angH;//angH;

            //if (atan2(PlayerBus::ourPlayer(i)->position().y() - PlayerBus::theirPlayer(_entityId)->position().y(),
            //          PlayerBus::ourPlayer(i)->position().x() - PlayerBus::theirPlayer(_entityId)->position().x()) > 0)
            if (Dx < 0){
                if ((Dy > 0))
                    chkAngle -= M_PI/2;
                else
                    chkAngle += M_PI/2;

                if (Dx < Dxv)//se está atrás, ajustar trajetória para recuar!
                    if (((Dyv > 0)&&(Dy > 0))||((Dyv < 0)&&(Dy < 0))){
                        chkAngle += M_PI;
                        Dx = -1;
                    }
            }
            else{//&&(Dx > 0))
                chkAngle+=M_PI;
                if ((Dy < 0))
                    chkAngle += M_PI/2;
                else
                    chkAngle -= M_PI/2;

                if (Dx < Dxv){//se está atrás, ajustar trajetória para recuar!
                    if (((Dyv > 0)&&(Dy > 0))||((Dyv < 0)&&(Dy < 0))){
                        //chkAngle += M_PI;
                        Dx = 1;
                    }

                }
            }

            if ((fabs(Dy) > 1)){
                if (i == 1)
                    printf("Oscilando\n");
                Dx = 1;
            }

            if ((fabs(Dx) < 0.3))
                Dx = 0;


            sideDirection = AGS.getBestRange(chkAngle);

            //if ((Dx > 0)&&(Dx < 1))
             //   Dx = 1;

            //if (Dx > Dxv){//se está na frente, não há necessidade de ajustes especiais

                //if (Dx > 0){
                //    sideDirection = AGS.getBestRange(chkAngle+M_PI);
                //    Dx = 1;
                //}


            //}
            /*if (Dx < Dxv){//se está atrás, ajustar trajetória para recuar!
                //if (((ang0 > 0)&&(ang1 > 0))||((ang0 < 0)&&(ang1 < 0))){
                if (((Dyv > 0)&&(Dy > 0))||((Dyv < 0)&&(Dy < 0))){
                    //sideDirection = AGS.getBestRange(chkAngle+M_PI);
                    if (Dx > 0)
                        Dx = -1;
                    else
                        Dx = 1;
                }
            }*/
            posToGo = Position(true,distAdv*cos(sideDirection)*fabs(Dx), distAdv*sin(sideDirection)*fabs(Dx), 0.0);

            /*if((ang0<0)&&(sameQdr(ang0,ang1)==0))
                if ((zShift==1)&&(i==0))
                    if ((ang0>0)&&sameQdr(ang0,ang1))
                        sideDirection = AGS.getBestRange(alpha+M_PI);
                    else
                        sideDirection = AGS.getBestRange(alpha-M_PI);
                else if ((oShift==1)&&(i==1))
                    if ((ang0>0)&&sameQdr(ang0,ang1))
                        sideDirection = AGS.getBestRange(alpha+M_PI);
                    else
                        sideDirection = AGS.getBestRange(alpha-M_PI);
            else*/

            /*if ((zShift==1)&&(i==0))
                posToGo = Position(true,distAdv*cos(reverseSD)*Dx, distAdv*sin(reverseSD)*Dx, 0.0);
            else if ((oShift==1)&&(i==1))
                posToGo = Position(true,distAdv*cos(reverseSD)*Dx, distAdv*sin(reverseSD)*Dx, 0.0);
            else
        */
                //posToGo = Position(true,distAdv*cos(sideDirection)*Dx, distAdv*sin(sideDirection)*Dx, 0.0);




            /*if ((sameQuadrant)){
                   if ((zShift==1)&&(i==0))
                        posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() - distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() - distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);
                   else if ((oShift==1)&&(i==1))
                        posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() - distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() - distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);
                   else
                        posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + distAdv*cos(sideDirection)*1 + distFront*cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + distAdv*sin(sideDirection)*1 + distFront*sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);

            }*/

            //else
            //    posToGo = Position(true, PlayerBus::theirPlayer(_entityId)->position().x() + cos(sideDirection)*1 + cos(PlayerBus::theirPlayer(_entityId)->orientation().value())- posMine.x(),PlayerBus::theirPlayer(_entityId)->position().y() + sin(1-sideDirection)*1 + sin(PlayerBus::theirPlayer(_entityId)->orientation().value()) - posMine.y(), 0.0);
        }
        if ((_enableSlacs == 0)&&(!((nid < 2)&&(_sideMode)))){
            posToAvoid = Position(true,0,0,0); //SUBSTITUÍDO PELO SLACS
            if ((i > -1))
                for (nid = 0; nid < 6; nid++){
                    if (i != nid){
                        dist_squ = pow(WR::Utils::distance(posMine,PlayerBus::ourPlayer(nid)->position()),2.0);
                        //if (dist_squ < 2)
                            //if (((nid < 2)&&(_sideMode)))
                            //    posToAvoid = Position(true,posToAvoid.x()+0*(posMine.x()-PlayerBus::ourPlayer(nid)->position().x())/(dist_squ),posToAvoid.y()+0*(posMine.y()-PlayerBus::ourPlayer(nid)->position().y())/(dist_squ),PlayerBus::ourPlayer(nid)->orientation().value());
                            //    i = i;
                            //        else
                                posToAvoid = Position(true,posToAvoid.x()+((posMine.x()-PlayerBus::ourPlayer(nid)->position().x())/(dist_squ)),posToAvoid.y()+((posMine.y()-PlayerBus::ourPlayer(nid)->position().y())/(dist_squ)),PlayerBus::ourPlayer(nid)->orientation().value());

                    }
                }
        }
        /*for (nid = 0; nid < 6; nid++){
            if ((3 != nid)||(i < 2)){
                dist_squ = pow(WR::Utils::distance(posMine,PlayerBus::theirPlayer(nid)->position()),2.0);
                posToAvoid = Position(true,posToAvoid.x()+(posMine.x()-PlayerBus::theirPlayer(nid)->position().x())/dist_squ,posToAvoid.y()+(posMine.y()-PlayerBus::theirPlayer(nid)->position().y())/dist_squ,PlayerBus::ourPlayer(nid)->orientation().value());
            }
        }*/

        /*nearestNeighbor = findNearest(i,6);
        nearestOpponent = 3;

        nnDist = 0.0;


        if (nearestNeighbor > -1){
            nnDist = WR::Utils::distance(posMine,PlayerBus::ourPlayer(nearestNeighbor)->position());
            posToAvoid = Position(true,posMine.x()-PlayerBus::ourPlayer(nearestNeighbor)->position().x(),posMine.y()-PlayerBus::ourPlayer(nearestNeighbor)->position().y(),PlayerBus::ourPlayer(nearestNeighbor)->orientation().value());
            posToAvoid = Position(true,posToAvoid.x()/(nnDist*nnDist),posToAvoid.y()/(nnDist*nnDist),0);


        }
        else{
            nnDist = 1;
             posToAvoid = Position(true,0,0,0);
        }

        //if(nnDist < 2)

        //else
          //  nnDist = 0;

        if ((nearestNeighbor > -1)||(nearestOpponent > -1)){
            if (nearestOpponent == -1){
                nnDist = WR::Utils::distance(posMine,PlayerBus::ourPlayer(nearestNeighbor)->position());

                posToAvoid = Position(true,posMine.x()-PlayerBus::ourPlayer(nearestNeighbor)->position().x(),posMine.y()-PlayerBus::ourPlayer(nearestNeighbor)->position().y(),PlayerBus::ourPlayer(nearestNeighbor)->orientation().value());
                posToAvoid = Position(true,posToAvoid.x()/(nnDist*nnDist),posToAvoid.y()/(nnDist*nnDist),0);
            }
            else if (nearestNeighbor == -1){
                noDist = WR::Utils::distance(posMine,PlayerBus::theirPlayer(nearestOpponent)->position());

                posToAvoid = Position(true,posMine.x()-PlayerBus::theirPlayer(nearestOpponent)->position().x(),posMine.y()-PlayerBus::theirPlayer(nearestOpponent)->position().y(),PlayerBus::theirPlayer(nearestOpponent)->orientation().value());
                posToAvoid = Position(true,posToAvoid.x()/(noDist*noDist),posToAvoid.y()/(noDist*noDist),0);


            }
            else{
                nnDist = WR::Utils::distance(posMine,PlayerBus::ourPlayer(nearestNeighbor)->position());
                noDist = WR::Utils::distance(posMine,PlayerBus::theirPlayer(nearestOpponent)->position());
                if (nnDist < noDist){
                    posToAvoid = Position(true,posMine.x()-PlayerBus::ourPlayer(nearestNeighbor)->position().x(),posMine.y()-PlayerBus::ourPlayer(nearestNeighbor)->position().y(),PlayerBus::ourPlayer(nearestNeighbor)->orientation().value());
                    posToAvoid = Position(true,posToAvoid.x()/(nnDist*nnDist),posToAvoid.y()/(nnDist*nnDist),0);

                }
                else{
                    posToAvoid = Position(true,posMine.x()-PlayerBus::theirPlayer(nearestOpponent)->position().x(),posMine.y()-PlayerBus::theirPlayer(nearestOpponent)->position().y(),PlayerBus::theirPlayer(nearestOpponent)->orientation().value());
                    posToAvoid = Position(true,posToAvoid.x()/(noDist*noDist),posToAvoid.y()/(noDist*noDist),0);

                }
            }
            //posAvoidSize = pow(posToAvoid.x()*posToAvoid.x() + posToAvoid.y()*posToAvoid.y(),0.5);
            //if (posAvoidSize > gauss_chk_range){
            //    posToAvoid = Position(true,gauss_chk_range*posToAvoid.x()/posAvoidSize,gauss_chk_range*posToAvoid.y()/posAvoidSize,0.0);
            //}
        }
        else{
            posToAvoid = Position(true,0,0,0);
        }

        //Definir robôs-obstáculo

                squareObstacle(i,0,&posToAvoid);
                squareObstacle(i,1,&posToAvoid);


        posToAvoid = Position(true,posToAvoid.x()+0,posToAvoid.y()+0,0);



        /*if (nnDist > 0){
            if (getDx(posMine, PlayerBus::theirPlayer(_entityId)->position(),PlayerBus::theirPlayer(_entityId)->orientation().value()) > getDx(PlayerBus::ourPlayer(_entityId)->position(), PlayerBus::theirPlayer(_entityId)->position(),PlayerBus::theirPlayer(_entityId)->orientation().value()))
                posToAvoid = Position(true,
                            0,0,0);
        }*/


        //if(getDx(PlayerBus::ourPlayer(i)->position(), PlayerBus::theirPlayer(_entityId)->position(),PlayerBus::theirPlayer(_entityId)->orientation().value()) < 0) {
        if ((i < 2)&&(_sideMode))
            targetPosition = Position(true,
                               posToGo.x()*_gaussForce + (posToAvoid.x()*_avoidForce)/2,//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().x()),
                               posToGo.y()*_gaussForce + (posToAvoid.y()*_avoidForce)/2,//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().y()),
                               0.0);
        else
            targetPosition = Position(true,
                           posToGo.x()*_gaussForce + posToAvoid.x()*_avoidForce,//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().x()),
                           posToGo.y()*_gaussForce + posToAvoid.y()*_avoidForce,//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().y()),
                           0.0);

        vectorModule = WR::Utils::distance(targetPosition,nullPosition)*10;

        if ((i <2)&&(_sideMode)){
            if (vectorModule < 1)
                setStopState(i,1);
            if (vectorModule > 1.0)
                setStopState(i,0);
        }
        else{
            if (vectorModule < 1)
                setStopState(i,1);
            if (vectorModule > 1.0)
                setStopState(i,0);
        }

        if (getStopState(i)){

            targetPosition = Position(true,
                                      nullPosition.x(),
                                      nullPosition.y(),
                                      0.0);
            targetLookPosition = Position(true,
                                      posMine.x()+cos(PlayerBus::theirPlayer(_entityId)->orientation().value()),
                                      posMine.y()+sin(PlayerBus::theirPlayer(_entityId)->orientation().value()),
                                      0);
        }
        else{

            targetPosition = Position(true,
                                      targetPosition.x()*vectorModule,
                                      targetPosition.y()*vectorModule,
                                      0.0);
            targetLookPosition = Position(true,
                                      posMine.x()+posToGo.x(),
                                      posMine.y()+posToGo.y(),
                                      0);

        }

        double  at2 = atan2(targetPosition.y(),targetPosition.x());
        if (pow(targetPosition.x()*targetPosition.x() + targetPosition.y()*targetPosition.y(),0.5) > 0.2)
            targetPosition = Position(true,
                                      0.2*cos(at2),
                                      0.2*sin(at2),
                                      //posMine.x() + 0.1,
                                      //posMine.y() + 0.1,
                                      0.0);

        targetPosition = Position(true,
                                      targetPosition.x()+posMine.x(),
                                      targetPosition.y()+posMine.y(),
                                      0.0);



        targetLookPosition = Position(true,
                                  posMine.x()+cos(PlayerBus::theirPlayer(_entityId)->orientation().value()),
                                  posMine.y()+sin(PlayerBus::theirPlayer(_entityId)->orientation().value()),
                                  0);


        /* float distance = WR::Utils::distance(PlayerBus::ourPlayer(i)->position(), PlayerBus::theirPlayer(_entityId)->position());

         float lineAtan = atan2((PlayerBus::ourPlayer(i)->position().y()) - PlayerBus::theirPlayer(_entityId)->position().y(),((PlayerBus::ourPlayer(i)->position().x()) - PlayerBus::theirPlayer(_entityId)->position().x()));

         float referencePosition[2];
         referencePosition[0] = PlayerBus::theirPlayer(_entityId)->position().x() + _distEntity*cos(lineAtan);
         referencePosition[1] = PlayerBus::theirPlayer(_entityId)->position().y() + _distEntity*sin(lineAtan);*/

         /*   posToLook = Position(true,
                                 referencePosition[0],
                    referencePosition[1], 0.0);

        //distToDest[0] = ((1-_forceWeight)*posToGo.x() + _forceWeight*referencePosition[0]) - PlayerBus::ourPlayer(i)->position().x();
        //distToDest[1] = ((1-_forceWeight)*posToGo.y() + _forceWeight*referencePosition[1]) - PlayerBus::ourPlayer(i)->position().y();

        /*if (pow((distToDest[0]*distToDest[0])+(distToDest[1]*distToDest[1]),0.5) < 0.2)
            posToGo = Position(true,
                               PlayerBus::ourPlayer(i)->position().x(),//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().x()),
                               PlayerBus::ourPlayer(i)->position().y(),//fixForce*_forceWeight*PlayerBus::theirPlayer(_entityId)->position().y()),
                               0.0);
        */

        /*if (i == 3){
            targetPosition = Position(true,
                                      posToAvoid.x() + posMine.x(),
                                      posToAvoid.y() + posMine.y(),
                                      0.0);
            targetLookPosition = Position(true,
                                          posToAvoid.x() + posMine.x(),
                                          posToAvoid.y() + posMine.y(),
                                      0);

        }*/



        Behavior_GoTo *bh_goto = _bh_goto.at(i);
        setPlayerBehavior(i, bh_goto);
        at2 = atan2(posMine.y(),posMine.x());

        if ((PlayerBus::ourPlayer(i)->teamId()==1)&&(i == 3)){
            targetPosition = Position(true,
                                      1.5*cos(at2+M_PI/50),
                                      1.5*sin(at2+M_PI/50),
                                      //posMine.x(),
                                      //posMine.y(),
                                      0.0);
            targetLookPosition = Position(true,
                                          targetPosition.x(),// + 0.1,
                                          targetPosition.y(),// + 0.3,
                                      0);


        }

         if ((PlayerBus::ourPlayer(i)->teamId()==1)&&(i != 3)){
             targetPosition = Position(true,

                                       posMine.x(),
                                       posMine.y(),
                                       0.0);
             targetLookPosition = Position(true,
                                           0,
                                           0,
                                       0);

         }

         at2 = atan2(targetPosition.y(),targetPosition.x());



            bh_goto->setDestination(targetPosition);


       // if ((fabs(PlayerBus::theirPlayer(_entityId)->orientation().value() - PlayerBus::ourPlayer(i)->orientation().value())>M_PI/8.0)||(getStopState(i) == 0))
      // if (getStopState(i))
            bh_goto->setPositionToLook(targetLookPosition);

       }
}

bool Playbook_Flock::sampleIsValid(int id, const Position &pos, int numPlayers) {
    if//((pos.isValid()==true) &&
       ((WR::Utils::distance(PlayerBus::ourPlayer(id)->position(), pos) <= _laserRange) ){//&&
       //(WR::Utils::distance(PlayerBus::theirPlayer(_entityId)->position(),pos) > 0.01)) {
        for(int i=0; i<numPlayers; i++) {
            if(WR::Utils::distance(pos, PlayerBus::ourPlayer(i)->position()) < 0.05)
                return false;
        }
            /*if(PlayerBus::theirPlayerAvailable(i)) {
                if(WR::Utils::distance(pos, PlayerBus::theirPlayer(i)->position()) < 0.005)
                    return false;
            }
        }*/
        return true;
    }
    else {
        return false;
    }
}

float Playbook_Flock::whiteGaussianNoise(float noiseGain) {
    float r1 = (rand()%10000)/10000.0;
    float r2 = (rand()%10000)/10000.0;
    return (pow(-2*noiseGain*log(r1),0.5)*cos(2*M_PI*r2));
}
