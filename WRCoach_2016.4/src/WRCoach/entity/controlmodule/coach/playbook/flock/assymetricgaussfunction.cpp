#include "assymetricgaussfunction.h"
#include<math.h>
#include<stdio.h>

AssymetricGaussFunction::AssymetricGaussFunction()
{
}

void AssymetricGaussFunction::setTargetPosition(float x, float y, float theta){
    targPose[0] = x;
    targPose[1] = y;
    targPose[2] = theta;
}

void AssymetricGaussFunction::setMyPosition(float x, float y, float theta){
    myPose[0] = x;
    myPose[1] = y;
    myPose[2] = theta;
}

float AssymetricGaussFunction::getValue(double px, double py, int usePrediction){
    double alpha, nalpha, dirSigma, vala, valb, valc;
    double predx, predy, predspeed;
    if (usePrediction){
        //predictHumanPose(&predx, &predy, &predspeed);
        alpha = atan2(predy - py, predx - px) - targPose[2] - M_PI/2.0;
    }
    else
        alpha = atan2(targPose[1] - py, targPose[0] - px) - targPose[2] - M_PI/2.0;
    nalpha = atan2(sin(alpha), cos(alpha));
    if (nalpha <= 0)
        dirSigma = sigma_r;
    else
        dirSigma = sigma_h;

    vala = pow(cos(targPose[2]),2.0)/(2.0*dirSigma*dirSigma) + pow(sin(targPose[2]),2.0)/(2.0*sigma_s*sigma_s);
        valb = sin(2.0*targPose[2])/(4.0*dirSigma*dirSigma) - sin(2*targPose[2])/(4.0*sigma_s*sigma_s);
        valc = pow(sin(targPose[2]),2.0)/(2.0*dirSigma*dirSigma) + pow(cos(targPose[2]),2.0)/(2.0*sigma_s*sigma_s);

    if (usePrediction)
        return exp(-(vala*(px - predx)*(px - predx) + 2.0*valb*(px - predx)*(py - predy) + valc*(py - predy)*(py - predy)));
    else
        return exp(-(vala*(px - targPose[0])*(px - targPose[0]) + 2.0*valb*(px - targPose[0])*(py - targPose[1]) + valc*(py - targPose[1])*(py - targPose[1])));

}

float AssymetricGaussFunction::getBestRange(double gaussRange){
    double lookRange;

    double increment;
    double solutionDist = INFINITY;
    double solutionAngle;
    double incChosen;
    for (increment = -M_PI/2; increment < M_PI/2; increment+=M_PI/20.0){
        lookRange = targPose[2] + increment + gaussRange;
        while (lookRange < -M_PI)
            lookRange += 2*M_PI;
        while (lookRange > M_PI)
            lookRange -= 2*M_PI;
        if (solutionDist > fabs(gauss_curve_value-getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0))){
            solutionDist = fabs(gauss_curve_value-getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0));
            solutionAngle = lookRange;
            incChosen = increment;
        }
    }
    //printf("val = %.4f",getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0));
    return solutionAngle;

}

float AssymetricGaussFunction::getBestRangeLR(int LR){//0:left; 1:right
    double lookRange;

    double increment;
    double solutionDist = INFINITY;
    double solutionAngle;
    double incChosen;
    if (LR==0){//left
        for (increment = 0; increment < M_PI; increment+=M_PI/20.0){
            lookRange = /*targPose[2] +*/ increment;
            while (lookRange < -M_PI)
                lookRange += 2*M_PI;
            while (lookRange > M_PI)
                lookRange -= 2*M_PI;
            if (solutionDist > fabs(gauss_curve_value-getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0))){
                solutionDist = fabs(gauss_curve_value-getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0));
                solutionAngle = lookRange;
                incChosen = increment;
            }
        }
    }
    else{
        for (increment = -M_PI; increment < 0; increment+=M_PI/20.0){
            lookRange = targPose[2] + increment;
            while (lookRange < -M_PI)
                lookRange += 2*M_PI;
            while (lookRange > M_PI)
                lookRange -= 2*M_PI;
            if (solutionDist > fabs(gauss_curve_value-getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0))){
                solutionDist = fabs(gauss_curve_value-getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0));
                solutionAngle = lookRange;
                incChosen = increment;
            }
        }
    }
    printf("val = %.4f",getValue(cos(lookRange)*gauss_chk_range + myPose[0],sin(lookRange)*gauss_chk_range + myPose[1],0));
    return solutionAngle;

}
