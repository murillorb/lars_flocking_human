#ifndef ASSYMETRICGAUSSFUNCTION_H
#define ASSYMETRICGAUSSFUNCTION_H

#define sigma_h 2//head
#define sigma_r 1//rear
#define sigma_s 1.5//side

#define gauss_chk_range 0.1//check range
#define gauss_curve_value 0.9

class AssymetricGaussFunction
{
public:
    AssymetricGaussFunction();
    void setTargetPosition(float x, float y, float theta);
    void setMyPosition(float x, float y, float theta);
    float getValue(double px, double py,int usePrediction);
    float getBestRange(double gaussRange = gauss_chk_range);
    float getBestRangeLR(int LR);
private:
    float targPose[3];
    float myPose[3];
};

#endif // ASSYMETRICGAUSSFUNCTION_H
