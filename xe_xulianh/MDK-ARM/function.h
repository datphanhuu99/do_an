#include <math.h>

#define Length  16.5
#define Width   19
#define Max_Angle  		45
#define PI      3.14159
#define WheelSize	18.8

#define AngleServoRatio	38/60

float SpeedRatioCalculator(float angle){
  double a = tan (angle * PI / 180);
  return (Length / a)/((Length / a) + Width);
}

float GetLargerDistance(float angle){
	double a = tan (angle * PI / 180);
	return ((Length / a) + Width)*PI/2;
}
float GetSmallerDistance(float angle){
	double a = tan (angle * PI / 180);
	return (Length / a)*PI/2;
}

float GetRound (float angle){
	return GetLargerDistance(angle)/ WheelSize ; 
}
float GetFreq(int angle){
	return GetRound(angle * AngleServoRatio)*374*2;
}