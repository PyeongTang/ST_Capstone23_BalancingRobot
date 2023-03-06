#include "PointDetector.h"
#include "math.h"

PointDetector::PointDetector(PinName x_encA, PinName x_encB, PinName y_encA, PinName y_encB) :
xencA (x_encA), xencB (x_encB), yencA (y_encA), yencB (y_encB) {}

// Public Functions

void PointDetector::printCount(Serial* pc_, bool plot) {
    if (plot) {
        pc_->printf("X pos : $%d;\t"      , x_pos);
        pc_->printf("Y pos : $%d;\t"      , y_pos);
        pc_->printf("Distance : $%d;\n"   , distance);
    }
    else {
        pc_->printf("X pos : %d\t"      , x_pos);
        pc_->printf("Y pos : %d\t"      , y_pos);
        pc_->printf("Distance : %d\n"   , distance);
    }
}

int PointDetector::getPositionX(){
    return x_pos;
}

int PointDetector::getPositionY(){
    return y_pos;
}

int PointDetector::getDistance(){
    return distance;
}

int PointDetector::getTheta(){
    return theta;
}

// Private Functions

int PointDetector::calculateDistance(int _x, int _y){
  return int(sqrt(pow(_x, 2) + pow(_y, 2)));
}

int PointDetector::calculateAngle(int _x, int _y){
    return int(atan2(double(_x), double(_y)));
}

void PointDetector::xencoderRead() {
  if      (xencB.read())     {xencoderCount++;}
  else                       {xencoderCount--;}
}

void PointDetector::yencoderRead() {
  if      (yencB.read())    {yencoderCount--;}
  else                      {yencoderCount++;}
}

int PointDetector::mapCount(int countValue){
   return (countValue - 0) * (POS_MAP_MAX - POS_MAP_MIN) / (693 - 0) + POS_MAP_MIN;
}