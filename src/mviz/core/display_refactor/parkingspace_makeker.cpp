#include "parkingspace_makeker.h"
void ParkingspaceMakeker::init(/* args */) {}

bool ParkingspaceMakeker::update(PbType& pb_object) {
  perception::ParkingSpace parkingspace = std::get<perception::ParkingSpace>(pb_object);

  return true;
}
void ParkingspaceMakeker::publish(/* args */) {}
