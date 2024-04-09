#ifndef PARKINGSPACE_MAKER_H_
#define PARKINGSPACE_MAKER_H_
#include "dispaly_base.h"
class ParkingspaceMakeker :public DispalyBase{
 private:
  /* data */
 public:
  ParkingspaceMakeker(/* args */) = default;
  ~ParkingspaceMakeker() = default;


 public:
  virtual void init() override;
  virtual bool update(PbType& pb_object) override;
  virtual void publish() override;
};
#endif