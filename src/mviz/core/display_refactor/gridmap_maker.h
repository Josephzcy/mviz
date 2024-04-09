#ifndef GRIDMAP_MAKER_H_
#define GRIDMAP_MAKER_H_


#include "dispaly_base.h"
class GridmapMaker : public DispalyBase {
 private:
  /* data */
 public:
  GridmapMaker(/* args */) = default;
  ~GridmapMaker() = default;

 public:
  virtual void init() override;
  virtual bool update(PbType& pb_object) override;
  virtual void publish() override;
};
#endif