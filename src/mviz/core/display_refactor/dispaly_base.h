#ifndef DISPLAY_BASE_H_
#define DISPLAY_BASE_H_

#include "platform_defination.h"
class DispalyBase {
 private:
  /* data */
 public:
  DispalyBase() = default;
  ~DispalyBase() = default;

  public:
    virtual void init() = 0;
    virtual bool update(PbType& pb_object) = 0;
    virtual void publish() = 0;
};

#endif