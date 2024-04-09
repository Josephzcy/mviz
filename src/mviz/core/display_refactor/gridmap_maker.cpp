#include "gridmap_maker.h"


void GridmapMaker::init(/* args */){


}

bool GridmapMaker::update(PbType& pb_object){

  perception::FreespaceObstacles gridmap = std::get<perception::FreespaceObstacles>(pb_object);
  return true;
}
void GridmapMaker::publish(/* args */){

  
}
