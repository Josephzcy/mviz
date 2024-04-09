#include "category_color_mapping.h"

std_msgs::ColorRGBA CategoryColorMapping::GetColor(const uint8_t& type_name) {


  return color_map_.at(static_cast<MarkerColor>(type_name));
}
void CategoryColorMapping::InitColorMap() {
  color_map_ = {
    {RED, GetColorRGBAObject(1.0, 0.0, 0.0)},
    {CYAN, GetColorRGBAObject(0.0, 1.0, 1.0)},
    {GREEN, GetColorRGBAObject(0.0, 1.0, 0.0)},
    {BLUE, GetColorRGBAObject(0.0, 0.0, 0.1)},
    {YELLOW, GetColorRGBAObject(1.0, 1.0, 0.0)},
    {MAGENTA, GetColorRGBAObject(1.0, 0.0, 1.0)}, 
    {ORANGE,GetColorRGBAObject(1.0, 0.5, 0.0) },
    {PURPLE,GetColorRGBAObject(0.5, 0.0, 0.5) },
    {PINK, GetColorRGBAObject(1.0, 0.75, 0.8)},
    {LIME, GetColorRGBAObject(0.75, 1.0, 0.0)},
    {TEAL, GetColorRGBAObject(0.0, 0.5, 0.5, 1.0)},
    {LAVENDER, GetColorRGBAObject(0.9, 0.9, 0.98)},
    {BROWN, GetColorRGBAObject(0.6, 0.4, 0.2, 1.0)},
    {GRAY, GetColorRGBAObject(0.5, 0.5, 0.5, 1.0)},
    {WHITE, GetColorRGBAObject(1.0, 1.0, 1.0)}
  };
}
std_msgs::ColorRGBA CategoryColorMapping::GetColorRGBAObject(const float& r, const float& g, const float& b,
                                                             const float a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}


CategoryColorMapping::CategoryColorMapping(){
  InitColorMap();
}