// Copyright 2023 MINIEYE
#ifndef CATEGORY_COLOR_MAPPING_H_
#define CATEGORY_COLOR_MAPPING_H_

#include <std_msgs/ColorRGBA.h>

#include <map>

class CategoryColorMapping final {
 public:
  enum MarkerColor : uint8_t {
    RED,
    CYAN,
    GREEN,
    BLUE,
    YELLOW,
    MAGENTA,
    ORANGE,
    PURPLE,
    PINK,
    LIME,
    TEAL,
    LAVENDER,
    BROWN,
    GRAY,
    WHITE
  };
  using ColorMap = std::map<MarkerColor, std_msgs::ColorRGBA>;
  ColorMap color_map_;

 private:
  CategoryColorMapping(/* args */);

 public:
  ~CategoryColorMapping() = default;
  CategoryColorMapping(const CategoryColorMapping&) = delete;
  CategoryColorMapping(CategoryColorMapping&&) = delete;

 public:
  std_msgs::ColorRGBA GetColor(const uint8_t& type_name);
  void InitColorMap();
  std_msgs::ColorRGBA GetColorRGBAObject(const float& r, const float& g, const float& b, const float a = 1.0);

  static std::map<MarkerColor, std_msgs::ColorRGBA>& GetColorMap() {
    static CategoryColorMapping category_color_mapping;
    return category_color_mapping.color_map_;
  }
};

#endif