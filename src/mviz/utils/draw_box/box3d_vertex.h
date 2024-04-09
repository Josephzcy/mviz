#ifndef MVIZ_UTIL_BOX3D_VERTEX
#define MVIZ_UTIL_BOX3D_VERTEX
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

namespace mviz_utils {
struct Point3D {
  double x;
  double y;
  double z;
};

struct Box3D {
  Point3D center;
  double yaw;
  double roll;
  double pitch;
  double length;
  double width;
  double height;
};

class Box3dVertex {
 private:
 public:
  Box3dVertex();
  ~Box3dVertex();
  void rotationMatrix(double yaw, double roll, double pitch, double matrix[3][3]);
  void calculateBoxVertices(const Box3D& box, Point3D vertices[10]);
  std::vector<std::pair<int, int>> Box3DSides{
      // loewer
      {0, 1},
      {1, 2},
      {2, 3},
      {3, 0},
      // upper
      {4, 5},
      {5, 6},
      {6, 7},
      {7, 4},
      // mid
      {0, 4},
      {1, 5},
      {2, 6},
      {3, 7},
      // front
      {0, 5},
      {1, 4},
      // line
      {8, 9},
  };
};

}  // namespace mviz_utils

#endif