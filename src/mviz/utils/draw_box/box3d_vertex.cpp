#include "draw_box/box3d_vertex.h"
namespace mviz_utils {

Box3dVertex::Box3dVertex() {}

Box3dVertex::~Box3dVertex() {}

// 旋转矩阵
void Box3dVertex::rotationMatrix(double yaw, double roll, double pitch, double matrix[3][3]) {
  double cosYaw = cos(yaw);
  double sinYaw = sin(yaw);
  double cosRoll = cos(roll);
  double sinRoll = sin(roll);
  double cosPitch = cos(pitch);
  double sinPitch = sin(pitch);

  matrix[0][0] = cosYaw * cosPitch;
  matrix[0][1] = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
  matrix[0][2] = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;

  matrix[1][0] = sinYaw * cosPitch;
  matrix[1][1] = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
  matrix[1][2] = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;

  matrix[2][0] = -sinPitch;
  matrix[2][1] = cosPitch * sinRoll;
  matrix[2][2] = cosPitch * cosRoll;
}

// 计算盒子顶点坐标
void Box3dVertex::calculateBoxVertices(const Box3D& box, Point3D vertices[10]) {
  double matrix[3][3];
  rotationMatrix(box.yaw, box.roll, box.pitch, matrix);

  double halfLength = box.length / 2.0;
  double halfWidth = box.width / 2.0;
  double halfHeight = box.height / 2.0;

  // 顶点偏移量
  //        前左  前右  后右  后左
  // lower  0     1    2    3
  // upper  4     5    6    7
  double offsets[8][3] = {{halfLength, halfWidth, -halfHeight},   {halfLength, -halfWidth, -halfHeight},
                          {-halfLength, -halfWidth, -halfHeight}, {-halfLength, halfWidth, -halfHeight},
                          {halfLength, halfWidth, halfHeight},    {halfLength, -halfWidth, halfHeight},
                          {-halfLength, -halfWidth, halfHeight},  {-halfLength, halfWidth, halfHeight}};

  for (int i = 0; i < 8; i++) {
    Point3D offset = {offsets[i][0], offsets[i][1], offsets[i][2]};
    Point3D rotatedOffset;

    // 应用旋转矩阵
    rotatedOffset.x = matrix[0][0] * offset.x + matrix[0][1] * offset.y + matrix[0][2] * offset.z;
    rotatedOffset.y = matrix[1][0] * offset.x + matrix[1][1] * offset.y + matrix[1][2] * offset.z;
    rotatedOffset.z = matrix[2][0] * offset.x + matrix[2][1] * offset.y + matrix[2][2] * offset.z;

    // 计算顶点坐标
    vertices[i].x = box.center.x + rotatedOffset.x;
    vertices[i].y = box.center.y + rotatedOffset.y;
    vertices[i].z = box.center.z + rotatedOffset.z;
  }
  vertices[8] = box.center;
  vertices[9].x = (vertices[0].x + vertices[5].x) * 0.5;
  vertices[9].y = (vertices[0].y + vertices[5].y) * 0.5;
  vertices[9].z = (vertices[0].z + vertices[5].z) * 0.5;
}

}  // namespace mviz_utils