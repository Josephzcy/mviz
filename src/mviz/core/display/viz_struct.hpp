#ifndef __MVIZ_STRUCT_H
#define __MVIZ_STRUCT_H
#include <dirent.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>

#include "json.hpp"
#include "msgpack.hpp"
using nlohmann::json;

extern "C" {
#include <libavfilter/buffersrc.h>

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavutil/buffer.h"
#include "libavutil/error.h"
#include "libavutil/hwcontext.h"
// #include "libavutil/hwcontext_qsv.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
}

#define STRING(num) STR(num)  //将浮点数变量转成string, a=0.1231;STRING(a)
#define STR(num) #num         //将浮点数转成string, STRING(1.1212)

#define RED cv::Scalar(0, 0, 255)             //红
#define BLOOD cv::Scalar(0, 0, 255)           //鲜红
#define GREEN cv::Scalar(0, 255, 0)           //绿
#define BLUE cv::Scalar(255, 0, 0)            //蓝 cv::Scalar(12, 12, 200)
#define PINK cv::Scalar(203, 192, 255)        //粉色
#define CRIMSON cv::Scalar(133, 182, 199)     //猩红
#define DEEPPINK cv::Scalar(147, 20, 255)     //深粉色
#define MAGENTA cv::Scalar(255, 0, 255)       //洋红
#define PURPLE cv::Scalar(147, 20, 255)       //紫色
#define DODERBLUE cv::Scalar(255, 144, 30)    //道奇蓝
#define SKYBLUE cv::Scalar(235, 206, 135)     //天蓝色
#define CYAN cv::Scalar(255, 255, 0)          //青色
#define SPRINGGREEN cv::Scalar(113, 179, 60)  //春天的绿色
#define YELLOW cv::Scalar(0, 255, 255)        //黄色
#define GOLD cv::Scalar(0, 215, 255)          //金色
#define ORANGE cv::Scalar(0, 165, 255)        //橙色
#define CHOCOLATE cv::Scalar(20, 105, 210)    //巧克力色
#define TOMATO cv::Scalar(71, 99, 255)        //番茄色
#define LIGHTGREY cv::Scalar(211, 211, 211)   //轻灰色
#define WHITE cv::Scalar(255, 255, 255)
#define BLACK cv::Scalar(0, 0, 0)

// libflow订阅消息，接收图像和算法结果
struct FlowIPconfig {
  std::string flow_addr;
  std::string flow_port;
  std::string flow_topic;
  FlowIPconfig(std::string addr = "127.0.0.1", std::string port = "24011", std::string topic = "*")
      : flow_addr(addr), flow_port(port), flow_topic(topic) {}
  FlowIPconfig(const FlowIPconfig& F) {
    this->flow_addr = F.flow_addr;
    this->flow_port = F.flow_port;
    this->flow_topic = F.flow_topic;
  }
  FlowIPconfig& operator=(const FlowIPconfig& F) {
    if (this == &F) {
      return *this;
    }
    this->flow_addr = F.flow_addr;
    this->flow_port = F.flow_port;
    this->flow_topic = F.flow_topic;
    return *this;
  }
};

//图像通过libflow接收，解码，转码，叠加检测结果，转成ros消息发布，rviz显示
struct ImageFlowHeaderStruct {
  int32_t Height;
  int32_t Width;
  uint32_t SendTimeHigh;
  uint32_t SendTimeLow;
  int32_t FrameType;
  int32_t DataSize;
  uint32_t Seq;
  uint32_t Sec;
  uint32_t Nsec;
};
//存放解码后转成BGR的图像与图像信息
struct ImageFlowStruct {
  std::string topic;
  size_t size;
  ros::Time timestamp;  // us
  ImageFlowHeaderStruct Head;
  cv::Mat data;
};

//板子端的算法结果，msgpack打包，通过libflow发送，可视化通过libflow接收，msgpack解包，protobuff反序列化，用OpenCV画到图像上
struct NewHilMsg {
  std::string topic;
  uint32_t frame_id;
  uint32_t time_s;
  uint32_t time_us;
  std::vector<char> data;
  MSGPACK_DEFINE_MAP(topic, frame_id, time_s, time_us, data);
};
//车道线曲线方程结构体 y = c0 + c1*x + c2*x^2 + c3*x^3
struct CurveCoeff {
  float longitude_min = 1;  // view range start 类型:float 字节数:4 取值范围:(FLT_MIN~FLT_MAX)
  float longitude_max = 2;  // view range end   类型:float 字节数:4 取值范围:(FLT_MIN~FLT_MAX)
  double c0 = 3;            // 类型:double 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
  double c1 = 4;            // 类型:double 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
  double c2 = 5;            // 类型:double 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)
  double c3 = 6;
  CurveCoeff(float min, float max, double c0, double c1, double c2, double c3)
      : longitude_min(min), longitude_max(max), c0(c0), c1(c1), c2(c2), c3(c3) {}
};

extern std::map<int, std::string> VizLineType;
extern std::map<int, std::string> VizColorType;
extern std::map<int, std::string> VizLanelinePositionType;
//分割图像，返回一个std::vector<cv::Mat>
void SplitImg(cv::Mat& srcImg, int imgNum, std::vector<cv::Mat>& imgSplited);
/*规划路线的显示，需要把车辆当前的位姿和规划指令结合起来生成路径的采样序列来显示，
转换函数，输入就是定位的数据和规划的指令，输出就是轨迹的采样序列，绘制时用采样序列的x和y就行*/
// 均分采样
std::vector<double> LinSpace(const double& x_start, const double& x_end, const int& num = 10);
/**
 * @brief  根据车辆坐标与规划指令，输出规划轨迹采样
 * @param  x_start: 车辆坐标X(m)
 * @param  y_start: 车辆坐标Y(m)
 * @param  theta_start: 车辆朝向(rad)
 * @param  move_direct: 规划指令->前进后退(1/-1/0)
 * @param  trun_radius: 规划指令->转向半径(m)
 * @param  move_distance: 规划指令->行驶距离(m)
 * @param  ds_set: 采样点距(m)
 * @return 采样点序列std::vector<std::tuple<x,y,theta>>，单位同上面一致
 * @author 张鹏
 * @date   20220530
 */
std::vector<std::tuple<double, double, double>> PathSample(const double x_start, const double y_start,
                                                           const double theta_start, const int move_direct,
                                                           const double trun_radius, const double move_distance,
                                                           const double ds_set = 0.5);
//原文链接：https://blog.csdn.net/gs1069405343/article/details/83414131
// 图像、圆心、开始点、结束点、线宽
void DrawArc(cv::Mat* src, cv::Point ArcCenter, cv::Point StartPoint, cv::Point EndPoint, int Fill);
/*
 * @brief  根据规划指令，输出车身坐标系规划轨迹maker
 * @param  moveDirect: 规划指令->前进后退(1/2)
 * @param  turnRadius: 规划指令->转向半径(m) >0右拐 =0直线 <0左拐
 * @param  moveDistance: 规划指令->行驶距离(m)
 * @param  marker：车身坐标系规划轨迹maker
 * @author 许瑞龙
 * @date   20220615
 */
void getPathPoints(size_t moveDirect, float turnRadius, float moveDistance, visualization_msgs::Marker& marker);
//返回车模型的 Marker
visualization_msgs::Marker getcarModelMaker(std::string frameId, float scale = 3.0);
//返回时间戳yy/mm/dd/ hh/mm/ss的 Marker
visualization_msgs::Marker DrawTimeStamp3D(int64_t timestamp, std::string topic = "location",
                                           std::string frame_id = "odome", std::string whichSeconds = "s",
                                           float x = -5.0, float y = 0.0, float z = 0.0);

int MakeDir(std::string dir);
int EnsureDir(std::string& dir, int limit_depth);

void SaveToJson(json minieye_obj, std::string dst_dir, std::string file_name);
void SaveToJson(json minieye_obj, std::string file_name);

void PrintHead(const ImageFlowHeaderStruct& head);
#endif