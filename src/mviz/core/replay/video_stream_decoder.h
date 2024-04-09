#ifndef VIDEO_STREAM_DECODER_H_
#define VIDEO_STREAM_DECODER_H_

#include <QDebug>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

#include "json.hpp"

using nlohmann::json;

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/buffer.h>
#include <libavutil/channel_layout.h>
#include <libavutil/common.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/mem.h>
#include <libavutil/opt.h>
#include <libavutil/parseutils.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}
// 解码裸流
class VideoStreamDecoder {
 private:
 public:
  VideoStreamDecoder() {}
  /// @brief 解码，结果通过cv::Mat传出
  /// @param inbuf 输入
  /// @param insize 输入
  /// @param outMat 输出
  /// @return 0成功 非0失败 @Notice 成功不一定有输出，解码器有缓存
  virtual int decode(const uint8_t *inbuf, int insize, cv::Mat &outMat, int &status) { return -1; }

  /// @brief 解码，用imshow显示解码结果
  /// @param inbuf 输入
  /// @param insize 输入
  /// @return 0成功 非0失败 @Notice 成功不一定有输出，解码器有缓存
  virtual int decode(const uint8_t *inbuf, int insize) { return -1; }

  /// @brief 解码，结果拷贝到outbuf中，
  /// @param inbuf
  /// @param insize
  /// @param outbuf
  /// @return 0成功 非0失败 @Notice 成功不一定有输出，解码器有缓存
  virtual int decode(const uint8_t *inbuf, int insize, uint8_t *outbuf) { return -1; }
  virtual ~VideoStreamDecoder() {
    // cv::destroyAllWindows();
  }

 protected:
  const AVCodec *videoCodec_ = NULL;
  AVCodecContext *videoCodecCtx_ = NULL;
  AVCodecParserContext *parserCtx_ = NULL;
  AVPacket *pkt_ = NULL;
  AVFrame *yuvFrame_ = NULL;
  AVFrame *nv12Frame_ = NULL;
  AVFrame *rgbFrame_ = NULL;
  int pushedFrameCnt_ = 0;
  int queueSize_;
  bool creatCVWindow_{true};
  char err2str[256] = {0};
};

class VideoStreamDecoderCPU : public VideoStreamDecoder {
 private:
  virtual int decode(const uint8_t *inbuf, int insize) override;
  virtual int decode(const uint8_t *inbuf, int insize, cv::Mat &outMat, int &status) override;
  int threadCnt_{1};

 public:
  VideoStreamDecoderCPU(int codetype, int threadCnt = 1);
  ~VideoStreamDecoderCPU();
};

class VideoStreamDecoderCuda : public VideoStreamDecoder {
 private:
  virtual int decode(const uint8_t *inbuf, int insize) override;
  virtual int decode(const uint8_t *inbuf, int insize, cv::Mat &outMat, int &status) override;
  enum AVHWDeviceType deviceType_ = AV_HWDEVICE_TYPE_CUDA;
  AVBufferRef *hwDeviceCtx = NULL;

 public:
  VideoStreamDecoderCuda(int codetype);
  ~VideoStreamDecoderCuda();
};

class VideoStreamDecoderQsv : public VideoStreamDecoder {
 private:
  virtual int decode(const uint8_t *inbuf, int insize) override;

 public:
  VideoStreamDecoderQsv(int codetype);
  ~VideoStreamDecoderQsv(){};
};

enum FrameType {
  UNKONW = 0,
  PFrame = 1,
  IFrame = 32,
};  ////1：P帧； 32：I帧; 0:帧有问题（start code不对）

enum DecodeType {
  H265 = 1,
  H264 = 2,
};

struct ImgInfo {
  int frameId{0};
  int64_t offset{0};
  int size{0};
  DecodeType decodeType{DecodeType::H265};
  FrameType frameType{FrameType::PFrame};
  ImgInfo(int id = -1, int64_t off = 0, int siz = 0, DecodeType dType = DecodeType::H265,
          FrameType fType = FrameType::PFrame)
      : frameId(id), offset(off), size(siz), decodeType(dType), frameType(fType) {}
  void Print() { std::cout << "Image Ifno,id:" << frameId << ", Frame Type:" << frameType << std::endl; }
};
class VideoBinReadDecode {
 private:
  bool stop_{false};
  std::map<size_t, cv::Mat> decodedMatMap_;
  std::deque<cv::Mat> decodedMat_;
  std::deque<int> decodedFrameid_;
  std::shared_ptr<VideoStreamDecoder> imgDecoder_;
  std::mutex mutex_;
  std::condition_variable matfull_;
  std::condition_variable matempty_;

  std::ifstream imgStream_;
  std::map<int, ImgInfo> imgInfo_;  // frameid: <offset, size>
  std::vector<ImgInfo> imgInfoList_;

  std::pair<int, int> frameidMinMax_{-1, -1};
  std::atomic<unsigned int> currentFraneIdx_{0};
  int currentOutFraneIdx_{0};

  // int gopSize_{0}; //丢帧时，这个值就失效了，故弃用
  int imgWidth_;
  int imgHeight_;

  std::thread decodeThread;
  bool firstIFrame_{false};   // false表示还没遇到第一帧I帧
  int firstIFrameIndex_{-1};  // 第一个IDR帧的 index
  // int lastIFrameIndex_{-1}; // 最后一个IDR帧的 index
  DecodeType decodeType_;
  bool isReverseDecode_{false};  //默认正序解码。
  std::deque<size_t> inFrameIdx_;
  size_t maxDecodedMatNumber_{3};

  void ReadJson(std::string dataJson);
  void ReadJson(const json &dataJsonItem, const json &imgIndexItem_, const std::string binPath);
  void Decode();
  bool IsInRange();

 public:
  VideoBinReadDecode(std::string);
  // VideoBinReadDecode(std::string,std::string);
  VideoBinReadDecode(std::string dataJsonPath, std::string videoEncodeType = "H265");
  VideoBinReadDecode(const json &dataJsonItem, const json &imgIndexItem_, std::string bin_path,
                     std::string videoEncodeType);
  ~VideoBinReadDecode();
  cv::Mat GetMat(size_t &frameId);
  int GetMat(cv::Mat &img, size_t frameId);
  /// @brief 获取解码顺序
  /// @return true：正序；false：反序
  bool GetDecodeOrder() { return !isReverseDecode_; }
  /// @brief 设置解码顺序
  /// @param order true：正序；false：反序
  void SetDecodeOrder(bool order) { isReverseDecode_ = !order; }
  /// @brief 获取currentID最近的I帧的id(id <= currentID)
  /// @param currentID
  /// @return I帧的id (当前是I帧则返回自己，不是则往小的id找);找不到时返回0；
  int GetLatestIFrameID(const size_t currentID);
};

#endif
