#include "video_stream_decoder.h"
// #define Debug

extern "C" {
void av_register_all(void);
void __attribute__((weak)) av_register_all(void) {}
}
using namespace std;

// 判断当前帧 startcode是否正常（offset会导致startcode不对）
static bool IsStartCodeCorrect(const char *const data) {
  //    printf("start code: 0x%x%x%x%x \n", data[0], data[1], data[2], data[3]);
  // int int_num = (static_cast<int>(data[0]) << 24) |
  //               (static_cast<int>(data[1]) << 16) |
  //               (static_cast<int>(data[2]) << 8) |
  //               static_cast<int>(data[3]);
  // std::cout << int_num << std::endl;
  if (data[0] != 0 || data[1] != 0 || data[2] != 0 || data[3] != 1)
    return false;  // 0x00000001
  else
    return true;
}

// 使用前提：只能是 H265和 H264，只有I P这两种帧类型 (I帧指的是IDR帧)
// @Notice 先用IsStartCodeCorrect()检查startcode，在判断是否为I帧（startcode不对时，判断是否为I帧无意义）。
static bool IsIFrame(const char *const data, DecodeType type) {
  int frameType = 1;
  // printf("start code: 0x%x%x%x%x \n", data[0], data[1], data[2], data[3]);
  // printf("nalu type: 0x%x \n\n", data[4]);
  // h264,I:0x67, P:0x61; h265 I:0x40,P:0x02
  if (type == DecodeType::H264) {
    frameType = data[4] & 0x1F;
  } else {
    frameType = (data[4] & 0x7E) >> 1;
  }
  // std::cout << "int frameType " << frameType << std::endl;
  // H265 H264 P帧都是 1
  if (frameType == 1) {
    return false;
  } else {
    return true;
  }
}

void avframeNv12ToBuffer(const AVFrame *frame, unsigned char *outbuf) {
  // NV12 Y和UV分为两个Plane
  int i;
  unsigned char *start_buf;
  int wrap = frame->linesize[0];
  int xsize = frame->width;
  int ysize = frame->height;

  for (i = 0; i < ysize; i++) {
    memcpy(outbuf + i * wrap, frame->data[0] + i * wrap, xsize);
  }
  start_buf = outbuf + xsize * ysize;
  for (i = 0; i < ysize / 2; i++) {
    memcpy(start_buf + i * wrap, frame->data[1] + i * wrap, xsize);
  }
}
void avframeI420ToBuffer(const AVFrame *frame, unsigned char *outbuf) {
  // I420 将Y、U、V分量分别打包，依次存储
  int i;
  unsigned char *start_buf;
  int wrap = frame->linesize[0];
  int xsize = frame->width;
  int ysize = frame->height;

  for (i = 0; i < ysize; i++) {
    memcpy(outbuf + i * wrap, frame->data[0] + i * wrap, xsize);
  }
  start_buf = outbuf + xsize * ysize;
  for (i = 0; i < ysize / 4; i++) {
    memcpy(start_buf + i * wrap, frame->data[1] + i * wrap, xsize);
  }
  start_buf = outbuf + xsize * ysize + xsize * ysize / 4;
  for (i = 0; i < ysize / 4; i++) {
    memcpy(start_buf + i * wrap, frame->data[2] + i * wrap, xsize);
  }
}

uint8_t outbuff[1024 * 1024 * 16];
void avframeNv12ToCvmat(const AVFrame *frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;

  AVPixelFormat fmt = (AVPixelFormat)frame->format;
  int buf_size = av_image_get_buffer_size(fmt, width, height, 1);
  av_image_copy_to_buffer(outbuff, buf_size, frame->data, frame->linesize, fmt, width, height, 1);

  // yuv 2 bgr
  cv::Mat yuv(cv::Size(width, int(height * 1.5)), CV_8UC1, outbuff);
  cv::cvtColor(yuv, image, cv::COLOR_YUV2BGR_NV12);
}

void avframeI420ToCvmat(const AVFrame *frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;

  AVPixelFormat fmt = (AVPixelFormat)frame->format;
  int buf_size = av_image_get_buffer_size(fmt, width, height, 1);
  av_image_copy_to_buffer(outbuff, buf_size, frame->data, frame->linesize, fmt, width, height, 1);

  // yuv 2 bgr
  cv::Mat yuv(cv::Size(width, int(height * 1.5)), CV_8UC1, outbuff);
  cv::cvtColor(yuv, image, cv::COLOR_YUV2BGR_I420);
}

/// @brief AVFrame to cv::Mat
/// @param frame 输入
/// @param image 需要初始化尺寸再传进来
void avframeToCvmat(const AVFrame *frame, cv::Mat &image) {
  int width = frame->width;
  int height = frame->height;
  int cvLinesizes[1];
  cvLinesizes[0] = image.step1();
  SwsContext *conversion =
      sws_getContext(width, height, (AVPixelFormat)frame->format, width, height, AVPixelFormat::AV_PIX_FMT_BGR24,
                     SWS_BICUBIC, NULL, NULL, NULL);  // SWS_BICUBIC,SWS_FAST_BILINEAR
  sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
  sws_freeContext(conversion);
}

void avframeToCvmat(const AVFrame *frame, AVFrame *bgrFrame, cv::Mat &image) {
  av_image_fill_arrays(bgrFrame->data, bgrFrame->linesize, image.data, AV_PIX_FMT_BGR24, frame->width, frame->height,
                       1);

  struct SwsContext *cvtCtx = nullptr;
  cvtCtx = sws_getContext(frame->width, frame->height, (AVPixelFormat)frame->format, frame->width, frame->height,
                          AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);

  sws_scale(cvtCtx, (const uint8_t *const *)frame->data, frame->linesize, 0, frame->height, bgrFrame->data,
            bgrFrame->linesize);

  sws_freeContext(cvtCtx);
}

/////////////////////////////////////////////////////////////
// cpu decode
////////////////////////////////////////////////////////////

VideoStreamDecoderCPU::VideoStreamDecoderCPU(int codetype, int threadCnt) : threadCnt_(threadCnt) {
  // av_register_all();
  /* find the MPEG-1 video decoder */
  videoCodec_ = avcodec_find_decoder((AVCodecID)codetype);
  if (!videoCodec_) {
    fprintf(stderr, "Codec not found\n");
    exit(1);
  }

  parserCtx_ = av_parser_init(videoCodec_->id);
  if (!parserCtx_) {
    fprintf(stderr, "parserCtx_ not found\n");
    exit(1);
  }

  videoCodecCtx_ = avcodec_alloc_context3(videoCodec_);
  if (!videoCodecCtx_) {
    fprintf(stderr, "Could not allocate video videoCodec_ context\n");
    exit(1);
  }
  videoCodecCtx_->thread_count = threadCnt;  // 默认1，方便对齐。推荐：核心数*2或0(ffmpeg调用最大线程数)
  // videoCodecCtx_->thread_safe_callbacks = 1; //ffmpeg5 已经弃用
  /* open it */
  if (avcodec_open2(videoCodecCtx_, videoCodec_, NULL) < 0) {
    fprintf(stderr, "Could not open videoCodec_\n");
    exit(1);
  }

  pkt_ = av_packet_alloc();
  if (!pkt_) {
    fprintf(stderr, "av packet_alloc fail");
    exit(1);
  }
  yuvFrame_ = av_frame_alloc();
  if (!yuvFrame_) {
    fprintf(stderr, "Could not allocate video yuvFrame_\n");
    exit(1);
  }
  nv12Frame_ = av_frame_alloc();
  if (!nv12Frame_) {
    fprintf(stderr, "Could not allocate video yuvFrame_\n");
    exit(1);
  }
  rgbFrame_ = av_frame_alloc();
  if (!rgbFrame_) {
    fprintf(stderr, "Could not allocate video yuvFrame_\n");
    exit(1);
  }
  pushedFrameCnt_ = 0;
}

int VideoStreamDecoderCPU::decode(const uint8_t *inbuf, int insize) {
  // if (creatCVWindow_)
  // {
  //     cv::namedWindow("VideoStreamDecoder", cv::WINDOW_NORMAL);
  //     cv::resizeWindow("VideoStreamDecoder", 1280, 720);
  //     creatCVWindow_ = false;
  // }
  int ret = -1;
  while (insize > 0) {
    ret = av_parser_parse2(parserCtx_, videoCodecCtx_, &pkt_->data, &pkt_->size, inbuf, insize, AV_NOPTS_VALUE, 0,
                           AV_NOPTS_VALUE);
    if (ret < 0) {
      fprintf(stderr, "Error while parsing\n");
      return -1;
    }
    inbuf += ret;
    insize -= ret;
    if (pkt_->size) {
      ret = avcodec_send_packet(videoCodecCtx_, pkt_);
      if (ret < 0) {
        fprintf(stderr, "Error sending a packet for decoding\n");
        // return -1; //不要return 回到av_parser_parse2继续，或者改完continue
      }
      // int ret = -1;
      while (ret >= 0) {
        ret = avcodec_receive_frame(videoCodecCtx_, yuvFrame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          fprintf(stderr, "Error receive frame\n");
          continue;  // 这里也不要return
        } else if (ret < 0) {
          fprintf(stderr, "Error during decoding\n");
          return -1;
        }
        break;
      }
      if (ret == 0) {
        // cv::Mat bgr(yuvFrame_->height, yuvFrame_->width, CV_8UC3);
        // avframeI420ToCvmat(yuvFrame_, bgr);
        // cv::imshow("VideoStreamDecoder", bgr);
        // cv::waitKey(2);
      }
    }
    av_frame_unref(yuvFrame_);
    av_packet_unref(pkt_);
  }
  return 0;
}

int VideoStreamDecoderCPU::decode(const uint8_t *inbuf, int insize, cv::Mat &outMat, int &status) {
  int ret = -1;
  //    int status = -1;
  while (insize > 0) {
    ret = av_parser_parse2(parserCtx_, videoCodecCtx_, &pkt_->data, &pkt_->size, inbuf, insize, AV_NOPTS_VALUE, 0,
                           AV_NOPTS_VALUE);
    if (ret < 0) {
      av_strerror(ret, err2str, sizeof(err2str));
      fprintf(stderr, "DecoderError(av_parser_parse2):%s\n", err2str);
      return -1;
    }
    inbuf += ret;
    insize -= ret;
    if (pkt_->size) {
      ret = avcodec_send_packet(videoCodecCtx_, pkt_);
      if (ret < 0) {
        av_strerror(ret, err2str, sizeof(err2str));
        fprintf(stderr, "DecoderError(avcodec_send_packet):%s\n", err2str);
        // return -1; //不要return 回到av_parser_parse2继续，或者改完continue
      }
      // int ret = -1;
      while (ret >= 0) {
        ret = avcodec_receive_frame(videoCodecCtx_, yuvFrame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          av_strerror(ret, err2str, sizeof(err2str));
          fprintf(stderr, "DecoderError(avcodec_receive_frame):%s\n", err2str);
          continue;  // 这里也不要return
        } else if (ret < 0) {
          av_strerror(ret, err2str, sizeof(err2str));
          fprintf(stderr, "DecoderError(avcodec_receive_frame):%s\n", err2str);
          return -1;
        }
#ifdef Debug
        fprintf(stderr, "(avcodec_receive_frame):success\n");
#endif
        break;
      }
      if (ret == 0) {
        // avframeI420ToCvmat(yuvFrame_, outMat); //  % 360%
        // avframeToCvmat(yuvFrame_, outMat); //58% 286%
        avframeToCvmat(yuvFrame_, rgbFrame_, outMat);  // 58% 78%三
        ++status;
      }
    }
    av_frame_unref(yuvFrame_);
    av_packet_unref(pkt_);
  }
  return ret;
}

VideoStreamDecoderCPU::~VideoStreamDecoderCPU() {
  if (!pkt_) av_packet_free(&pkt_);
  if (!yuvFrame_) av_frame_free(&yuvFrame_);
  if (!nv12Frame_) av_frame_free(&nv12Frame_);
  if (!rgbFrame_) av_frame_free(&rgbFrame_);
  if (!videoCodecCtx_) avcodec_free_context(&videoCodecCtx_);
  if (!videoCodecCtx_) avcodec_close(videoCodecCtx_);
}

/////////////////////////////////////////////////////////////
// cuda decode
////////////////////////////////////////////////////////////

AVPixelFormat getHwFormat(AVCodecContext *ctx, const AVPixelFormat *pix_fmts) { return AV_PIX_FMT_CUDA; }
VideoStreamDecoderCuda::VideoStreamDecoderCuda(int codetype) {
  // av_register_all();

  int ret = -1;
  /* open the hardware device */
  ret = av_hwdevice_ctx_create(&hwDeviceCtx, deviceType_, NULL, NULL, 0);
  if (ret < 0) {
    //        fprintf(stderr, "Cannot open the hardware device\n");
    //        exit(1);
    throw "Cannot open the hardware device\n";
  }
  char codecname[16];
  memset(codecname, 0x0, sizeof(codecname));
  if (AV_CODEC_ID_H265 == codetype) {
    sprintf(codecname, "hevc_cuvid");
  } else if (AV_CODEC_ID_H264 == codetype) {
    sprintf(codecname, "h264_cuvid");
  } else if (AV_CODEC_ID_MJPEG == codetype) {
    sprintf(codecname, "mjpeg_cuvid");
  }
  videoCodec_ = avcodec_find_decoder_by_name(codecname);
  if (!videoCodec_) {
    throw "The cuda decoder is not present in libavcodec\n";
    //        fprintf(stderr, "The cuda decoder is not present in libavcodec\n");
    //        exit(1);
  }
  printf("inited avcodec %s\n", codecname);

  parserCtx_ = av_parser_init(videoCodec_->id);
  if (!parserCtx_) {
    throw "parser not found\n";
    //        fprintf(stderr, "parser not found\n");
    //        exit(1);
  }

  if (!(videoCodecCtx_ = avcodec_alloc_context3(videoCodec_))) {
    throw "avcodec_alloc_context3 failed\n Failed to create decoder content based on decoder parameters\n";
    //        exit(1);
  }

  videoCodecCtx_->codec_id = (AVCodecID)codetype;
  videoCodecCtx_->get_format = getHwFormat;
  videoCodecCtx_->hw_device_ctx = av_buffer_ref(hwDeviceCtx);

  /* open it */
  if ((ret = avcodec_open2(videoCodecCtx_, videoCodec_, NULL)) < 0) {
    //        fprintf(stderr, "Could not open videoCodec_\n");
    //        exit(1);
    throw "Could not open videoCodec_\n";
  }

  pkt_ = av_packet_alloc();
  if (!pkt_) {
    throw "av packet_alloc fail";
    //        fprintf(stderr, "av packet_alloc fail");
    //        exit(1);
  }
  yuvFrame_ = av_frame_alloc();
  if (!yuvFrame_) {
    throw "Could not allocate video yuvFrame_\n";
    //        fprintf(stderr, "Could not allocate video yuvFrame_\n");
    //        exit(1);
  }
  nv12Frame_ = av_frame_alloc();
  if (!nv12Frame_) {
    throw "Could not allocate video yuvFrame_\n";
    //        fprintf(stderr, "Could not allocate video yuvFrame_\n");
    //        exit(1);
  }
  rgbFrame_ = av_frame_alloc();
  if (!rgbFrame_) {
    throw "Could not allocate video yuvFrame_\n";
    //        fprintf(stderr, "Could not allocate video yuvFrame_\n");
    //        exit(1);
  }
  pushedFrameCnt_ = 0;
}

int VideoStreamDecoderCuda::decode(const uint8_t *inbuf, int insize) {
  // if (creatCVWindow_)
  // {
  //     cv::namedWindow("VideoStreamDecoder", cv::WINDOW_NORMAL);
  //     cv::resizeWindow("VideoStreamDecoder", 1280, 720);
  //     creatCVWindow_ = false;
  // }
  int ret = -1;
  while (insize > 0) {
    ret = av_parser_parse2(parserCtx_, videoCodecCtx_, &pkt_->data, &pkt_->size, inbuf, insize, AV_NOPTS_VALUE, 0,
                           AV_NOPTS_VALUE);
    if (ret < 0) {
      av_strerror(ret, err2str, sizeof(err2str));
      fprintf(stderr, "DecoderError(av_parser_parse2):%s\n", err2str);
      return -1;
    }
    inbuf += ret;
    insize -= ret;
    if (pkt_->size) {
      ret = avcodec_send_packet(videoCodecCtx_, pkt_);
      if (ret < 0) {
        av_strerror(ret, err2str, sizeof(err2str));
        fprintf(stderr, "DecoderError(avcodec_send_packet):%s\n", err2str);
        // return -1; //不要return 继续av_parser_parse2，或者改完continue
      }
      // int ret = -1;
      while (ret >= 0) {
        ret = avcodec_receive_frame(videoCodecCtx_, yuvFrame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          av_strerror(ret, err2str, sizeof(err2str));
          fprintf(stderr, "DecoderError(avcodec_receive_frame):%s\n", err2str);
          continue;  // 这里也不要return
        } else if (ret < 0) {
          av_strerror(ret, err2str, sizeof(err2str));
          fprintf(stderr, "DecoderError(avcodec_receive_frame):%s\n", err2str);
          return -1;
        }
#ifdef Debug
        fprintf(stderr, "(avcodec_receive_frame):success\n");
#endif
        break;
      }
      if (ret == 0) {
        if (yuvFrame_->format == videoCodecCtx_->pix_fmt) {
          // 将GPU中的数据转移到CPU中来
          ret = av_hwframe_transfer_data(nv12Frame_, yuvFrame_, 0);
          if (ret < 0) {
            av_strerror(ret, err2str, sizeof(err2str));
            fprintf(stderr, "DecoderError(av_hwframe_transfer_data):%s\n", err2str);
            continue;
          }
          fflush(stdout);
        }
        // cv::Mat bgr(yuvFrame_->height, yuvFrame_->width, CV_8UC3);
        // avframeNv12ToCvmat(nv12Frame_, bgr);
        // cv::imshow("VideoStreamDecoder", bgr);
        // cv::waitKey(2);
      }
    }
    av_frame_unref(yuvFrame_);
    av_packet_unref(pkt_);
  }
  return 0;
}

int VideoStreamDecoderCuda::decode(const uint8_t *inbuf, int insize, cv::Mat &outMat, int &status) {
  int ret = -1;
  //    int status = -1;
  while (insize > 0) {
    ret = av_parser_parse2(parserCtx_, videoCodecCtx_, &pkt_->data, &pkt_->size, inbuf, insize, AV_NOPTS_VALUE, 0,
                           AV_NOPTS_VALUE);
    if (ret < 0) {
      av_strerror(ret, err2str, sizeof(err2str));
      fprintf(stderr, "DecoderError(av_parser_parse2):%s\n", err2str);
      return -1;
    }
    inbuf += ret;
    insize -= ret;
    if (pkt_->size) {
      ret = avcodec_send_packet(videoCodecCtx_, pkt_);
      if (ret < 0) {
        av_strerror(ret, err2str, sizeof(err2str));
        fprintf(stderr, "DecoderError(avcodec_send_packet):%s\n", err2str);
        // return -1; //不要return 回到av_parser_parse2继续，或者改完continue
      }
      // int ret = -1;
      while (ret >= 0) {
        ret = avcodec_receive_frame(videoCodecCtx_, yuvFrame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
          av_strerror(ret, err2str, sizeof(err2str));
          fprintf(stderr, "DecoderError(avcodec_receive_frame):%s\n", err2str);
          continue;  // 这里也不要return
        } else if (ret < 0) {
          av_strerror(ret, err2str, sizeof(err2str));
          fprintf(stderr, "DecoderError(avcodec_receive_frame):%s\n", err2str);
          return -1;
        }
#ifdef Debug
        fprintf(stderr, "(avcodec_receive_frame):success\n");
#endif
        break;
      }
      if (ret == 0) {
        if (yuvFrame_->format == videoCodecCtx_->pix_fmt) {
          // 将GPU中的数据转移到CPU中来
          ret = av_hwframe_transfer_data(nv12Frame_, yuvFrame_, 0);
          if (ret < 0) {
            fprintf(stderr, "Error transferring the data to system memory\n");
            continue;
          }
          fflush(stdout);
        }
        // avframeNv12ToCvmat(nv12Frame_, outMat); //105% 270%
        // avframeToCvmat(nv12Frame_, outMat); //58% 248%
        avframeToCvmat(nv12Frame_, rgbFrame_, outMat);  // 58% 248%
        ++status;
      }
    }
    av_frame_unref(yuvFrame_);
    av_frame_unref(nv12Frame_);
    av_packet_unref(pkt_);
  }
  return ret;
}

VideoStreamDecoderCuda::~VideoStreamDecoderCuda() {
  if (!pkt_) av_packet_free(&pkt_);
  if (!yuvFrame_) av_frame_free(&yuvFrame_);
  if (!nv12Frame_) av_frame_free(&nv12Frame_);
  if (!rgbFrame_) av_frame_free(&rgbFrame_);
  if (!videoCodecCtx_) avcodec_free_context(&videoCodecCtx_);
  if (!videoCodecCtx_) avcodec_close(videoCodecCtx_);
}
/////////////////////////////////////////////////////////////
// qsv decode @Todo
////////////////////////////////////////////////////////////

////////////////

VideoBinReadDecode::VideoBinReadDecode(std::string dataJsonPath, std::string videoEncodeType) {
  int codeType;
  if (videoEncodeType == "H265") {
    codeType = AV_CODEC_ID_H265;
    decodeType_ = DecodeType::H265;
  } else if (videoEncodeType == "H264") {
    codeType = AV_CODEC_ID_H264;
    decodeType_ = DecodeType::H264;
  }

  try {
    imgDecoder_ = std::make_shared<VideoStreamDecoderCuda>(codeType);
  } catch (char const *error) {
    qDebug() << __FUNCTION__ << " Init VideoStreamDecoderCuda Fail!";
    qDebug() << __FUNCTION__ << " error: " << error;
    qDebug() << __FUNCTION__ << " Use VideoStreamDecoderCPU";
    imgDecoder_ = std::make_shared<VideoStreamDecoderCPU>(codeType);
  }
  ReadJson(dataJsonPath);
  decodeThread = std::thread(&VideoBinReadDecode::Decode, this);
}

VideoBinReadDecode::VideoBinReadDecode(const json &dataJsonItem, const json &imgIndexItem_, std::string bin_path,
                                       std::string videoEncodeType) {
  int codeType;
  if (videoEncodeType == "H265") {
    codeType = AV_CODEC_ID_H265;
    decodeType_ = DecodeType::H265;
  } else if (videoEncodeType == "H264") {
    codeType = AV_CODEC_ID_H264;
    decodeType_ = DecodeType::H264;
  }

  try {
    // imgDecoder_ = std::make_shared<VideoStreamDecoderCPU>(codeType);
    imgDecoder_ = std::make_shared<VideoStreamDecoderCuda>(codeType);
  } catch (char const *error) {
    qDebug() << __FUNCTION__ << " Init VideoStreamDecoderCuda Fail!";
    qDebug() << __FUNCTION__ << " Error: " << error;
    qDebug() << __FUNCTION__ << " Use VideoStreamDecoderCPU";
    imgDecoder_ = std::make_shared<VideoStreamDecoderCPU>(codeType);
  }
  ReadJson(dataJsonItem, imgIndexItem_, bin_path);
  decodeThread = std::thread(&VideoBinReadDecode::Decode, this);
}

VideoBinReadDecode::~VideoBinReadDecode() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_ = true;
    // decodedMatMap_.clear();
  }
  // delete imgDecoder_;
  if (decodeThread.joinable()) {
    // qDebug() << __FUNCTION__ << ":line " << __LINE__;
    matempty_.notify_all();
    decodeThread.join();
  }
  matfull_.notify_all();
  // qDebug() << __FUNCTION__ << ":line " << __LINE__;
  if (imgStream_.is_open()) {
    imgStream_.close();
  }
  // qDebug() << __FUNCTION__ << ":finish ";
}

void VideoBinReadDecode::ReadJson(std::string dataJsonPath) {
  json dataJsonItem;
  std::ifstream dataJsonStream(dataJsonPath, std::ifstream::in);
  if (!dataJsonStream.is_open()) {
    fprintf(stderr, "Can not open: %s", dataJsonPath.c_str());
    exit(1);
  }
  dataJsonStream >> dataJsonItem;
  // std::cout << "data.json " << dataJsonItem << std::endl;
  if (dataJsonItem["data"].size() != 1 || dataJsonItem["index"].size() != 1) {
    fprintf(stderr, "!=1");
    exit(1);
  }

  imgWidth_ = dataJsonItem["width"];
  imgHeight_ = dataJsonItem["height"];
  auto pos = dataJsonPath.rfind('/');
  std::string path = dataJsonPath.substr(0, pos + 1);
  std::string cameraIndexPath = path + dataJsonItem["index"][0].get<std::string>();
  std::string binPath = path + dataJsonItem["data"][0].get<std::string>();

  std::ifstream indexJsonStream(cameraIndexPath, std::ifstream::in);
  if (!indexJsonStream.is_open()) {
    fprintf(stderr, "Can not open: %s", cameraIndexPath.c_str());
    exit(1);
  }
  json imgIndexItem_;
  indexJsonStream >> imgIndexItem_;

  // std::cout << imgIndexItem_["fields"] << std::endl;
  imgStream_.open(binPath, std::ios::in | std::ios::binary);
  if (!imgStream_.is_open()) {
    fprintf(stderr, "Can not open: %s", binPath.c_str());
    exit(1);
  }
  int frameCount = imgIndexItem_.at("index").size();
  imgInfoList_.resize(frameCount);

  int idx1 = imgIndexItem_.at("fields").at("size").get<int>();
  int idx2 = imgIndexItem_.at("fields").at("frame_id").get<int>();
  int idx4 = imgIndexItem_.at("fields").at("offset").get<int>();

  for (int i = 0; i < frameCount; i++) {
    auto &index = imgIndexItem_.at("index").at(i);
    int frameId = index.at(idx2).get<int>();
    int size = index.at(idx1).get<int>();
    int64_t offset = index.at(idx4).get<int64_t>();
    ImgInfo igif;
    igif.offset = offset;
    igif.size = size;
    igif.frameId = frameId;

    // 优化：遍历整个文件在读取nas文件时特别慢，改成找到第一个I帧直接后停止遍历
    if (-1 == firstIFrameIndex_) {
      char *data = (char *)malloc(size);
      imgStream_.read(data, size);
      if (IsStartCodeCorrect(data)) {
        if (IsIFrame(data, decodeType_)) {
          if (-1 == firstIFrameIndex_) {
            firstIFrameIndex_ = i;
          }
          igif.frameType = FrameType::IFrame;
        }
      } else {
        std::cout << "frameId: " << frameId << " startcode error " << std::endl;
        igif.frameType = FrameType::UNKONW;
      }
      free(data);
    }
    imgInfoList_[i] = igif;
  }

  if (-1 == firstIFrameIndex_) {
    fprintf(stderr, "Can not find I frame\n");
    exit(2);
  }

  currentFraneIdx_ = firstIFrameIndex_;
  // cout << "frameCount " << frameCount << std::endl;
  // cout << "currentFraneIdx_ " << currentFraneIdx_ << std::endl;
  // cout << "firstIFrameIndex_ " << firstIFrameIndex_ << std::endl;
}

void VideoBinReadDecode::ReadJson(const json &dataJsonItem, const json &imgIndexItem_, const std::string binPath) {
  imgStream_.open(binPath, std::ios::in | std::ios::binary);
  if (!imgStream_.is_open()) {
    fprintf(stderr, "Can not open: %s", binPath.c_str());
    exit(1);
  }
  imgWidth_ = dataJsonItem["width"];
  imgHeight_ = dataJsonItem["height"];
  // std::cout << imgIndexItem_["fields"] << std::endl;
  int frameCount = imgIndexItem_.at("index").size();
  imgInfoList_.resize(frameCount);

  int idx1 = imgIndexItem_.at("fields").at("size").get<int>();
  int idx2 = imgIndexItem_.at("fields").at("frame_id").get<int>();
  int idx4 = imgIndexItem_.at("fields").at("offset").get<int>();

  for (int i = 0; i < frameCount; i++) {
    auto &index = imgIndexItem_.at("index").at(i);
    int frameId = index.at(idx2).get<int>();
    int size = index.at(idx1).get<int>();
    int64_t offset = index.at(idx4).get<int64_t>();
    ImgInfo igif;
    igif.offset = offset;
    igif.size = size;
    igif.frameId = frameId;

    // 优化：遍历整个文件在读取nas文件时特别慢，改成找到第一个I帧直接后停止遍历
    if (-1 == firstIFrameIndex_) {
      char *data = (char *)malloc(size);
      imgStream_.read(data, size);
      if (IsStartCodeCorrect(data)) {
        if (IsIFrame(data, decodeType_)) {
          if (-1 == firstIFrameIndex_) {
            firstIFrameIndex_ = i;
          }
          igif.frameType = FrameType::IFrame;
        }
      } else {
        std::cout << "frameId: " << frameId << " startcode error " << std::endl;
        igif.frameType = FrameType::UNKONW;
      }
      free(data);
    }
    imgInfoList_[i] = igif;
  }

  if (-1 == firstIFrameIndex_) {
    fprintf(stderr, "Can not find I frame\n");
    exit(2);
  }

  currentFraneIdx_ = firstIFrameIndex_;
  // cout << "frameCount " << frameCount << std::endl;
  // cout << "currentFraneIdx_ " << currentFraneIdx_ << std::endl;
  // cout << "firstIFrameIndex_ " << firstIFrameIndex_ << std::endl;
}

void VideoBinReadDecode::Decode() {
  // int noOutCnt = 0;
  bool decoderCacheIsOk = false;
  while (!stop_) {
    // 当 [](){} lambda为false时线程等待
    // 收到其他线程通知 && lambda为true == 被唤醒
    std::unique_lock<std::mutex> lk(mutex_);
    matempty_.wait(lk, [&]() {
      bool ret = (decodedMatMap_.size() < maxDecodedMatNumber_) && (0 <= currentFraneIdx_) &&
                 (currentFraneIdx_ < imgInfoList_.size());
      std::cout << "ID:" << std::this_thread::get_id() << ",matempty_.wait: " << currentFraneIdx_ << ",ret:" << ret
                << std::endl;
      if (stop_) return true;
      return ret;
    });
    if (stop_) continue;
#ifdef Debug
    std::cout << "decoding id " << currentFraneIdx_ << std::endl;
#endif
    // if (currentFraneIdx_ >= imgInfoList_.size() || currentFraneIdx_ < 0)
    // {
    //     continue;
    // }
    auto &curImgInfo = imgInfoList_[currentFraneIdx_];

    imgStream_.seekg(curImgInfo.offset, std::ios::beg);
    char *data = (char *)malloc(curImgInfo.size);
    imgStream_.read(data, curImgInfo.size);
    // 帧类型存到 imgInfo_
    if (IsStartCodeCorrect(data)) {
      // if (IsIFrame(data, decodeType_))
      // {
      //     curImgInfo.frameType = FrameType::IFrame;
      // }
    } else {
      curImgInfo.frameType = FrameType::UNKONW;
    }

    if (curImgInfo.frameType == FrameType::UNKONW) {
      fprintf(stderr, "Current Frame %d start code error\n", curImgInfo.frameId);
      throw "Current Frame start code error";
    }
    cv::Mat bgr(imgHeight_, imgWidth_, CV_8UC3);
    inFrameIdx_.push_back(currentFraneIdx_);
    // cv::putText(bgr, std::to_string(seq), cv::Point(100,100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 0, 0),
    // 2); 解码
    int decode_status = -1;
    int ret = imgDecoder_->decode((uint8_t *)(data), curImgInfo.size, bgr, decode_status);
    currentFraneIdx_ += 1;
    if (decode_status !=
        0)  // 解码器需要连续输入n帧码流后 才输出一帧图像。软解：n与thread_count有关。硬解不一定。用noOutCnt记录
    {
      printf("decoder no output!\n");
      free(data);
      // noOutCnt++;
      if (decoderCacheIsOk) {
        inFrameIdx_.pop_back();  //解码器中途解码失败时，需要这一帧ID直接pop出来。
      }
      continue;
    }
    decoderCacheIsOk = true;
    int seq = inFrameIdx_.front();
    // seq -= noOutCnt;
    std::string info;
    if (imgInfoList_[seq].frameType == FrameType::PFrame) {
      info += "P " + std::to_string(seq);
    } else {
      info += "I " + std::to_string(seq);
    }
    //        cv::putText(bgr, info, cv::Point(100,100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 0, 0), 2);
    decodedMatMap_[seq] = bgr;
    inFrameIdx_.pop_front();
#ifdef Debug
    std::cout << "decoded id " << seq << std::endl;
    std::cout << "decoded map size " << decodedMatMap_.size() << std::endl;
    std::cout << "decoded in_Id size " << inFrameIdx_.size() << std::endl;
#endif
    matfull_.notify_one();
    free(data);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

int VideoBinReadDecode::GetMat(cv::Mat &img, size_t getIdx) {
  if (stop_) {
    return -3;
  }
#ifdef Debug
  std::cout << "map size " << decodedMatMap_.size() << std::endl;
  std::cout << "get index" << getIdx << std::endl;
  std::cout << "curr Id:" << getIdx << ",last I frame:" << GetLatestIFrameID(getIdx) << std::endl;
#endif
  if (getIdx >= imgInfoList_.size() || getIdx < 0) {
    return -1;                                 // 越界
  } else if (int(getIdx) < firstIFrameIndex_)  // 第一个I帧之前的P帧直接返回 背景黑色的图像
  {
    cv::Mat bgr(imgHeight_, imgWidth_, CV_8UC3);
    std::string info = "P " + std::to_string(getIdx);
    //        cv::putText(bgr, info, cv::Point(100,100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 0, 0), 2);
    bgr.copyTo(img);
    return 0;
  } else {
    //当 [](){} 为false时wait()函数才会使线程等待，在收到其他线程通知时只有当pred返回true时才会被唤醒
    std::unique_lock<std::mutex> lk(mutex_);
    matfull_.wait(lk, [&]() {
      auto ret = decodedMatMap_.size() > 0;
      if (stop_) return true;
      return ret;
    });
    if (stop_) {
      return -3;
    }
    auto pos = decodedMatMap_.find(getIdx);
    if (pos != decodedMatMap_.end()) {
      img = pos->second;
      decodedMatMap_.erase(decodedMatMap_.begin(), pos);
      matempty_.notify_one();
      return 0;
    } else {
      currentFraneIdx_ = getIdx;
      decodedMatMap_.clear();
      matempty_.notify_one();
      return -2;  // 未解码
    }
  }
}

// int VideoBinReadDecode::GetLatestIFrameID(const int currentID)
// {
//     int currId = currentID;
//     if (currId < 0 || currId > imgInfoList_.size()-1) //处理越界
//     {
//         return 0;
//     }
//     while (currId >=0)
//     {
//         auto curImgInfo = imgInfoList_[currId];
//         if (curImgInfo.frameType == FrameType::IFrame)
//         {
//             return currId;
//         }
//         currId--;
//     }
//     if (currId < 0)
//     {
//         return 0;
//     }
// }

int VideoBinReadDecode::GetLatestIFrameID(const size_t currentID) {
  size_t currId = currentID;
  if (currId < 0 || currId > imgInfoList_.size() - 1)  //处理越界
  {
    return 0;
  }
  while (currId > 0) {
    auto curImgInfo = imgInfoList_[currId];

    imgStream_.seekg(curImgInfo.offset, std::ios::beg);
    char *data = (char *)malloc(curImgInfo.size);
    imgStream_.read(data, curImgInfo.size);
    // 帧类型存到 imgInfo_
    if (IsStartCodeCorrect(data)) {
      if (IsIFrame(data, decodeType_)) {
        curImgInfo.frameType = FrameType::IFrame;
      }
    } else {
      curImgInfo.frameType = FrameType::UNKONW;
    }

    if (curImgInfo.frameType == FrameType::UNKONW) {
      fprintf(stderr, "Current Frame %d start code error\n", curImgInfo.frameId);
      throw "Current Frame start code error";
    }

    if (curImgInfo.frameType == FrameType::IFrame) {
      return currId;
    }
    currId--;
  }
  return 0;
}
