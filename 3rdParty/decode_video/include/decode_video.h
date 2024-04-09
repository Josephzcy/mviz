#pragma once
#include <memory>

extern "C" {
#include <libavcodec/avcodec.h>
}

class decode_video_hw {
   public:
    decode_video_hw(int codetype); /*AV_CODEC_ID_H265 // AV_CODEC_ID_H264
                                      AV_CODEC_ID_MJPEG */
    ~decode_video_hw();
    int decode(int *w, int *h, int *status,
               int *push_framecnt); /*status return value > 0 decode succeed
                                       outbuf active*/
    int decode_rgb(int *w, int *h, int *status, int *push_framecnt);
    int decode_extbuf(uint8_t *inbuf, int insize, uint8_t *outbuf, int *status,
                      int *push_framecnt); /*status return value > 0 decode
                                              succeed outbuf active*/
    int get_queuesize(void);
    bool is_frame_available() const;
    ptrdiff_t parse(const u_char* in_data, ptrdiff_t in_size);
    const AVFrame& get_frame(void);

   private:
    class decode_video_hw_;
    std::unique_ptr<decode_video_hw_> pimpl;
};

class decode_video_sw {
   public:
    decode_video_sw(int codetype); /*AV_CODEC_ID_H265 // AV_CODEC_ID_H264
                                      AV_CODEC_ID_MJPEG */
    ~decode_video_sw();
    int decode(uint8_t *inbuf, int insize, uint8_t *outbuf, int *status,
               int *push_framecnt); /*status return value > 0 decode succeed
                                       outbuf active*/
    int decode_extbuf(uint8_t *inbuf, int insize, uint8_t *outbuf, int *status,
                      int *push_framecnt); /*status return value > 0 decode
                                              succeed outbuf active*/
    int get_queuesize(void);

   private:
    class decode_video_sw_;
    std::unique_ptr<decode_video_sw_> pimpl;
};
