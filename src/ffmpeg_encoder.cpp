#include "phntm_bridge/ffmpeg_encoder.hpp"
#include "phntm_bridge/lib.hpp"
#include "phntm_bridge/const.hpp"

#include <cstddef>
#include <functional>
#include <libswscale/swscale.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>

namespace phntm {

   FFmpegEncoder::FFmpegEncoder(int width, int height, AVPixelFormat opencv_format, std::string frame_id, std::string topic, std::shared_ptr<rclcpp::Node> node, std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback)
        : width(width), height(height), packet_callback(callback), frame_id(frame_id), topic(topic), node(node) {

        // Initialize FFmpeg
        avformat_network_init();
        
        // Find hardware device type
        if (hw_device == "cuda") {
            hw_device_type = av_hwdevice_find_type_by_name("cuda");
        } else if (hw_device == "vaapi") {
            hw_device_type = av_hwdevice_find_type_by_name("vaapi");
        } else {
            hw_device_type = AV_HWDEVICE_TYPE_NONE;
        }
        
        // Create hardware device context if needed
        if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (av_hwdevice_ctx_create(&hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
            }
        }
        
        // Find the H.264 encoder
        const AVCodec* codec = nullptr;
        if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            codec = avcodec_find_encoder_by_name("h264_nvenc"); // NVIDIA
            if (!codec) codec = avcodec_find_encoder_by_name("h264_vaapi"); // Intel
            if (!codec) codec = avcodec_find_encoder_by_name("h264_amf"); // AMD
        }
        
        if (!codec) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264); // Fallback to software
            //log("["+this->toString()+"] Warning: Software h.264 encoding selected for " + topic+", this is rather slow and expensive");
            RCLCPP_WARN(this->node->get_logger(), "Software h.264 encoding selected for %s, this is rather slow and expensive", this->topic.c_str());
        }
        
        if (!codec) {
            throw std::runtime_error("["+this->toString()+"] H.264 encoder not found for " + topic);
        }
        
        // Set up codec context
        this->codec_ctx = avcodec_alloc_context3(codec);
        if (!this->codec_ctx) {
            throw std::runtime_error("["+this->toString()+"] Could not allocate codec context");
        }
        
        this->codec_ctx->width = width;
        this->codec_ctx->height = height;
        this->codec_ctx->time_base = (AVRational){1, fps};
        this->codec_ctx->framerate = (AVRational){fps, 1};
        this->codec_ctx->pix_fmt = hw_device_type != AV_HWDEVICE_TYPE_NONE ? AV_PIX_FMT_CUDA : AV_PIX_FMT_YUV420P;
        this->codec_ctx->gop_size = gop_size; // 60
        this->codec_ctx->max_b_frames = 0;
        this->codec_ctx->thread_count = thread_count;
        this->codec_ctx->bit_rate = bit_rate; // 512 * 1024 * 8; // 0.5 MB/s
        // this->conec_ctx->
        
        //codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        this->codec_ctx->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;
        this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;     // For real-time
        // this->codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;        // Faster encoding

        if (hw_device_ctx) {
            this->codec_ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
        }
        
        // Set encoder options
        av_opt_set(this->codec_ctx->priv_data, "preset", "fast", 0);
        av_opt_set(this->codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(this->codec_ctx->priv_data, "profile", "high", 0);
        
        // Open codec
        if (avcodec_open2(this->codec_ctx, codec, nullptr) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not open codec for " + topic);
        }
        
        // Allocate frame
        frame = av_frame_alloc();
        if (!frame) {
            throw std::runtime_error("[Enc] Could not allocate frame for "+topic);
        }
        
        frame->format = this->codec_ctx->pix_fmt;
        frame->width = width;
        frame->height = height;
        
        if (av_frame_get_buffer(frame, 0) < 0) {
            throw std::runtime_error("[Enc] Could not allocate frame data for " + topic);
        }
        
        // Initialize color conversion context
        sws_ctx = sws_getContext(width, height, opencv_format, // OpenCV format
                                width, height, this->codec_ctx->pix_fmt, // codec input
                                SWS_POINT, nullptr, nullptr, nullptr);

        //avformat_write_header(format_ctx, nullptr);
        
        // this->packet_receiver_running = true;
        // this->packet_receiver_thread = std::thread(&FFmpegEncoder::packetReceiverWorker, this);
        // this->packet_receiver_thread.detach();
    }

    void FFmpegEncoder::encodeFrame(const cv::Mat& raw_frame, std_msgs::msg::Header header, bool debug_log) {
        if (raw_frame.empty()) {
            throw std::invalid_argument("[Enc] Empty frame provided");
        }

        // Convert from OpenCV BGR to encoder's format
        const uint8_t* src_data[] = { raw_frame.data };
        int src_linesize[] = { static_cast<int>(raw_frame.step) };
        
        sws_scale(sws_ctx, src_data, src_linesize, 0, height, 
                 frame->data, frame->linesize);

        // frame->data[0] = raw_frame.data;
        // frame->linesize[0] = raw_frame.step;
        frame->pts = convertToRtpTimestamp(header.stamp.sec, header.stamp.nanosec);

        // Send for encoding
        this->sendFrameToEncoder(frame, header, debug_log);
    }

    void FFmpegEncoder::sendFrameToEncoder(AVFrame* frame, std_msgs::msg::Header header, bool debug_log) {
        
        int ret = avcodec_send_frame(this->codec_ctx, frame);
        if (ret < 0) {
            throw std::runtime_error("[Enc] Error sending frame to encoder");
        }

        auto pkt = av_packet_alloc();
            if (!pkt) {
                throw std::runtime_error("[Enc] Could not allocate packet");
            }
            
        ret = avcodec_receive_packet(this->codec_ctx, pkt);
        if (ret == AVERROR(EAGAIN)) { // The encoder needs more input frames before it can output a packet
                // No more packets will be produced (flush completed)
            av_packet_free(&pkt);
            return;
        } else if (ret == AVERROR_EOF) {
            av_packet_free(&pkt);
            // this->packet_receiver_running = false;
            return;
        } else if (ret < 0) {
            av_packet_free(&pkt);
            log("[Enc] Error during encoding" + std::to_string(ret), true);
            return;
        }
            
        // if (debug_log) {
        //     log("Done encoding frame w pts=" + std::to_string(pkt->pts));
        // }

        if (packet_callback) {
            auto frame = std::make_shared<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
            frame->header = std_msgs::msg::Header();
            frame->header.frame_id = this->frame_id;
            frame->header.stamp = node->now();
            frame->encoding = "h.264";
            frame->width = width;
            frame->height = height;
            frame->flags = pkt->flags;
            frame->is_bigendian = false;
            frame->pts = pkt->pts; //calculated from the initial header stamp
            // frame->data.resize(pkt->size);
            frame->data.assign(pkt->data, pkt->data + pkt->size);
            packet_callback(frame);
        }
        
        av_packet_unref(pkt);
    }

    // void FFmpegEncoder::packetReceiverWorker() {

    //     log(GRAY + "Encoder packet receiver running" + CLR);
    //     while (this->packet_receiver_running) {

            
    //     }
    //     log(GRAY + "Encoder packet receiver stopped" + CLR);
    // }

    FFmpegEncoder::~FFmpegEncoder() {
        // Flush encoder
        flush(); // kills the thread when complete
        
        // Cleanup
        if (frame) av_frame_free(&frame);
        if (codec_ctx) avcodec_free_context(&this->codec_ctx);
        // this->codec_ctx.reset();

        if (hw_device_ctx) av_buffer_unref(&hw_device_ctx);
        if (fmt_ctx) avformat_free_context(fmt_ctx);
        if (sws_ctx) sws_freeContext(sws_ctx);
        this->node.reset();
    }

    void FFmpegEncoder::flush() {
        sendFrameToEncoder(nullptr, std_msgs::msg::Header(), false);  // Flush the encoder
    }

}