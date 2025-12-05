#include "phntm_bridge/ffmpeg_encoder.hpp"
#include "phntm_bridge/const.hpp"
#include "phntm_bridge/lib.hpp"

#include <cstddef>
#include <functional>
#include <libavcodec/avcodec.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
#include <mutex>
#include <ostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>
#include <string>

namespace phntm {

    std::vector<AVCodecID> FFmpegEncoder::encoder_input_logged;

    FFmpegEncoder::FFmpegEncoder(std::shared_ptr<sensor_msgs::msg::Image> first_msg,
                                 std::string topic, std::shared_ptr<rclcpp::Node> node,
                                 int depth_colormap, double depth_max_sensor_value,
                                 std::string& hw_device, int thread_count, int gop_size, int bit_rate,
                                 PacketCallback callback)
    : topic(topic), node(node), depth_colormap(depth_colormap), depth_max_sensor_value(depth_max_sensor_value), packet_callback(callback)
    {
        AVPixelFormat src_opencv_format;
        if (first_msg->encoding == "rgb8") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else if (first_msg->encoding == "bgr8") {
            src_opencv_format = AV_PIX_FMT_BGR24;
        } else if (first_msg->encoding == "mono8" || first_msg->encoding == "8uc1") {
            src_opencv_format = AV_PIX_FMT_GRAY8;
        } else if (first_msg->encoding == "mono16" || first_msg->encoding == "16uc1") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else if (first_msg->encoding == "32uc1" || first_msg->encoding == "32fc1") {
            src_opencv_format = AV_PIX_FMT_RGB24;
        } else {
            throw std::runtime_error("["+this->toString()+"] Received frame with unsupported Image encoding: " + first_msg->encoding);
        }
        
        this->use_compressed_images = false;
        this->width = first_msg->width;
        this->height = first_msg->height;
        this->frame_id = first_msg->header.frame_id;
        this->format = first_msg->encoding;

        this->initEncoder(src_opencv_format, hw_device, thread_count, gop_size, bit_rate);
    }

    void FFmpegEncoder::loadRawFrame(std::shared_ptr<sensor_msgs::msg::Image> msg, cv::Mat* out_frame) {
        if (msg->encoding == "rgb8") {

            *out_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());

        } else if (msg->encoding == "bgr8") {

            *out_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());

        } else if (msg->encoding == "mono8" || msg->encoding == "8uc1") {

            *out_frame = cv::Mat(msg->height, msg->width, CV_8UC1, msg->data.data());

        } else if (msg->encoding == "mono16" || msg->encoding == "16uc1") {

            cv::Mat mono16(msg->height, msg->width, CV_16UC1, msg->data.data());
            cv::Mat mono8;
            mono16.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
            cv::applyColorMap(mono8, *out_frame, this->depth_colormap); // Apply color map

        } else if (msg->encoding == "32uc1") {

            cv::Mat mono32(msg->height, msg->width, CV_32SC1, msg->data.data());
            cv::Mat mono8;
            mono32.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
            cv::applyColorMap(mono8, *out_frame, this->depth_colormap); // Apply color map

        } else if (msg->encoding == "32fc1") {

            cv::Mat mono32(msg->height, msg->width, CV_32FC1, msg->data.data());
            cv::Mat mono8;
            mono32.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
            cv::applyColorMap(mono8, *out_frame, this->depth_colormap); // Apply color map

        }
    }

    FFmpegEncoder::FFmpegEncoder(std::shared_ptr<sensor_msgs::msg::CompressedImage> first_msg,
                                 std::string topic, std::shared_ptr<rclcpp::Node> node,
                                 std::string& hw_device, int thread_count, int gop_size, int bit_rate,
                                 PacketCallback callback)
    : topic(topic), node(node), packet_callback(callback)                   
    {
        AVPixelFormat src_opencv_format;
        auto format_parts = split(first_msg->format, ';');
        if (format_parts.size() != 2) {
            throw std::runtime_error("["+this->toString()+"] Received frame with invalid CompressedImage format: '" + first_msg->format + "'");
        }
        auto orig_pixfmt = trim(format_parts[0]);
        auto compressed_type = trim(format_parts[1]);
        auto compressed_parts = split(compressed_type, ' ');
        
        if (compressed_parts.size() < 2) {
            throw std::runtime_error("["+this->toString()+"] Received frame with invalid CompressedImage format: '" + first_msg->format+"'");
        }

        auto codec = strToLower(compressed_parts[0]);
        if (codec == "compresseddepth") {
            throw std::runtime_error("["+this->toString()+"] CompressedDepth not yet supported; format: " + first_msg->format);
        }

        if (compressed_parts[1] != "compressed") {
            throw std::runtime_error("["+this->toString()+"] Received frame with invalid CompressedImage format: " + first_msg->format);
        }

        if (codec != "jpeg" && codec != "png") {
            throw std::runtime_error("["+this->toString()+"] Received frame with invalid CompressedImage format: " + first_msg->format);
        }

        this->compressed_pixfmt = "bgr8";
        if (compressed_parts.size() > 2) {
            this->compressed_pixfmt = compressed_parts[2];
        }

        if (codec == "jpeg" && (this->compressed_pixfmt != "bgr8" && this->compressed_pixfmt != "rgb8"))
            throw std::runtime_error("["+this->toString()+"] Received frame with invalid CompressedImage JPEG format: " + first_msg->format);
        else if (codec == "png" && (this->compressed_pixfmt != "bgr8" && this->compressed_pixfmt != "rgb8" && this->compressed_pixfmt != "bgr16" && this->compressed_pixfmt != "rgb16"))
            throw std::runtime_error("["+this->toString()+"] Received frame with invalid CompressedImage PNG format: " + first_msg->format);

        //if (this->compressed_pixfmt == "rgb8" || this->compressed_pixfmt == "rgb16") {
        //    src_opencv_format = AV_PIX_FMT_RGB24;
        //} else if (this->compressed_pixfmt == "bgr8" || this->compressed_pixfmt == "bgr16") {
        src_opencv_format = AV_PIX_FMT_BGR24;
        //}
        
        cv::Mat first_frame;
        this->loadCompressedFrame(first_msg, &first_frame);
        if (first_frame.empty())
             throw std::runtime_error("["+this->toString()+"] Failed to read first frame");

        this->use_compressed_images = true;
        this->width = first_frame.cols;
        this->height = first_frame.rows;
        this->frame_id = first_msg->header.frame_id;
        this->format = first_msg->format;
        this->initEncoder(src_opencv_format, hw_device, thread_count, gop_size, bit_rate);
    }

    void FFmpegEncoder::loadCompressedFrame(std::shared_ptr<sensor_msgs::msg::CompressedImage> msg, cv::Mat* out_frame) {
        // cv::Mat raw_frame;

        //if (this->compressed_pixfmt == "rgb8" || this->compressed_pixfmt == "bgr8") {

            *out_frame = cv::imdecode(msg->data, cv::IMREAD_COLOR); // AV_PIX_FMT_RGB24 or AV_PIX_FMT_BGR24

        // } else if (this->compressed_pixfmt == "bgr16" ||  this->compressed_pixfmt == "rgb16") {
            
        //      *out_frame = cv::imdecode(msg->data, cv::IMREAD_ANYDEPTH | cv::IMREAD_COLOR);

        // }

        // if (this->src_encoding_or_format == "rgb8") {
        //     raw_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
        // } else if (this->src_encoding_or_format == "bgr8") {
        //     raw_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
        // } else if (this->src_encoding_or_format == "mono8" || this->src_encoding_or_format == "8uc1") {
        //     raw_frame = cv::Mat(msg->height, msg->width, CV_8UC1, msg->data.data());
        // } else if (this->src_encoding_or_format == "16uc1" || this->src_encoding_or_format == "mono16") {
        //     cv::Mat mono16(msg->height, msg->width, CV_16UC1, msg->data.data());
        //     cv::Mat mono8;
        //     mono16.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
        //     cv::applyColorMap(mono8, raw_frame, this->depth_colormap); // Apply color map
        // } else if (this->src_encoding_or_format == "32uc1") {
        //     cv::Mat mono32(msg->height, msg->width, CV_32SC1, msg->data.data());
        //     cv::Mat mono8;
        //     mono32.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
        //     cv::applyColorMap(mono8, raw_frame, this->depth_colormap); // Apply color map
        // } else if (this->src_encoding_or_format == "32fc1") {
        //     cv::Mat mono32(msg->height, msg->width, CV_32FC1, msg->data.data());
        //     cv::Mat mono8;
        //     mono32.convertTo(mono8, CV_8UC1, 255.0 / this->depth_max_sensor_value); // Convert to 8-bit (0-255 range)
        //     cv::applyColorMap(mono8, raw_frame, this->depth_colormap); // Apply color map
        // } 

        //return raw_frame;
    }

    void FFmpegEncoder::initEncoder(AVPixelFormat src_opencv_format, std::string& hw_device, int thread_count, int gop_size, int bit_rate) {

        // Initialize FFmpeg
        avformat_network_init();
        
        // Find hardware device type
        if (hw_device == "cuda") {
            this->hw_device_type = av_hwdevice_find_type_by_name("cuda");
        } else if (hw_device == "vaapi") {
            this->hw_device_type = av_hwdevice_find_type_by_name("vaapi");
        } else {
            this->hw_device_type = AV_HWDEVICE_TYPE_NONE;
        }
        
        // Create hardware device context if needed
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (av_hwdevice_ctx_create(&hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
            }
        }
        
        // Find the H.264 encoder
        const AVCodec* codec = nullptr;
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (hw_device == "cuda") {
                RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to cuda", this->toString().c_str());
                codec = avcodec_find_encoder_by_name("h264_nvenc"); // NVIDIA
            } else if (hw_device == "vaapi") {
                RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to h264_amf", this->toString().c_str());
                codec = avcodec_find_encoder_by_name("h264_amf"); // try AMD
                if (!codec) {
                    RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to h264_vaapi", this->toString().c_str());
                    codec = avcodec_find_encoder_by_name("h264_vaapi"); // Intel
                }
            }
        }
        
        if (!codec) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264); // Fallback to software
            //log("["+this->toString()+"] Warning: Software h.264 encoding selected for " + topic+", this is rather slow and expensive");
            RCLCPP_WARN(this->node->get_logger(), "[%s] Software h.264 encoding selected for %s, this is rather slow and expensive", this->toString().c_str(),  this->topic.c_str());
        }
        
        if (!codec) {
            throw std::runtime_error("[" + this->toString() + "] H.264 encoder not found for " + topic);
        }
        
        // output supported input pixel formats for each codec (only once)
        if (std::find(FFmpegEncoder::encoder_input_logged.begin(), FFmpegEncoder::encoder_input_logged.end(), codec->id) == FFmpegEncoder::encoder_input_logged.end()) {
            FFmpegEncoder::encoder_input_logged.push_back(codec->id);
            if (codec->pix_fmts) {
                const enum AVPixelFormat *p = codec->pix_fmts;
                while (*p != AV_PIX_FMT_NONE) {
                    RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Supported input pixel format: %s", codec->name, av_get_pix_fmt_name(*p));
                    // Optionally, use av_get_pix_fmt_name(*p) to print the name
                    p++;
                }
            } else {
                RCLCPP_WARN(this->node->get_logger(), "[AVCodec %s] No supported input pixel formats detected!", codec->name);
            }
        }
        RCLCPP_INFO(this->node->get_logger(), "[%s] OpenCV conversion format for sw-scaling: %s", this->toString().c_str(), av_get_pix_fmt_name(src_opencv_format));

        // Set up codec context
        this->codec_ctx = avcodec_alloc_context3(codec);
        if (!this->codec_ctx) {
            throw std::runtime_error("[" + this->toString() + "] Could not allocate codec context");
        }
        
        this->codec_ctx->width = this->width;
        this->codec_ctx->height = this->height;
        this->codec_ctx->time_base = {1, this->fps}; // t
        this->codec_ctx->framerate = {this->fps, 1};
        this->codec_ctx->pix_fmt = this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI ? AV_PIX_FMT_VAAPI : AV_PIX_FMT_NV12; // this is input to the codec (output of sws_scale)
        this->codec_ctx->gop_size = gop_size; // 60
        this->codec_ctx->max_b_frames = 0;
        this->codec_ctx->thread_count = thread_count;
        this->codec_ctx->bit_rate = bit_rate; // 512 * 1024 * 8; // 0.5 MB/s
        
        this->codec_ctx->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;
        this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;     // For real-time
        // this->codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;        // Faster encoding
        
        // Set encoder options
        av_opt_set(this->codec_ctx->priv_data, "preset", "fast", 0);
        if (this->hw_device_type == AV_HWDEVICE_TYPE_NONE)
            av_opt_set(this->codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(this->codec_ctx->priv_data, "profile", "high", 0);
                
        // Initialize color conversion context
        this->sws_ctx = sws_getContext(this->width, this->height,
                                src_opencv_format, // OpenCV format
                                this->width, this->height,
                                AV_PIX_FMT_NV12, // sw scaler output, codec input
                                SWS_FAST_BILINEAR,
                                nullptr, nullptr,
                                nullptr);

        // allocate sw frames pool
        for (uint i = 0; i < this->num_frame_buffers; i++) {
            auto sw_frame = av_frame_alloc();
            if (!sw_frame) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate sw frame #" + std::to_string(i));
            }
            sw_frame->format = AV_PIX_FMT_NV12; //this->codec_ctx->pix_fmt;
            sw_frame->width = this->width;
            sw_frame->height = this->height;
            if (av_frame_get_buffer(sw_frame, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate sw frame data #" + std::to_string(i));
            }
            this->sw_frame_buffers.push_back(sw_frame);
        }

        // create hardware device context
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                // TODO: Customize/detect device
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx for VAAPI");
                if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, "/dev/dri/renderD128", nullptr, 0) < 0) {
                   throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
                }
                this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
            } else {
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx");
                if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                   throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
                }
                this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
            }
        }

        if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
            RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Making hw frames ctx", codec->name);
            
            int err = 0;
            AVBufferRef *hw_frames_ref;
            AVHWFramesContext *hw_frames_ctx = NULL;

            if (!(hw_frames_ref = av_hwframe_ctx_alloc(hw_device_ctx))) {
                throw std::runtime_error("Failed to create VAAPI frame context.");
            }
            hw_frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
            hw_frames_ctx->format = AV_PIX_FMT_VAAPI;       // Hardware pixel format
            hw_frames_ctx->sw_format = AV_PIX_FMT_NV12;     // SW pixel format to upload from
            hw_frames_ctx->width = this->width;
            hw_frames_ctx->height = this->height;
            hw_frames_ctx->initial_pool_size = this->num_frame_buffers;

            if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0) {
                av_buffer_unref(&hw_frames_ref);
                throw std::runtime_error("Failed to initialize VAAPI frame context. Error code: " + std::to_string(err));
            }

            this->codec_ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
            if (!this->codec_ctx->hw_frames_ctx) {
                av_buffer_unref(&hw_frames_ref);
                throw std::runtime_error("Failed to allocate codec_ctx->hw_frames_ctx");
            }

            av_buffer_unref(&hw_frames_ref);

            // allocate hw frames pool
            for (uint i = 0; i < this->num_frame_buffers; i++) {

                AVFrame * hw_frame;
                
                if (!(hw_frame = av_frame_alloc())) {
                    err = AVERROR(ENOMEM);
                    throw std::runtime_error("Error allocating hw frame " + std::to_string(i));
                }
                if ((err = av_hwframe_get_buffer(this->codec_ctx->hw_frames_ctx, hw_frame, 0)) < 0) {
                    throw std::runtime_error("Error getting buffer for hw frame " + std::to_string(i));
                }
                if (!hw_frame->hw_frames_ctx) {
                    throw std::runtime_error("Error checking allocated hw frame " + std::to_string(i));
                }

                this->hw_frame_buffers.push_back(hw_frame);
            }
        }
        
        // open codec
        if (avcodec_open2(this->codec_ctx, codec, nullptr) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not open codec");
        }

        // lanuch workers
        this->running = true;
        this->scaler_thread = std::thread(&FFmpegEncoder::scalerWorker, this);
        this->scaler_thread.detach();
        this->encoder_thread = std::thread(&FFmpegEncoder::encoderWorker, this);
        this->encoder_thread.detach();
    }

    void FFmpegEncoder::encodeFrame(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
        if (!this->running)
            return;

        if (msg->width != this->width || msg->height != this->height || msg->encoding != this->format || this->needs_reset) {
            this->needs_reset = true; // reset encoder
            return; 
        }

        {
            std::lock_guard<std::mutex> queue_lock(this->scaler_mutex);
            this->scaler_queue_image.push(msg);
            this->scaler_cv.notify_one();
        }
    }

     void FFmpegEncoder::encodeFrame(const std::shared_ptr<sensor_msgs::msg::CompressedImage> msg) {
        if (!this->running)
            return;

        if (msg->format != this->format || this->needs_reset) { // more checks in scaler thread, when we decode the bytes
            this->needs_reset = true;
            return;
        }

        {
            std::lock_guard<std::mutex> queue_lock(this->scaler_mutex);
            this->scaler_queue_compressed_image.push(msg);
            this->scaler_cv.notify_one();
        }
    }

    void FFmpegEncoder::sendFrameToEncoder(AVFrame* input_frame, std_msgs::msg::Header header) {
        std::lock_guard<std::mutex> queue_lock(this->encoder_mutex);
        EncoderRequest req;
        req.frame = input_frame;
        req.header = header;
        this->encoder_queue.push(req);
        this->encoder_cv.notify_one();
    }

    void FFmpegEncoder::flushEncoder() { 
    //    std::lock_guard<std::mutex> queue_lock(this->encoder_mutex);
    //     EncoderRequest req;
    //     req.frame = nullptr; //flush the encoder
    //     this->encoder_queue.push(req);
    //     this->encoder_cv.notify_one();
        int ret = avcodec_send_frame(this->codec_ctx, nullptr);
        if (ret < 0) {
            throw std::runtime_error("["+this->toString()+"] Error sending flush to encoder");
        }
    }

    void FFmpegEncoder::scalerWorker() {
        std::cout << "["+this->toString()+"] FFmpegEncoder scaler runnig..." << std::endl;
        this->scaler_running = true;

        int err;
        while (this->running) {

            try {
                std::unique_lock<std::mutex> scaler_lock(this->scaler_mutex);

                cv::Mat raw_frame;
                std_msgs::msg::Header header;
                std::shared_ptr<sensor_msgs::msg::Image> msg_image; // ref is kept until sws_scale so we don't need to .clone() cv::mat where not necessary
                std::shared_ptr<sensor_msgs::msg::CompressedImage> msg_compresed_image;

                if (!this->use_compressed_images) { // Image

                    this->scaler_cv.wait(scaler_lock, [this] { return !this->scaler_queue_image.empty() || !this->running; });

                    if (this->scaler_queue_image.empty() || !this->running) 
                        break; // end worker loop
                    
                    while (!this->scaler_queue_image.empty()) {
                        msg_image = this->scaler_queue_image.front();
                        this->scaler_queue_image.pop();
                    }
                
                    scaler_lock.unlock();

                    loadRawFrame(msg_image, &raw_frame);
                    header = msg_image->header;

                } else { // CompressedImage

                    this->scaler_cv.wait(scaler_lock, [this] { return !this->scaler_queue_compressed_image.empty() || !this->running; });

                    if (this->scaler_queue_compressed_image.empty() || !this->running) 
                        break; // end worker loop
                    
                    while (!this->scaler_queue_compressed_image.empty()) {
                        msg_compresed_image = this->scaler_queue_compressed_image.front();
                        this->scaler_queue_compressed_image.pop();
                    }

                    scaler_lock.unlock();

                    loadCompressedFrame(msg_compresed_image, &raw_frame);
                    header = msg_compresed_image->header;
                }

                if (this->needs_reset) {
                    //this->flush();
                    break; // end worker loop
                }

                if (raw_frame.empty())
                    continue; // failed to load

                AVFrame* sw_frame = this->sw_frame_buffers[this->current_frame_buffer];
                AVFrame* hw_frame = this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI ? this->hw_frame_buffers[this->current_frame_buffer] : nullptr;
                this->current_frame_buffer++;
                if (this->current_frame_buffer == this->num_frame_buffers) {
                    this->current_frame_buffer = 0;
                }

                const uint8_t* src_data[] = { raw_frame.data };
                int src_linesize[] = { static_cast<int>(raw_frame.step) };

                // std::cout << "sws_scale ctx=" << this->sws_ctx << "; src_data=" << src_data
                //           << " src_linesize=" << src_linesize[0] << "; height=" << this->height
                //           << " sw_frame->data=" << sw_frame->data << "; sw_frame->linesize=" << sw_frame->linesize[0]
                //           << std::endl;

                // sw scaling & copy here - expensive
                sws_scale(this->sws_ctx, src_data,
                        src_linesize, 0, this->height, 
                        sw_frame->data, sw_frame->linesize);

                // Send for encoding
                if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                    if ((err = av_hwframe_transfer_data(hw_frame, sw_frame, 0)) < 0) {
                    std::cerr << "FFMpegEncoder: Error while transferring frame data to surface. Error code: " << std::to_string(err) << std::endl;
                    continue;
                    }
                    // hw_frame->pts = convertToRtpTimestamp(im->header.stamp.sec, im->header.stamp.nanosec);
                    this->sendFrameToEncoder(hw_frame, header);
                } else {
                    // sw_frame->pts = convertToRtpTimestamp(im->header.stamp.sec, im->header.stamp.nanosec);
                    this->sendFrameToEncoder(sw_frame, header);
                }
            } catch (...) {
                RCLCPP_ERROR(this->node->get_logger(), "[%s] Exception in scalerWorker", this->toString().c_str());
                //this->flush();
                break;
            }
        }

        this->scaler_running = false;
        std::cout << "["+this->toString()+"] FFmpegEncoder scaler stopped..." << std::endl;
    }

    void FFmpegEncoder::encoderWorker() {

        log("["+this->toString()+"] FFmpegEncoder worker runnig...");
        this->encoder_running = true;

        while (this->running) {

            try {
                auto pkt = av_packet_alloc();
                if (!pkt) {
                    throw std::runtime_error("["+this->toString()+"] Could not allocate packet");
                }

                std::unique_lock<std::mutex> queue_lock(this->encoder_mutex);
                this->encoder_cv.wait(queue_lock, [this] { return !this->encoder_queue.empty() || !this->running; });

                if (this->encoder_queue.empty()) 
                    break;

                EncoderRequest req;
                while (!this->encoder_queue.empty()) {
                    req = this->encoder_queue.front();
                    this->encoder_queue.pop();
                }

                queue_lock.unlock();

                int ret = avcodec_send_frame(this->codec_ctx, req.frame);
                if (ret < 0) {
                    throw std::runtime_error("["+this->toString()+"] Error sending frame to encoder");
                }

                if (req.frame == nullptr) { // flushed
                    //this->running = false;
                    //this->scaler_cv.notify_one(); // clear the scaler too
                    break;
                }
                    
                // Convert from OpenCV BGR to encoder's format
                ret = avcodec_receive_packet(this->codec_ctx, pkt);

                if (ret == AVERROR(EAGAIN)) { // The encoder needs more input frames before it can output a packet
                    // No more packets will be produced (flush completed)
                    // log("["+this->toString()+"] Error during receiving frame - EAGAIN");
                    av_packet_free(&pkt);
                    continue;
                } else if (ret == AVERROR_EOF) {
                    log("["+this->toString()+"] Error during receiving frame - AVERROR_EOF");
                    av_packet_free(&pkt);
                    //this->running = false;
                    break;
                } else if (ret < 0) {
                    log("["+this->toString()+"] Error during receiving frame, " + std::to_string(ret), true);
                    av_packet_free(&pkt);
                    //this->running = false;
                    break;
                }

                if (this->packet_callback) {
                    auto frame = std::make_shared<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
                    frame->header = std_msgs::msg::Header();
                    frame->header.frame_id = this->frame_id;
                    frame->header.stamp.sec = req.header.stamp.sec;
                    frame->header.stamp.nanosec = req.header.stamp.nanosec;
                    frame->encoding = "h.264";
                    frame->width = this->width;
                    frame->height = this->height;
                    frame->flags = pkt->flags;
                    frame->is_bigendian = false;
                    frame->pts = 0; // header stamp is used
                    // frame->data.resize(pkt->size);
                    frame->data.assign(pkt->data, pkt->data + pkt->size);
                    this->packet_callback(frame);
                }
                
                av_packet_unref(pkt);
            } catch (...) {
                RCLCPP_ERROR(this->node->get_logger(), "[%s] Exception in encodeWorker", this->toString().c_str());
                //this->running = false;
                //this->scaler_cv.notify_one(); // clear the scaler too
                break;
            }
        }    
        this->encoder_running = false;
        log("["+this->toString()+"] FFmpegEncoder worker finished.");
    }

    FFmpegEncoder::~FFmpegEncoder() {

        log("["+this->toString()+"] Destroying encoder...");

        this->running = false;
        this->scaler_cv.notify_one();
        this->encoder_cv.notify_one();
        while (this->scaler_running || this->encoder_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        this->flushEncoder();
    
        log("["+this->toString()+"] Claning up...");
        // Cleanup
        for (uint i = 0; i < this->num_frame_buffers; i++) {
            if (this->sw_frame_buffers[i])
                av_frame_free(&this->sw_frame_buffers[i]);
            if (this->hw_device_type == AV_HWDEVICE_TYPE_VAAPI && this->hw_frame_buffers[i]) {
                av_frame_free(&this->hw_frame_buffers[i]);
            }
        }
        this->sw_frame_buffers.clear();
        this->hw_frame_buffers.clear();
        
        if (this->codec_ctx) {
            this->codec_ctx->hw_device_ctx = nullptr; //remove this before deallocating
            avcodec_free_context(&this->codec_ctx);
        }
        if (this->hw_device_ctx)
            av_buffer_unref(&this->hw_device_ctx);
        if (this->fmt_ctx)
            avformat_free_context(this->fmt_ctx);
        if (this->sws_ctx)
            sws_freeContext(this->sws_ctx);

        // if (this->node.get() != nullptr)
        //     this->node.reset();

        log("["+this->toString()+"] Cleanup done.");
    }
}