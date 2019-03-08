/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "third_party/winuwp_h264/H264Decoder/H264Decoder.h"

#include <Windows.h>
#include <codecapi.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <ppltasks.h>
#include <robuffer.h>
#include <stdlib.h>
#include <wrl.h>
#include <wrl\implements.h>
#include <iomanip>
#include "../Utils/Utils.h"
#include "common_video/include/video_frame_buffer.h"
#include "libyuv/convert.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfuuid.lib")

namespace webrtc {

//////////////////////////////////////////
// H264 WinUWP Decoder Implementation
//////////////////////////////////////////
/**
 * Pending work [kevin@hart.mn]
 *   - Add logging using RTC logger.
 *   - Switch to NV12 decode format out. Needed to use HW decoder.
 *   - Use consistent return code handling convention (ON_SUCCEEDED for RTC).
 *   - Use rtc C++ coding style (var naming etc).
 *   - Handle stream format changes.
 *   - This code is not thread-safe (like other decoders such as libvpx). Ensure
 * UWP is respecting that.
 *   X Add "needs key frame" member to discard until key frame.
 *   X Check if auto-create output sample is available to reduce allocations
 *     (property of some MFTs). ==> only works for D3D which we don't use.
 *   X Use I420BufferPool to reduce allocations when constructing out buffers.
 *   X Set CODECAPI_AVLowLatencyMode in case Windows 8 (otherwise it fills all
 *       buffers before emitting decoded frames resulting in huge latency
 *   X Invesitgate varient_true to copy attributes from in to out samples:
 *       https://docs.microsoft.com/en-us/windows/desktop/medfound/basic-mft-processing-model#sample-attributes
 *   X Set MFSampleExtension_Discontinuity on sample when receiving
 * "missing_frames"
 *   X Set props for hw accel:
 * https://docs.microsoft.com/en-us/windows/desktop/medfound/h-264-video-decoder
 *   X Call Release on MFT resources explicitly if needed.
 *   X Is there more optimal ConvertToContiguous buffer even when no 2D buffer
 * available?
 *
 * Stretch
 *   - Looks like D3Dbuffer is fastest medium. Try decoding H264 to D3D buffer
 * and use kNative instead of I420.
 *
 * Rate issue debugging
 *   - Try using RTP time directly for sample time.
 *   - Check auto-duration for frames.
 *   - Check if sample time coming out is based after 0 or is same as input.
 *   - Use WMF trace to see sample time info and look for anything weird.
 *   - Try starting samples at 0. Keep initial frame time in class. Try for RTP
 * and NTP timestamps.
 *   - Try same build on both peers. Try Release mode builds.
 *   - Try getting framerate directly from format change.
 *   - Try sending discontinuity flag on first sample.
 *   - Follow codepath to see if WebRTC is discarding.
 *   - Use critical section at beginning of decode to ensure single access to
 * MFT.
 *   - Try setting render_time on decoded frame to see if there's any impact.
 *   - MFSampleExtension_CleanPoint?
 *   - data.frame_ =
 * std::make_unique<webrtc::VideoFrame>(frame.video_frame_buffer(),
 * frame.rotation(), frame.timestamp()); looks wrong in media source
 */

WinUWPH264DecoderImpl::WinUWPH264DecoderImpl()
    : width_(absl::nullopt),
      height_(absl::nullopt),
      decodeCompleteCallback_(nullptr),
      buffer_pool_(false, 300) /* max_number_of_buffers*/ {}

WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl() {
  OutputDebugString(L"WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl()\n");
  Release();
}

HRESULT ConfigureOutputMediaType(ComPtr<IMFTransform> decoder,
                                 GUID media_type, bool *type_found) {
  *type_found = false;

  int type = 0;
  while (true) {
    ComPtr<IMFMediaType> spMediaType;
    HRESULT hr = decoder->GetOutputAvailableType(0, type, &spMediaType);
    if (hr == MF_E_NO_MORE_TYPES)
      return S_OK;

    GUID cur_type;
    ON_SUCCEEDED(spMediaType->GetGUID(MF_MT_SUBTYPE, &cur_type));
    if (FAILED(hr))
      return hr;

    if (cur_type == media_type) {
      hr = decoder->SetOutputType(0, spMediaType.Get(), 0);
      ON_SUCCEEDED(*type_found = true);
      return hr;
    }

    type++;
  }
}

HRESULT CreateInputMediaType(IMFMediaType** pp_input_media, absl::optional<UINT32> img_width,
                             absl::optional<UINT32> img_height, absl::optional<UINT32> frame_rate) {
  HRESULT hr = MFCreateMediaType(pp_input_media);
  
  IMFMediaType* input_media = *pp_input_media;
  ON_SUCCEEDED(input_media->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
  ON_SUCCEEDED(input_media->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264));
  ON_SUCCEEDED(MFSetAttributeRatio(input_media, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));
  ON_SUCCEEDED(input_media->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_MixedInterlaceOrProgressive));

  if (frame_rate.has_value()) {
    ON_SUCCEEDED(MFSetAttributeRatio(input_media, MF_MT_FRAME_RATE, frame_rate.value(), 1));
  }

  if (img_width.has_value() && img_height.has_value()) {
    ON_SUCCEEDED(MFSetAttributeSize(input_media, MF_MT_FRAME_SIZE, img_width.value(), img_height.value()));
  }

  return hr;
}

int WinUWPH264DecoderImpl::InitDecode(const VideoCodec* codec_settings,
                                      int number_of_cores) {
  RTC_LOG(LS_INFO) << "WinUWPH264DecoderImpl::InitDecode()\n";

  width_ = codec_settings->width > 0 ? absl::optional<UINT32>(codec_settings->width) : absl::nullopt;
  height_ = codec_settings->height > 0 ? absl::optional<UINT32>(codec_settings->height) : absl::nullopt;

  HRESULT hr = S_OK;
  ON_SUCCEEDED(CoInitializeEx(0, COINIT_APARTMENTTHREADED));
  ON_SUCCEEDED(MFStartup(MF_VERSION, 0));

  ON_SUCCEEDED(CoCreateInstance(CLSID_MSH264DecoderMFT, nullptr, CLSCTX_INPROC_SERVER,
                                IID_IUnknown, (void**)&m_spDecoder));

  if (FAILED(hr)) {
    RTC_LOG(LS_ERROR) << "Init failure: could not create Media Foundation H264 decoder instance.";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Try set decoder attributes
  ComPtr<IMFAttributes> decoderAttrs;
  ON_SUCCEEDED(m_spDecoder->GetAttributes(decoderAttrs.GetAddressOf()));

  if (SUCCEEDED(hr)) {
    ON_SUCCEEDED(decoderAttrs->SetUINT32(CODECAPI_AVLowLatencyMode, TRUE));
    if (FAILED(hr)) {
      RTC_LOG(LS_WARNING) << "Init warning: failed to set low latency in H264 decoder.";
      hr = S_OK;
    }

    ON_SUCCEEDED(decoderAttrs->SetUINT32(CODECAPI_AVDecVideoAcceleration_H264, TRUE));
    if (FAILED(hr)) {
      RTC_LOG(LS_WARNING) << "Init warning: failed to set HW accel in H264 decoder.";
    }
  }

  // Clear any error from try set attributes
  hr = S_OK;

  ComPtr<IMFMediaType> spInputMedia;
  ON_SUCCEEDED(CreateInputMediaType(spInputMedia.GetAddressOf(), width_, height_,
    codec_settings->maxFramerate > 0 ? absl::optional<UINT32>(codec_settings->maxFramerate) : absl::nullopt));

  if (FAILED(hr)) {
    RTC_LOG(LS_ERROR) << "Init failure: could not create input media type.";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Register the input type with the decoder
  ON_SUCCEEDED(m_spDecoder->SetInputType(0, spInputMedia.Get(), 0));

  if (FAILED(hr)) {
    RTC_LOG(LS_ERROR) << "Init failure: failed to set input media type on decoder.";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Assert MF supports I420 output
  bool suitable_type_found;
  ON_SUCCEEDED(ConfigureOutputMediaType(m_spDecoder, MFVideoFormat_I420, &suitable_type_found));

  if (FAILED(hr) || !suitable_type_found) {
    RTC_LOG(LS_ERROR) << "Init failure: failed to find a valid output media type for decoding.";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  DWORD status;
  ON_SUCCEEDED(m_spDecoder->GetInputStatus(0, &status));

  // Validate that decoder is up and running
  if (SUCCEEDED(hr)) {
    if (MFT_INPUT_STATUS_ACCEPT_DATA != status)
      // H.264 decoder MFT is not accepting data
      return WEBRTC_VIDEO_CODEC_ERROR;
  }

  ON_SUCCEEDED(m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL));
  ON_SUCCEEDED(m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, NULL));
  ON_SUCCEEDED(m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, NULL));

  return SUCCEEDED(hr) ? WEBRTC_VIDEO_CODEC_OK : WEBRTC_VIDEO_CODEC_ERROR;
}

/**
 * Workaround [MF H264 bug: Output status is never set, even when ready]
 *  => For now, always mark "ready" (results in extra buffer alloc/dealloc).
 */
HRESULT GetOutputStatus(ComPtr<IMFTransform> decoder, DWORD* output_status) {
  HRESULT hr = decoder->GetOutputStatus(output_status);

  // Don't MFT trust output status for now.
  *output_status = MFT_OUTPUT_STATUS_SAMPLE_READY;
  return hr;
}

HRESULT WinUWPH264DecoderImpl::FlushFrames(uint32_t rtp_timestamp,
                                           uint64_t ntp_time_ms) {
  HRESULT hr;
  DWORD outputStatus;

  while (SUCCEEDED(hr = GetOutputStatus(m_spDecoder, &outputStatus)) &&
         outputStatus == MFT_OUTPUT_STATUS_SAMPLE_READY) {
    // Get needed size of our output buffer
    MFT_OUTPUT_STREAM_INFO strmInfo;
    ON_SUCCEEDED(m_spDecoder->GetOutputStreamInfo(0, &strmInfo));
    if (FAILED(hr)) {
      RTC_LOG(LS_ERROR) << "Decode failure: failed to get output stream info.";
      return hr;
    }

    // Create output sample
    ComPtr<IMFMediaBuffer> spOutBuffer;
    ON_SUCCEEDED(MFCreateMemoryBuffer(strmInfo.cbSize, &spOutBuffer));
    if (FAILED(hr)) {
      RTC_LOG(LS_ERROR) << "Decode failure: memory buffer creation failed.";
      return hr;
    }

    ComPtr<IMFSample> spOutSample;
    ON_SUCCEEDED(MFCreateSample(&spOutSample));
    if (FAILED(hr)) {
      RTC_LOG(LS_ERROR) << "Decode failure: sample creation failed.";
      return hr;
    }

    ON_SUCCEEDED(spOutSample->AddBuffer(spOutBuffer.Get()));
    if (FAILED(hr)) {
      RTC_LOG(LS_ERROR) << "Decode failure: failed to add buffer to sample.";
      return hr;
    }

    // Create output buffer description
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer;
    outputDataBuffer.dwStatus = 0;
    outputDataBuffer.dwStreamID = 0;
    outputDataBuffer.pEvents = nullptr;
    outputDataBuffer.pSample = spOutSample.Get();

    // Invoke the Media Foundation decoder
    // Note: we don't use ON_SUCCEEDED here since ProcessOutput returns
    //       MF_E_TRANSFORM_NEED_MORE_INPUT often.
    DWORD status;
    hr = m_spDecoder->ProcessOutput(0, 1, &outputDataBuffer, &status);

    if (FAILED(hr))
      return hr; /* can return MF_E_TRANSFORM_NEED_MORE_INPUT or 
                    MF_E_TRANSFORM_STREAM_CHANGE (entirely acceptable) */

    // Copy raw output sample data to video frame buffer.
    ComPtr<IMFMediaBuffer> srcBuffer;
    ON_SUCCEEDED(spOutSample->ConvertToContiguousBuffer(&srcBuffer));
    if (FAILED(hr)) {
      RTC_LOG(LS_ERROR) << "Decode failure: failed to get contiguous buffer.";
      return hr;
    }

    uint32_t width, height;
    if (width_.has_value() && height_.has_value()) {
        width = width_.value();
        height = height_.value();
    } else {
      // Query the size from MF output media type
      ComPtr<IMFMediaType> output_type;
      ON_SUCCEEDED(m_spDecoder->GetOutputCurrentType(0, output_type.GetAddressOf()));

      ON_SUCCEEDED(MFGetAttributeSize(output_type.Get(), MF_MT_FRAME_SIZE, &width, &height));
      if (FAILED(hr)) {
        RTC_LOG(LS_ERROR) << "Decode failure: could not read image dimensions from Media Foundation, so the video frame buffer size can not be determined.";
        return hr;
      }

      width_.emplace(width);
      height_.emplace(height);
    }

    rtc::scoped_refptr<I420Buffer> buffer =
        buffer_pool_.CreateBuffer(width, height);

    if (!buffer.get()) {
        // Pool has too many pending frames.
        RTC_LOG(LS_WARNING) << "WinUWPH264DecoderImpl::FlushFrames(): Too many frames. Dropping frame.";
        return WEBRTC_VIDEO_CODEC_NO_OUTPUT;
    }

    DWORD curLength;
    ON_SUCCEEDED(srcBuffer->GetCurrentLength(&curLength));
    if (FAILED(hr)) {
      RTC_LOG(LS_ERROR) << "Decode failure: could not get buffer length.";
      return hr;
    }
      
    if (curLength > 0) {
      BYTE* pSrcData;
      DWORD maxLen, curLen;
      ON_SUCCEEDED(srcBuffer->Lock(&pSrcData, &maxLen, &curLen));
      if (FAILED(hr)) {
        RTC_LOG(LS_ERROR) << "Decode failure: could lock buffer for copying.";
        return hr;
      }
      
      // TODO: should be the same as curLen, but calculated manually for now
      //       to avoid heap corruption in case unhandled format change results
      //       in too big of a buffer in MF sample.
      int stride_y = width;
      int stride_u = (width + 1) / 2;
      int stride_v = (width + 1) / 2;
      int yuv_size =
          stride_y * height + (stride_u + stride_v) * ((height + 1) / 2);

      memcpy(buffer->MutableDataY(), pSrcData, yuv_size);

      ON_SUCCEEDED(srcBuffer->Unlock());
      if (FAILED(hr))
        return hr;
    }

    // LONGLONG sample_time; /* unused */
    // ON_SUCCEEDED(spOutSample->GetSampleTime(&sample_time));

    // TODO: Ideally, we should convert sample_time (above) back to 90khz + base and use it in place
    //       of rtp_timestamp, since MF may interpolate it. Instead, we ignore the MFT sample
    //       time out, using rtp from in frame that triggered this decoded frame.
    VideoFrame decodedFrame(buffer, rtp_timestamp, 0, kVideoRotation_0);

    // Use ntp time from the earliest frame
    decodedFrame.set_ntp_time_ms(ntp_time_ms);

    // Emit image to downstream
    if (decodeCompleteCallback_ != nullptr) {
      decodeCompleteCallback_->Decoded(decodedFrame, absl::nullopt, absl::nullopt);
    }
  }

  return hr;
}

/**
 * Note: acceptable to return MF_E_NOTACCEPTING (though it shouldn't since
 * last loop should've flushed)
 */
HRESULT WinUWPH264DecoderImpl::EnqueueFrame(const EncodedImage& input_image,
                                            bool missing_frames) {
  // Create a MF buffer from our data
  ComPtr<IMFMediaBuffer> spBuffer;
  HRESULT hr = MFCreateMemoryBuffer(input_image._length, &spBuffer);
  if (FAILED(hr))
    return hr;

  DWORD maxLen, curLen;
  BYTE* pData;
  hr = spBuffer->Lock(&pData, &maxLen, &curLen);
  if (FAILED(hr))
    return hr;

  memcpy(pData, input_image._buffer, input_image._length);

  hr = spBuffer->Unlock();
  if (FAILED(hr))
    return hr;

  hr = spBuffer->SetCurrentLength(input_image._length);
  if (FAILED(hr))
    return hr;

  // Create a sample from media buffer
  ComPtr<IMFSample> spSample;
  hr = MFCreateSample(&spSample);
  if (FAILED(hr))
    return hr;

  hr = spSample->AddBuffer(spBuffer.Get());
  if (FAILED(hr))
    return hr;

  int64_t sampleTimeMs;
  if (first_frame_rtp_ == 0) {
    first_frame_rtp_ = input_image.Timestamp();
    sampleTimeMs = 0;
  } else {
    // Convert from 90 khz, rounding to nearest ms.
    sampleTimeMs = (static_cast<uint64_t>(input_image.Timestamp()) - first_frame_rtp_) / 90.0 + 0.5f;
  }

  hr = spSample->SetSampleTime(sampleTimeMs * 10000 /* convert milliseconds to 100-nanosecond unit */);
  if (FAILED(hr))
    return hr;

  // Set sample attributes
  ComPtr<IMFAttributes> sampleAttributes;
  hr = spSample.As(&sampleAttributes);

  if (FAILED(hr))
    return hr;

  if (input_image._frameType == kVideoFrameKey && input_image._completeFrame) {
    sampleAttributes->SetUINT32(MFSampleExtension_CleanPoint, TRUE);
  }

  if (missing_frames) {
    sampleAttributes->SetUINT32(MFSampleExtension_Discontinuity, TRUE);
  }

  // Enqueue sample with Media Foundation
  hr = m_spDecoder->ProcessInput(0, spSample.Get(), 0);
  return hr;
}

int WinUWPH264DecoderImpl::Decode(const EncodedImage& input_image,
                                  bool missing_frames,
                                  const CodecSpecificInfo* codec_specific_info,
                                  int64_t render_time_ms) {
  HRESULT hr = S_OK;

  // TODO: add additional precondition checks
  if (decodeCompleteCallback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  // Discard until keyframe.
  if (require_keyframe_) {
    if (input_image._frameType != kVideoFrameKey ||
        !input_image._completeFrame) {
      return WEBRTC_VIDEO_CODEC_ERROR;
    } else {
      require_keyframe_ = false;
    }
  }

  // Enqueue the new frame with Media Foundation
  hr = EnqueueFrame(input_image, missing_frames);
  if (hr == MF_E_NOTACCEPTING) {
    // For robustness (shouldn't happen). Flush any old MF data blocking the
    // new frame so we don't drop it. (TODO: request key frame here)
    m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
    hr = EnqueueFrame(input_image, missing_frames);
  }

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  // Flush any decoded samples resulting from new frame, invoking callback
  hr = FlushFrames(input_image.Timestamp(), input_image.ntp_time_ms_);

  if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
    // Output media type is no longer suitable. Reconfigure and retry.
    bool suitable_type_found;
    hr = ConfigureOutputMediaType(m_spDecoder, MFVideoFormat_I420, &suitable_type_found);

    if (FAILED(hr) || !suitable_type_found)
      return WEBRTC_VIDEO_CODEC_ERROR;

    // Reset width and height in case output media size has changed (though it seems that
    // would be unexpected, given that the input media would need to be manually changed too).
    width_.reset();
    height_.reset();

    hr = FlushFrames(input_image.Timestamp(), input_image.ntp_time_ms_);
  }

  if (SUCCEEDED(hr) || hr == MF_E_TRANSFORM_NEED_MORE_INPUT) {
    return WEBRTC_VIDEO_CODEC_OK;
  }

  return WEBRTC_VIDEO_CODEC_ERROR;
}

int WinUWPH264DecoderImpl::RegisterDecodeCompleteCallback(
    DecodedImageCallback* callback) {
  rtc::CritScope lock(&crit_);
  decodeCompleteCallback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::Release() {
  OutputDebugString(L"WinUWPH264DecoderImpl::Release()\n");
  HRESULT hr = S_OK;

  if (m_spDecoder != NULL) {
    ON_SUCCEEDED(m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, 0));
    ON_SUCCEEDED(m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_DRAIN, NULL));
    ON_SUCCEEDED(m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL));
  }

  // TODO: this should only be called if CoInitializeEx succeeded to begin with
  // since these must be balanced.
  if (SUCCEEDED(hr))
    CoUninitialize();

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  return WEBRTC_VIDEO_CODEC_OK;
}

const char* WinUWPH264DecoderImpl::ImplementationName() const {
  return "H264_MediaFoundation";
}

// TODO: safe delete this class from project
// Used to store an encoded H264 sample in a VideoFrame
class H264NativeHandleBuffer : public NativeHandleBuffer {
 public:
  H264NativeHandleBuffer(ComPtr<IMFSample> sample, int width, int height)
      : NativeHandleBuffer(sample.Get(), width, height), _sample(sample) {}

  virtual ~H264NativeHandleBuffer() {}

  rtc::scoped_refptr<I420BufferInterface> ToI420() override { return nullptr; }

 private:
  ComPtr<IMFSample> _sample;
};

}  // namespace webrtc
