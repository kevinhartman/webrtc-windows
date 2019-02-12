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
 *   - Use consistent return code handling convention (ON_SUCCEEDED for RTC).
 *   - Use rtc C++ coding style (var naming etc).
 *   - Handle stream format changes.
 *   - Use I420BufferPool to reduce allocations when constructing out buffers.
 *   - This code is not thread-safe (like other decoders such as libvpx). Ensure
 * UWP is respecting that.
 *   - Add "needs key frame" member to discard until key frame.
 *   - Check if auto-create output sample is available to reduce allocations
 * (property of some MFTs).
 *   - Set CODECAPI_AVLowLatencyMode in case Windows 8 (otherwise it fills all
 *       buffers before emitting decoded frames resulting in huge latency
 *   - Invesitgate varient_true to copy attributes from in to out samples:
 *       https://docs.microsoft.com/en-us/windows/desktop/medfound/basic-mft-processing-model#sample-attributes
 *   - Set MFSampleExtension_Discontinuity on sample when receiving
 * "missing_frames"
 *   - Set props for hw accel:
 * https://docs.microsoft.com/en-us/windows/desktop/medfound/h-264-video-decoder
 *   - Call Release on MFT resources explicitly if needed.
 *   - Add logging using RTC logger.
 *   - Is there more optimal ConvertToContiguous buffer even when no 2D buffer
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
    : width_(0), height_(0), decodeCompleteCallback_(nullptr) {}

WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl() {
  OutputDebugString(L"WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl()\n");
  Release();
}

HRESULT SupportsMediaType(ComPtr<IMFTransform> decoder,
                          GUID media_type,
                          bool* supported) {
  *supported = false;

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
      *supported = true;
      return hr;
    }

    type++;
  }
}

int WinUWPH264DecoderImpl::InitDecode(const VideoCodec* codec_settings,
                                      int number_of_cores) {
  RTC_LOG(LS_INFO) << "WinUWPH264DecoderImpl::InitDecode()\n";

  // TODO: take framerate from codec_settings: codec_settings->maxFramerate ?
  ULONG frameRateNumerator = 30;
  ULONG frameRateDenominator = 1;
  ULONG imageWidth = codec_settings->width;
  ULONG imageHeight = codec_settings->height;

  width_ = imageWidth;
  height_ = imageHeight;

  HRESULT hr = CoInitializeEx(0, COINIT_APARTMENTTHREADED);
  if (SUCCEEDED(hr))
    hr = MFStartup(MF_VERSION, 0);

  if (SUCCEEDED(hr))
    hr = CoCreateInstance(CLSID_MSH264DecoderMFT, nullptr, CLSCTX_INPROC_SERVER,
                          IID_IUnknown, (void**)&m_spDecoder);

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  // Set decoder attributes
  ComPtr<IMFAttributes> decoderAttrs;
  hr = m_spDecoder->GetAttributes(decoderAttrs.GetAddressOf());

  hr = decoderAttrs->SetUINT32(CODECAPI_AVLowLatencyMode, TRUE);
  if (FAILED(hr)) {
    // TODO: log, continue.
    __debugbreak();
  }

  hr = decoderAttrs->SetUINT32(CODECAPI_AVDecVideoAcceleration_H264, TRUE);
  if (FAILED(hr)) {
    // TODO: log, continue.
    __debugbreak();
  }

  // Create input media type
  ComPtr<IMFMediaType> spInputMedia;
  hr = MFCreateMediaType(&spInputMedia);
  if (SUCCEEDED(hr))
    hr = spInputMedia->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
  if (SUCCEEDED(hr))
    hr = spInputMedia->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spInputMedia.Get(), MF_MT_FRAME_RATE,
                             frameRateNumerator, frameRateDenominator);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeSize(spInputMedia.Get(), MF_MT_FRAME_SIZE, imageWidth,
                            imageHeight);
  if (SUCCEEDED(hr))
    hr =
        MFSetAttributeRatio(spInputMedia.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);

  // Register the input type with the decoder
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetInputType(0, spInputMedia.Get(), 0);

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  // Assert MF supports I420 output
  bool i420_supported;
  hr = SupportsMediaType(m_spDecoder, MFVideoFormat_I420, &i420_supported);

  if (FAILED(hr) || !i420_supported)
    return WEBRTC_VIDEO_CODEC_ERROR;

  // Create output media type
  ComPtr<IMFMediaType> spOutputMedia;
  if (SUCCEEDED(hr))
    hr = MFCreateMediaType(&spOutputMedia);
  if (SUCCEEDED(hr))
    hr = spOutputMedia->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
  if (SUCCEEDED(hr))
    hr = spOutputMedia->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_I420);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spOutputMedia.Get(), MF_MT_FRAME_RATE,
                             frameRateNumerator, frameRateDenominator);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeSize(spOutputMedia.Get(), MF_MT_FRAME_SIZE, imageWidth,
                            imageHeight);

  // Assign output media type to decoder
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetOutputType(0, spOutputMedia.Get(), 0);

  DWORD status;
  if (SUCCEEDED(hr))
    hr = m_spDecoder->GetInputStatus(0, &status);

  // Validate that decoder is up and running
  if (SUCCEEDED(hr)) {
    if (MFT_INPUT_STATUS_ACCEPT_DATA != status)
      // H.264 decoder MFT is not accepting data
      return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, NULL);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, NULL);

  if (SUCCEEDED(hr))
    return WEBRTC_VIDEO_CODEC_OK;

  return WEBRTC_VIDEO_CODEC_ERROR;
}

/**
 * Workaround [MF H264 bug: Output status is never set, even when ready]
 *  => For now, always mark "ready" (results in extra buffer alloc/dealloc).
 *  => Might have no perf impact if H264 MFT supports output buffer
 * auto-creation.
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
    hr = m_spDecoder->GetOutputStreamInfo(0, &strmInfo);
    if (FAILED(hr))
      return hr;
    _ASSERT(!(strmInfo.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES));
    _ASSERT(!(strmInfo.dwFlags & MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES));

    // Create output sample
    ComPtr<IMFMediaBuffer> spOutBuffer;
    hr = MFCreateMemoryBuffer(strmInfo.cbSize, &spOutBuffer);
    if (FAILED(hr))
      return hr;

    // TODO: MF can provide us with sample automatically when using DirectX
    // impl.
    //       We can skip sample creation in that case.
    ComPtr<IMFSample> spOutSample;
    hr = MFCreateSample(&spOutSample);
    if (FAILED(hr))
      return hr;

    hr = spOutSample->AddBuffer(spOutBuffer.Get());
    if (FAILED(hr))
      return hr;

    // Create output buffer description
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer;
    outputDataBuffer.dwStatus = 0;
    outputDataBuffer.dwStreamID = 0;
    outputDataBuffer.pEvents = nullptr;
    outputDataBuffer.pSample = spOutSample.Get();

    // Invoke the Media Foundation decoder
    DWORD status;
    hr = m_spDecoder->ProcessOutput(0, 1, &outputDataBuffer, &status);

    if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
      // TODO: we don't handle this yet
      __debugbreak();
    }

    if (FAILED(hr))
      return hr; /* can return MF_E_TRANSFORM_NEED_MORE_INPUT (entirely
                    acceptable) */

    // Copy raw output sample data to video frame buffer.
    ComPtr<IMFMediaBuffer> srcBuffer;
    HRESULT hr = spOutSample->ConvertToContiguousBuffer(&srcBuffer);
    if (FAILED(hr)) {
      return hr;
    }

    // TODO: libvpx vp8 impl uses I420BufferPool to make these which should
    // reduce allocations.
    rtc::scoped_refptr<I420Buffer> buffer(
        new rtc::RefCountedObject<I420Buffer>(width_, height_));

    DWORD curLength;
    hr = srcBuffer->GetCurrentLength(&curLength);
    if (FAILED(hr))
      return hr;

    if (curLength > 0) {
      BYTE* pSrcData;
      DWORD maxLen, curLen;
      hr = srcBuffer->Lock(&pSrcData, &maxLen, &curLen);
      if (FAILED(hr))
        return hr;

      // TODO: should be the same as curLen, but calculated manually for now
      //       to avoid heap corruption in case unhandled format change results
      //       in too big of a buffer in MF sample.
      int stride_y = width_;
      int stride_u = (width_ + 1) / 2;
      int stride_v = (width_ + 1) / 2;
      int yuv_size =
          stride_y * height_ + (stride_u + stride_v) * ((height_ + 1) / 2);

      memcpy(buffer->MutableDataY(), pSrcData, yuv_size);

      hr = srcBuffer->Unlock();
      if (FAILED(hr))
        return hr;
    }

    LONGLONG sampleTime;
    hr = spOutSample->GetSampleTime(&sampleTime);
    if (FAILED(hr))
      return hr;

    // VideoFrame decodedFrame(
    //    buffer, kVideoRotation_0,
    //    sampleTime / 10 /* convert 100-nanosecond unit to microseconds */);

    // TODO: The commented-out constructor above doesn't seem to work (ntp).
    //       Instead, we ignore the MFT sample time out, using rtp from in
    //       frame that triggered this decoded frame. If we keep this approach,
    //       it may be better to use the rtp timestamp of the earliest frame
    //       that contributed to the decoded frame instead.
    //
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

// TODO: handle missing frames not yet implemented
// TODO: reuse input buffers if possible to save on allocations
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

  hr = spSample->SetSampleTime(
      input_image.ntp_time_ms_ *
      10000 /* convert milliseconds to 100-nanosecond unit */);
  if (FAILED(hr))
    return hr;

  // TODO: set duration explicitly.

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

  // Require timestamps.
  if (input_image.ntp_time_ms_ < 0) {
    return WEBRTC_VIDEO_CODEC_ERROR;
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
  if (FAILED(hr) && hr != MF_E_TRANSFORM_NEED_MORE_INPUT)
    return WEBRTC_VIDEO_CODEC_ERROR;

  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::RegisterDecodeCompleteCallback(
    DecodedImageCallback* callback) {
  rtc::CritScope lock(&crit_);
  decodeCompleteCallback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::Release() {
  // TODO: uninit com and media foundation stuff
  OutputDebugString(L"WinUWPH264DecoderImpl::Release()\n");
  HRESULT hr = S_OK;

  if (m_spDecoder != NULL) {
    if (SUCCEEDED(hr))
      hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, 0);
    if (SUCCEEDED(hr))
      hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_DRAIN, NULL);
    if (SUCCEEDED(hr))
      hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);
  }

  // TODO: this should only be called if CoInitializeEx succeeded to begin with
  // since these must be balanced.
  if (SUCCEEDED(hr))
    CoUninitialize();

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  return WEBRTC_VIDEO_CODEC_OK;
}

// TODO: unused. Kept in case it's useful during stream format change.
void WinUWPH264DecoderImpl::UpdateVideoFrameDimensions(
    const EncodedImage& input_image) {
  auto w = input_image._encodedWidth;
  auto h = input_image._encodedHeight;

  if (input_image._frameType == FrameType::kVideoFrameKey && w > 0 && h > 0) {
    width_ = w;
    height_ = h;
  }
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
