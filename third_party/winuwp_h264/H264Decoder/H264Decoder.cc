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
#include <stdlib.h>
#include <ppltasks.h>
#include <mfapi.h>
#include <robuffer.h>
#include <wrl.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <wrl\implements.h>
#include <iomanip>
#include "../Utils/Utils.h"
#include "libyuv/convert.h"
#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "common_video/include/video_frame_buffer.h"
#include "modules/video_coding/include/video_codec_interface.h"

#pragma comment(lib, "mfreadwrite")
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mfuuid.lib")

namespace webrtc {

//////////////////////////////////////////
// H264 WinUWP Decoder Implementation
//////////////////////////////////////////

// TODO: it may be important that the same thread is used when interfacing with
// Media Foundation transform decoder
// TODO: set CODECAPI_AVLowLatencyMode in case Windows 8 (otherwise it fills all
// buffers before emitting decoded frames resulting in huge latency
// TODO: set varient_true to copy attributes from in to out samples:
// https://docs.microsoft.com/en-us/windows/desktop/medfound/basic-mft-processing-model#sample-attributes
// TODO: set MFSampleExtension_Discontinuity on sample when receiving
// "missing_frames"
// TODO (post hack): looks like D3Dbuffer is fastest medium. Try decoding H264 to D3D buffer and use kNative instead of I420.

WinUWPH264DecoderImpl::WinUWPH264DecoderImpl()
  : width_(0),
  height_(0),
  decodeCompleteCallback_(nullptr) {
}

WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl() {
  OutputDebugString(L"WinUWPH264DecoderImpl::~WinUWPH264DecoderImpl()\n");
  Release();
}

int WinUWPH264DecoderImpl::InitDecode(const VideoCodec* inst,
  int number_of_cores) {
  RTC_LOG(LS_INFO) << "WinUWPH264DecoderImpl::InitDecode()\n";

  HRESULT hr = CoInitializeEx(0, COINIT_APARTMENTTHREADED); // TODO: uninitialize when finished
  if (SUCCEEDED(hr))
    hr = MFStartup(MF_VERSION, 0);

  if (SUCCEEDED(hr))
    hr = CoCreateInstance(CLSID_MSH264DecoderMFT, nullptr, CLSCTX_INPROC_SERVER,
      IID_IUnknown, (void**)&m_spDecoder);

  if (FAILED(hr))
	  // TODO: log hr
    return WEBRTC_VIDEO_CODEC_ERROR;

  // Create input media type
  ULONG frameRateNumerator = 30; /* TODO: take framerate from inst: inst->maxFramerate ? */
  ULONG frameRateDenominator = 1;
  ULONG imageWidth = inst->width;
  ULONG imageHeight = inst->height;

  // TODO: these "SUCCEEDED" macros will all fall through if failure. Ensure this is expected paradigm.
  ComPtr<IMFMediaType> spInputMedia;
  hr = MFCreateMediaType(&spInputMedia);
  if (SUCCEEDED(hr))
    hr = spInputMedia->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
  if (SUCCEEDED(hr))
    hr = spInputMedia->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spInputMedia.Get(), MF_MT_FRAME_RATE, frameRateNumerator,
                             frameRateDenominator);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeSize(spInputMedia.Get(), MF_MT_FRAME_SIZE, imageWidth,
                            imageHeight);
  if (SUCCEEDED(hr))
    hr = MFSetAttributeRatio(spInputMedia.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);

  // Register the input type with the decoder
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetInputType(0, spInputMedia.Get(), 0);


  // TODO: here, we query the list of supported output formats after providing the input stream above
  //       this is a temporary check. Remove this unless it's possible for h264 decoder impl to not support I420 on some devices.
  //int type = 0;
  //while (true) {
  //  ComPtr<IMFMediaType> spMediaType;
  //  hr = m_spDecoder->GetOutputAvailableType(0, type, &spMediaType);
  //  if (hr == MF_E_NO_MORE_TYPES)
  //    return E_FAIL;
  //  GUID mediatype;
  //  if (SUCCEEDED(hr)) {
  //    hr = spMediaType->GetGUID(MF_MT_SUBTYPE, &mediatype);
  //    if (SUCCEEDED(hr)) {
  //      if (mediatype == MFVideoFormat_I420) {
  //        break;
  //      }
  //    }
  //  }
  //  type++;
  //}

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
      return WEBRTC_VIDEO_CODEC_ERROR;  // H.264 decoder MFT is not accepting
                                        // data
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

// Used to store an encoded H264 sample in a VideoFrame
class H264NativeHandleBuffer : public NativeHandleBuffer {
public:
  H264NativeHandleBuffer(ComPtr<IMFSample> sample, int width, int height)
    : NativeHandleBuffer(sample.Get(), width, height)
    , _sample(sample) {
  }

  virtual ~H264NativeHandleBuffer() {
  }

  rtc::scoped_refptr<I420BufferInterface> ToI420() override {
    return nullptr;
  }

private:
  ComPtr<IMFSample> _sample;
};

// TODO: comments for these params
HRESULT WinUWPH264DecoderImpl::FlushFrames(uint32_t rtp_timestamp, uint64_t ntp_time_ms) {

  HRESULT hr;
  DWORD outputStatus;

  while (SUCCEEDED(hr = m_spDecoder->GetOutputStatus(&outputStatus)) &&
         outputStatus == MFT_OUTPUT_STATUS_SAMPLE_READY)
  {
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

	// TODO: MF can potentially provide us one of these for free (check if provided in MF h264 decoder)
	// so in that case, we can skip buffer and sample creation.
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

	// TODO: if we need to detect stream format changes, read MF_E_TRANSFORM_STREAM_CHANGE here and adjust
    if (status == MF_E_TRANSFORM_NEED_MORE_INPUT) {
	  // Ideally, this won't happen (since that'd mean we allocated buffers unnecessarily as per GetOutputStatus())
	  // but there's nothing wrong (we're just out of data), so we return OK.
      return S_OK;
	}

	// Get the sample's buffer.
    ComPtr<IMF2DBuffer> sp2DBuffer;
    hr = spOutBuffer->QueryInterface(IID_IMF2DBuffer, (void**)&sp2DBuffer);
    if (FAILED(hr))
      return hr;

	// Get raw pointer to image buffer
    BYTE* pData;
    LONG stride;
    hr = sp2DBuffer->Lock2D(&pData, &stride);
    if (FAILED(hr))
    return hr;

	// TODO: libvpx vp8 impl uses I420BufferPool to make these which should reduce allocations.
    rtc::scoped_refptr<I420Buffer> buffer(new rtc::RefCountedObject<I420Buffer>(width_, height_));

	// Tiny hack. Use Y pointer to write entire frame.
	BYTE* pI420BufRaw = buffer->MutableDataY();

	// Copy data, ignoring pad. Faster than using sample->ConvertToContiguousBuffer which would
	// result in extra copying since it doesn't write directly into the webrtc buffer like we can.
    // TODO: use libyuv::copy if needed (if MF YUV I420 format is somehow different from WebRTC's)
	for (DWORD row = 0; row < height_; row++) {
      memcpy(pI420BufRaw, pData, width_);
      pI420BufRaw += width_;
      pData += stride;
	}

	// We got our data. Let it go... let it go.
	hr = sp2DBuffer->Unlock2D();
    if (FAILED(hr))
      return hr;

	LONGLONG sampleTime;
    hr = spOutSample->GetSampleTime(&sampleTime);
    if (FAILED(hr))
      return hr;

	// We use the timestamp direct from Media Foundation sample, which will be interpolated properly
	// Note: rtp_timestamp is unused, but could be forwarded from earliest frame as we do below for ntp
    VideoFrame decodedFrame(buffer, kVideoRotation_0, sampleTime / 10 /* convert 100-nanosecond unit to microseconds */);
    
	// Use ntp time from the earliest frame
	decodedFrame.set_ntp_time_ms(ntp_time_ms);

	// Emit image to downstream
    decodeCompleteCallback_->Decoded(decodedFrame, absl::nullopt, absl::nullopt);
  }

  return hr;
}

// TODO: handle missing frames not yet implemented
HRESULT WinUWPH264DecoderImpl::EnqueueFrame(const EncodedImage& input_image, bool missing_frames) {
  
  // Create a MF buffer from our data
  // TODO: reuse input buffers if possible to save on allocations
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
  hr = spSample->SetSampleTime(input_image.capture_time_ms_ * 10000 /* convert milliseconds to 100-nanosecond unit */);
  if (FAILED(hr))
    return hr;

  // Enqueue sample with Media Foundation
  hr = m_spDecoder->ProcessInput(0, spSample.Get(), 0);
  if (FAILED(hr)) {
	// TODO: log here especially. This would mean MF couldn't accept the sample, so we will drop it.
	// We attempt to ensure this won't happen by flushing any potentially pending output before enqueuing.
    return hr;
  }

  return S_OK;
}

int WinUWPH264DecoderImpl::Decode(const EncodedImage& input_image,
  bool missing_frames,
  const CodecSpecificInfo* codec_specific_info,
  int64_t render_time_ms) {

  // TODO: add more precondition checks
  if (decodeCompleteCallback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  DWORD inputStatus;
  HRESULT hr = m_spDecoder->GetInputStatus(0, &inputStatus);

  if (FAILED(hr)) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  if (inputStatus != MFT_INPUT_STATUS_ACCEPT_DATA) {
	// Process output until the stream can accept the new frame.
	// Use last frame times (TODO: this may not be sufficient)
    int res = FlushFrames(lastRtpTimestamp_, lastNtpTimeMs_);
    if (FAILED(hr))
      return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Save ntp in case MF has residual output data from this frame on the next one
  lastRtpTimestamp_ = input_image.Timestamp;
  lastNtpTimeMs_ = input_image.ntp_time_ms_;

  // Enqueue the new frame with Media Foundation
  hr = EnqueueFrame(input_image, missing_frames);
  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  // Flush any decoded resulting from this new frame, and invoke callback
  hr = FlushFrames(input_image.Timestamp, input_image.ntp_time_ms_);
  if (FAILED(hr))
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

  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, 0);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_DRAIN, NULL);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, NULL);

  if (SUCCEEDED(hr))
    CoUninitialize();

  if (FAILED(hr))
    return WEBRTC_VIDEO_CODEC_ERROR;

  return WEBRTC_VIDEO_CODEC_OK;
}

void WinUWPH264DecoderImpl::UpdateVideoFrameDimensions(const EncodedImage& input_image)
{
  auto w = input_image._encodedWidth;
  auto h = input_image._encodedHeight;

  if (input_image._frameType == FrameType::kVideoFrameKey && w > 0 && h > 0)
  {
    width_ = w;
    height_ = h;
  }
}

const char* WinUWPH264DecoderImpl::ImplementationName() const {
  return "H264_MediaFoundation";
}

}  // namespace webrtc
