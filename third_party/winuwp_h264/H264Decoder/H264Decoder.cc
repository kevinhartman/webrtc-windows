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
  
  // TODO: may need to do Windows::Foundation::Initialize or CoInitializeEx first
  // TODO: assuming this is called only at begin streaming since it's probably expensive.

  HRESULT hr = CoCreateInstance(CLSID_MSH264DecoderMFT, nullptr, CLSCTX_INPROC_SERVER,
      IID_IUnknown, (void**)&m_spDecoder);
  if (FAILED(hr))
	  // TODO: log hr
    return WEBRTC_VIDEO_CODEC_ERROR;

  //**********************************************************************
  // Create input media type
  //**********************************************************************
  ULONG frameRateNumerator = 30; /* inst->maxFramerate ? */
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

  //**********************************************************************
  // Create output media type
  //**********************************************************************
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

  //**********************************************************************
  // Assign media types to decoder
  //**********************************************************************
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetInputType(0, spInputMedia.Get(), 0);
  if (SUCCEEDED(hr))
    hr = m_spDecoder->SetOutputType(0, spOutputMedia.Get(), 0);

  DWORD status;
  if (SUCCEEDED(hr))
    hr = m_spDecoder->GetInputStatus(0, &status);

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

ComPtr<IMFSample> FromEncodedImage(const EncodedImage& input_image) {
  HRESULT hr = S_OK;

  ComPtr<IMFSample> sample;
  hr = MFCreateSample(&sample);

  ComPtr<IMFMediaBuffer> mediaBuffer;
  ON_SUCCEEDED(MFCreateMemoryBuffer((DWORD)input_image._length, &mediaBuffer));

  BYTE* destBuffer = NULL;
  if (SUCCEEDED(hr)) {
    DWORD cbMaxLength;
    DWORD cbCurrentLength;
    hr = mediaBuffer->Lock(&destBuffer, &cbMaxLength, &cbCurrentLength);
  }

  if (SUCCEEDED(hr)) {
    memcpy(destBuffer, input_image._buffer, input_image._length);
  }

  ON_SUCCEEDED(mediaBuffer->SetCurrentLength((DWORD)input_image._length));
  ON_SUCCEEDED(mediaBuffer->Unlock());

  ON_SUCCEEDED(sample->AddBuffer(mediaBuffer.Get()));

  return sample;
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

int WinUWPH264DecoderImpl::Decode(const EncodedImage& input_image,
  bool missing_frames,
  const CodecSpecificInfo* codec_specific_info,
  int64_t render_time_ms) {

  UpdateVideoFrameDimensions(input_image);
  auto sample = FromEncodedImage(input_image);

  if (sample != nullptr) {
    rtc::scoped_refptr<VideoFrameBuffer> buffer(new rtc::RefCountedObject<H264NativeHandleBuffer>(
      sample, width_, height_));
    VideoFrame decodedFrame(buffer, input_image.Timestamp(), render_time_ms, kVideoRotation_0);
    decodedFrame.set_ntp_time_ms(input_image.ntp_time_ms_);

    rtc::CritScope lock(&crit_);

    if (decodeCompleteCallback_ != nullptr) {
      decodeCompleteCallback_->Decoded(decodedFrame);
    }
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::RegisterDecodeCompleteCallback(
  DecodedImageCallback* callback) {
  rtc::CritScope lock(&crit_);
  decodeCompleteCallback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int WinUWPH264DecoderImpl::Release() {
  OutputDebugString(L"WinUWPH264DecoderImpl::Release()\n");
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
