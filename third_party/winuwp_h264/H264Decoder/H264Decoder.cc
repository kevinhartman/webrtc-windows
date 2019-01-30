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

// TODO: use consistent return code handling convention (ON_SUCCEEDED for RTC)
// TODO: use rtc C++ coding style (var naming etc)
// TODO: it may be important that the same thread is used when interfacing with
// Media Foundation transform decoder
// TODO: add "needs key frame" member to discard until key frame.
// TODO:check if auto-create output sample is available
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

HRESULT SupportsMediaType(ComPtr<IMFTransform> decoder, GUID media_type, bool* supported) {
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

int WinUWPH264DecoderImpl::InitDecode(const VideoCodec* inst,
  int number_of_cores) {
  RTC_LOG(LS_INFO) << "WinUWPH264DecoderImpl::InitDecode()\n";

  ULONG frameRateNumerator = 30; /* TODO: take framerate from inst: inst->maxFramerate ? */
  ULONG frameRateDenominator = 1;
  ULONG imageWidth = inst->width;
  ULONG imageHeight = inst->height;

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

  // Create input media type
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

// TODO: safe delete this class from project
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

HRESULT CopyOutBuffer2D(uint8_t* pDestData,
                        ComPtr<IMF2DBuffer> src,
                        uint32_t width,
                        uint32_t height) {
  // Get raw pointer to image buffer
  BYTE* pSrcData;
  LONG stride;
  HRESULT hr = src->Lock2D(&pSrcData, &stride);
  if (FAILED(hr))
    return hr;

  // Copy data, ignoring pad.
  for (DWORD row = 0; row < height; row++) {
    memcpy(pDestData, pSrcData, width);
    pDestData += width;
    pSrcData += stride;
  }

  // We got our data. Let it go... let it go.
  hr = src->Unlock2D();
  return hr;
}

HRESULT CopyOutBuffer(uint8_t* pDestData, ComPtr<IMFSample> src) {
  ComPtr<IMFMediaBuffer> srcBuffer;
  HRESULT hr = src->ConvertToContiguousBuffer(&srcBuffer);
  if (FAILED(hr)) {
    return hr;
  }

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

    memcpy(pDestData, pSrcData, curLen);

	hr = srcBuffer->Unlock();
    if (FAILED(hr))
      return hr;
  }

  return S_OK;
}

/** 
 * Workaround [MF H264 bug: Output status is never set, even when ready]
 *  => For now, always mark "ready".
 */
HRESULT GetOutputStatus(ComPtr<IMFTransform> decoder, DWORD* output_status) {
  HRESULT hr = decoder->GetOutputStatus(output_status);

  // Don't trust output status.
  *output_status = MFT_OUTPUT_STATUS_SAMPLE_READY;
  return hr;
}

HRESULT WinUWPH264DecoderImpl::FlushFrames(uint32_t rtp_timestamp, uint64_t ntp_time_ms) {

  HRESULT hr;
  DWORD outputStatus;

  while (SUCCEEDED(hr = GetOutputStatus(m_spDecoder, &outputStatus)) &&
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
    // TODO: if we need to detect stream format changes, read
    // MF_E_TRANSFORM_STREAM_CHANGE here and adjust

    if (FAILED(hr))
      return hr; /* can return MF_E_TRANSFORM_NEED_MORE_INPUT */

	// TODO: libvpx vp8 impl uses I420BufferPool to make these which should reduce allocations.
    rtc::scoped_refptr<I420Buffer> buffer(new rtc::RefCountedObject<I420Buffer>(width_, height_));

	// Copy raw output sample data to video frame buffer.
    ComPtr<IMF2DBuffer> sp2DBuffer;
    hr = spOutBuffer->QueryInterface(IID_IMF2DBuffer, (void**)&sp2DBuffer);
    if (SUCCEEDED(hr)) {
      hr = CopyOutBuffer2D(buffer->MutableDataY(), sp2DBuffer, width_, height_);
    } else {
      // No 2D buffer support.
      hr = CopyOutBuffer(buffer->MutableDataY(), spOutSample);
	}

	if (FAILED(hr))
      return hr;
	
	LONGLONG sampleTime;
    hr = spOutSample->GetSampleTime(&sampleTime);
    if (FAILED(hr))
      return hr;

	// We use the timestamp direct from Media Foundation sample, which will be interpolated properly
	// TODO: should we use rtp overload of this constructor? Hopefully this one can derive it from timestamp_us
    VideoFrame decodedFrame(buffer, kVideoRotation_0, sampleTime / 10 /* convert 100-nanosecond unit to microseconds */);
    
	// Use ntp time from the earliest frame
	decodedFrame.set_ntp_time_ms(ntp_time_ms);

	// Emit image to downstream
    decodeCompleteCallback_->Decoded(decodedFrame, absl::nullopt, absl::nullopt);
  }

  return hr;
}

// TODO: handle missing frames not yet implemented
// TODO: reuse input buffers if possible to save on allocations
// Note: can return MF_E_NOTACCEPTING (though it shouldn't since last loop should've flushed)
HRESULT WinUWPH264DecoderImpl::EnqueueFrame(const EncodedImage& input_image, bool missing_frames) {
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

  hr = spSample->SetSampleTime(input_image.capture_time_ms_ * 10000 /* convert milliseconds to 100-nanosecond unit */);
  if (FAILED(hr))
    return hr;

  // Enqueue sample with Media Foundation
  return m_spDecoder->ProcessInput(0, spSample.Get(), 0);
}

int WinUWPH264DecoderImpl::Decode(const EncodedImage& input_image,
  bool missing_frames,
  const CodecSpecificInfo* codec_specific_info,
  int64_t render_time_ms) {
  HRESULT hr = S_OK;

  // TODO: add more precondition checks
  if (decodeCompleteCallback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  // Enqueue the new frame with Media Foundation
  hr = EnqueueFrame(input_image, missing_frames);
  if (hr == MF_E_NOTACCEPTING) {
    // For robustness (shouldn't happen). Flush any old MF data blocking new frames.
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

  // TODO: this should only be called if CoInitializeEx succeeded to begin with since these must be balanced.
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
