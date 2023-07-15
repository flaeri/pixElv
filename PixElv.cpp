#include <d3d11.h>
#include <dxgi1_4.h>
#include <dxgi.h>
#include <d3dkmthk.h>
#include <d3d9.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
#include <future>
#include <mfidl.h>
#include <cstring> // for params
#include <cstdlib> // for params
#include <map> // for params
#include <filesystem>
#include <algorithm> // maffs
#include <csignal> // ctrl+c
#include <atomic>

//MF
#include <mfapi.h>
#include <atlbase.h>
#include <mfreadwrite.h>
#include <windows.h>
#include <atlcomcli.h>
#include <mferror.h>
#include <codecapi.h>
#include <strmif.h>
#include <wmcodecdsp.h>
#include <mfobjects.h>

#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "Kernel32.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "wmcodecdspuuid.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "d3d11.lib")


#define CHECK_HR(hr) { if (FAILED(hr)) { std::cerr << "Failed at line " << __LINE__ << std::endl; return {}; } }

std::mutex mtx;
std::condition_variable cv;
bool finished = false;
std::atomic<bool> runFlag{ true };

BOOL WINAPI consoleCtrlHandler(DWORD ctrlType) {
    switch (ctrlType) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
        finished = true;
        return TRUE;
    default:
        return FALSE;
    }
}

struct FrameData {
    unsigned int rowPitch;
    unsigned int depthPitch;
    unsigned char* data;  // The actual frame data, copied from GPU memory
};

class FrameQueue {
private:
    std::queue<FrameData> queue;
    std::size_t max_size;

public:
    FrameQueue(std::size_t max_size) : max_size(max_size) {}

    bool isFull() const {
        return queue.size() >= max_size;
    }

    void pushFrame(const D3D11_MAPPED_SUBRESOURCE& frameData) {
        // Create a new buffer and copy the frame data into it
        unsigned char* buffer = new unsigned char[frameData.DepthPitch];
        memcpy(buffer, frameData.pData, frameData.DepthPitch);

        // Create a FrameData instance
        FrameData fd;
        fd.rowPitch = frameData.RowPitch;
        fd.depthPitch = frameData.DepthPitch;
        fd.data = buffer;

        // Add the buffer to the queue
        queue.push(fd);
    }

    FrameData popFrame() {
        if (queue.empty()) {
            // No frames available
            std::cerr << "NO FRAMES LEFT IN QUEUE!!!";
        }

        // Get the oldest frame from the queue
        FrameData fd = queue.front();
        queue.pop();

        return fd;
    }

    bool empty() const {
        return queue.empty();
    }

    size_t size() {
        return queue.size();
    }
};

struct DxgiResources {
    ID3D11Device* pDevice;
    ID3D11DeviceContext* pContext;
    IDXGIOutputDuplication* duplication;
    DXGI_OUTDUPL_FRAME_INFO frameInfo;
    IDXGIResource* desktopResource;
    ID3D11Texture2D* frame;
    D3D11_TEXTURE2D_DESC desc;
    ID3D11Texture2D* pDebugTexture;
    D3D11_MAPPED_SUBRESOURCE mappedResource;
};

void EnumerateEncoders() {
    UINT32 count = 0;
    IMFActivate** ppActivate = NULL;

    HRESULT hr = MFTEnumEx(
        MFT_CATEGORY_VIDEO_ENCODER,
        MFT_ENUM_FLAG_SYNCMFT | MFT_ENUM_FLAG_ASYNCMFT | MFT_ENUM_FLAG_HARDWARE,
        NULL,   // Input type
        NULL,   // Output type
        &ppActivate,
        &count
    );

    if (SUCCEEDED(hr)) {
        for (UINT32 i = 0; i < count; i++) {
            UINT32 len = 0;
            WCHAR* name = NULL;

            hr = ppActivate[i]->GetAllocatedString(MFT_FRIENDLY_NAME_Attribute, &name, &len);
            if (SUCCEEDED(hr)) {
                std::wcout << L"Found encoder: " << name << std::endl;
                CoTaskMemFree(name);
            }
        }
        CoTaskMemFree(ppActivate);
    }
}

DxgiResources initializeDxgi(int monitorIndex) {
    DxgiResources resources;

    ID3D11Device* pDevice = NULL;
    ID3D11DeviceContext* pContext = NULL;
    D3D_FEATURE_LEVEL featureLevel;
    HRESULT hr;

    hr = D3D11CreateDevice(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, NULL, 0, D3D11_SDK_VERSION, &(resources.pDevice), &featureLevel, &(resources.pContext));
    CHECK_HR(hr);

    IDXGIDevice* dxgiDevice = NULL;
    hr = resources.pDevice->QueryInterface(__uuidof(IDXGIDevice), (void**)&dxgiDevice);
    CHECK_HR(hr);

    hr = dxgiDevice->SetGPUThreadPriority(2);
    CHECK_HR(hr);

    IDXGIAdapter* adapter = NULL;
    hr = dxgiDevice->GetAdapter(&adapter);
    CHECK_HR(hr);

    NTSTATUS status = D3DKMTSetProcessSchedulingPriorityClass(GetCurrentProcess(), D3DKMT_SCHEDULINGPRIORITYCLASS_HIGH);
    if (status != 0) {
        wprintf(L"GPU prio FAIL!\n");
    }

    DXGI_ADAPTER_DESC adapterDesc;
    adapter->GetDesc(&adapterDesc);
    wprintf(L"GPU: %s\n", adapterDesc.Description);

    IDXGIOutput* output = NULL;
    hr = adapter->EnumOutputs(monitorIndex, &output);
    CHECK_HR(hr);

    DXGI_OUTPUT_DESC outputDesc;
    output->GetDesc(&outputDesc);
    wprintf(L"Display: %s\n", outputDesc.DeviceName);

    IDXGIOutput1* output1 = NULL;
    hr = output->QueryInterface(__uuidof(IDXGIOutput1), (void**)&output1);
    CHECK_HR(hr);

    IDXGIOutputDuplication* duplication = NULL;
    hr = output1->DuplicateOutput(resources.pDevice, &(resources.duplication));
    CHECK_HR(hr);

    IDXGIResource* desktopResource = NULL;
    hr = resources.duplication->AcquireNextFrame(INFINITE, &(resources.frameInfo), &(resources.desktopResource));
    CHECK_HR(hr);

    ID3D11Texture2D* frame = NULL;
    hr = resources.desktopResource->QueryInterface(__uuidof(ID3D11Texture2D), (void**)&(resources.frame));
    CHECK_HR(hr);

    resources.frame->GetDesc(&(resources.desc));
    resources.desc.Usage = D3D11_USAGE_STAGING;
    resources.desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    resources.desc.BindFlags = 0;
    resources.desc.MiscFlags = 0;
    hr = resources.pDevice->CreateTexture2D(&(resources.desc), NULL, &(resources.pDebugTexture));
    CHECK_HR(hr);

    resources.duplication->ReleaseFrame();
    resources.pContext->Unmap(resources.pDebugTexture, 0);

    return resources;
}

void printError(const char* functionName, HRESULT hr)
{
    LPVOID lpMsgBuf;
    DWORD dw = FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        hr,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR)&lpMsgBuf,
        0, NULL);

    std::cerr << functionName << " failed with error " << hr << ": " << (LPTSTR)lpMsgBuf << std::endl;

    LocalFree(lpMsgBuf);
}

bool acquireFrame(DxgiResources& resources) {
    // 34ms timeout essentially came from 60fps, 17*2
    HRESULT hr = resources.duplication->AcquireNextFrame(34, &(resources.frameInfo), &(resources.desktopResource));

    // handle returns of just cursor updates.
    if (resources.frameInfo.AccumulatedFrames == 0 || resources.frameInfo.LastPresentTime.QuadPart == 0) {
        // cursor only, reject
        resources.duplication->ReleaseFrame();
        return false;
    }

    if (hr == S_OK) {
        return true;
    }
    else if (hr == DXGI_ERROR_WAIT_TIMEOUT) {
        std::cerr << "No new frame is available.\n";
        return false;
    }
    else if (hr == DXGI_ERROR_ACCESS_LOST) {
        std::cerr << "\nThe desktop duplication interface is invalid! Usually because UAC/admin, sleep, resolution, hz changes etc.\n";
        std::cerr << "Early exit, sorry :( \n";

        finished = true;
        return false;
    }
    else {
        std::cerr << "An error occurred while acquiring the next frame: " << std::hex << hr << "\n";
        return false;
    }
}

int framesWritten = 0;
int maxQueueSize = 15;
FrameQueue frameQueue(maxQueueSize);

bool isFirstSample = true;

unsigned char* convertARGBtoRGB(unsigned char* argbData, int width, int height) {
    unsigned char* rgbData = new unsigned char[width * height * 3];

    for (int j = 0; j < width * height; j++) {
        rgbData[j * 3] = argbData[j * 4];     // Red
        rgbData[j * 3 + 1] = argbData[j * 4 + 1]; // Green
        rgbData[j * 3 + 2] = argbData[j * 4 + 2]; // Blue
    }

    return rgbData;
}

unsigned char* convertARGBtoRGBFlipped(unsigned char* argbData, int width, int height) {
    unsigned char* rgbData = new unsigned char[width * height * 3];

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            // Compute the index in the destination image
            int dstIndex = ((height - j - 1) * width + i) * 3;

            // Compute the index in the source image
            int srcIndex = (j * width + i) * 4;

            // Copy the RGB values
            rgbData[dstIndex] = argbData[srcIndex];     // Red
            rgbData[dstIndex + 1] = argbData[srcIndex + 1]; // Green
            rgbData[dstIndex + 2] = argbData[srcIndex + 2]; // Blue
        }
    }

    return rgbData;
}

LONGLONG llOutputSampleTime{};
LONGLONG llOutputSampleDuration{};

std::chrono::duration<double, std::milli> maxTime(0); // variable to keep track of the maximum time
std::chrono::high_resolution_clock::time_point startTime; // variable to keep track of the start time

void start_timer() {
    // Get the current time
    startTime = std::chrono::high_resolution_clock::now();
}

void stop_timer() {
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(stopTime - startTime);

    // If the elapsed time is greater than the maximum time, update maxTime
    if (elapsedTime > maxTime) { maxTime = elapsedTime; }

    // If frames is a multiple of 60, print maxTime
    if (framesWritten % 60 == 0) {
        std::cout << "avgMax fGrab (ms): " << maxTime.count() << " TS: " << framesWritten / 60 << std::endl;
        // Reset maxTime after printing
        maxTime = std::chrono::duration<double, std::milli>(0);
    }
}

void captureFrames(DxgiResources& resources, int runFor) {
    int framesCaptured = 0;
    while (framesCaptured < runFor) {
        if (acquireFrame(resources)) {
            start_timer();
            resources.pContext->CopyResource(resources.pDebugTexture, resources.frame);
            resources.duplication->ReleaseFrame();

            HRESULT hr = resources.pContext->Map(resources.pDebugTexture, 0, D3D11_MAP_READ, 0, &resources.mappedResource);
            if (FAILED(hr)) {
                std::cout << hr;
            }

            resources.pContext->Unmap(resources.pDebugTexture, 0);

            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [] { return !frameQueue.isFull(); });
            frameQueue.pushFrame(resources.mappedResource);
            cv.notify_all();

            framesCaptured++;
            stop_timer();
            if (finished) { return; }
        }
        if (finished) { return; }
    }

    std::cout << "Total frames captured: " << framesCaptured << std::endl;
}

void writeFrameToDisk(FrameData frameData, DxgiResources& resources, IMFSinkWriter* pSinkWriter, IMFTransform* pTransform, IMFTransform* pEncoder, DWORD streamIndex, uint64_t framerate, bool isCompressed) {
    HRESULT hr;
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer{};

    // Convert ARGB to RGB
    unsigned char* rgbData = NULL;
    if (isCompressed) {
        rgbData = convertARGBtoRGBFlipped(frameData.data, resources.desc.Width, resources.desc.Height);
    }
    else {
        rgbData = convertARGBtoRGB(frameData.data, resources.desc.Width, resources.desc.Height);
    }

    CComPtr<IMFMediaBuffer> pInputBuffer;
    hr = MFCreateMemoryBuffer(resources.desc.Width * resources.desc.Height * 3, &pInputBuffer);
    if (FAILED(hr)) {
        std::cerr << "Failed to create memory buffer";
        return;
    }

    BYTE* pBytes = NULL;
    DWORD maxLength = 0, currentLength = 0;
    hr = pInputBuffer->Lock(&pBytes, &maxLength, &currentLength);
    if (FAILED(hr)) {
        std::cerr << "Failed to lock buffer";
        return;
    }

    std::memcpy(pBytes, rgbData, resources.desc.Width * resources.desc.Height * 3);
    delete[] rgbData;

    hr = pInputBuffer->Unlock();
    if (FAILED(hr)) {
        std::cerr << "Failed to unlock buffer";
        return;
    }

    hr = pInputBuffer->SetCurrentLength(resources.desc.Width * resources.desc.Height * 3);
    if (FAILED(hr)) {
        std::cerr << "Failed to set buffer length";
        return;
    }

    // Input
    CComPtr<IMFSample> pInputSample;
    hr = MFCreateSample(&pInputSample);
    if (FAILED(hr)) {
        std::cerr << "Failed to create sample";
        return;
    }

    hr = pInputSample->AddBuffer(pInputBuffer);
    if (FAILED(hr)) {
        std::cerr << "Failed to add buffer to sample";
        return;
    }

    // output
    CComPtr<IMFSample> pOutputSample;
    // Prepare an output sample to receive the transformed data
    hr = MFCreateSample(&pOutputSample);
    if (FAILED(hr)) {
        std::cerr << "Failed to create output sample";
        return;
    }

    // And an output buffer for that sample
    CComPtr<IMFMediaBuffer> pOutputBuffer;
    hr = MFCreateMemoryBuffer(resources.desc.Width * resources.desc.Height * 2, &pOutputBuffer); // YUY2 is 2 bytes per pixel
    if (FAILED(hr)) {
        std::cerr << "Failed to create output buffer";
        return;
    }

    hr = pOutputSample->AddBuffer(pOutputBuffer);
    if (FAILED(hr)) {
        std::cerr << "Failed to add output buffer to sample";
        return;
    }

    if (isFirstSample) {
        // Convert frame rate to frame duration in seconds
        double frameDurationSec = 1.0 / framerate;

        // Convert frame duration from seconds to nanosecond units
        LONGLONG llSampleDuration = static_cast<LONGLONG>(frameDurationSec * 10.0 * 1000 * 1000);
        LONGLONG llSampleTime = static_cast<LONGLONG>(framesWritten * frameDurationSec * 10.0 * 1000 * 1000);

        hr = pInputSample->SetSampleTime(llSampleTime);
        if (FAILED(hr)) {
            std::cerr << "Failed to set sample time";
            return;
        }

        hr = pInputSample->SetSampleDuration(llSampleDuration);
        if (FAILED(hr)) {
            std::cerr << "Failed to set input sample duration";
            return;
        }

        // Set the initial timestamp and duration for the first sample
        hr = pOutputSample->SetSampleTime(llSampleTime);
        if (FAILED(hr)) {
            std::cerr << "Failed to set sample time";
            return;
        }

        hr = pOutputSample->SetSampleDuration(llSampleDuration);
        if (FAILED(hr)) {
            std::cerr << "Failed to set sample duration";
            return;
        }
    }

    if (isCompressed) {
        if (!isFirstSample) {

            // Set the input sample timestamp and duration based on the output sample values
            hr = pInputSample->SetSampleTime(llOutputSampleTime);
            if (FAILED(hr)) {
                std::cerr << "Failed to set sample time";
                return;
            }

            hr = pInputSample->SetSampleDuration(llOutputSampleDuration);
            if (FAILED(hr)) {
                std::cerr << "Failed to set sample duration";
                return;
            }
        }
        isFirstSample = false;  // Reset flag after setting initial values

        outputDataBuffer = { 0, pOutputSample, 0, NULL };
        DWORD status;

        // first transform, RGB to YUY2

        // you MUST use ProcessOutput to ask pTransform if its empty. Otherwise its going to think it still has data.
        hr = pTransform->ProcessOutput(0, 1, &outputDataBuffer, &status);
        if (FAILED(hr) && hr != MF_E_TRANSFORM_NEED_MORE_INPUT) {
            std::cerr << "Failed at line " << __LINE__ << std::endl;
            return;
        }

        do {
            hr = pTransform->ProcessInput(0, pInputSample, 0);
            if (hr == MF_E_NOTACCEPTING) {
                // Not ready for input, attempt to process output
                hr = pTransform->ProcessOutput(0, 1, &outputDataBuffer, &status);
                if (FAILED(hr)) {
                    std::cerr << "Failed at line " << __LINE__ << std::endl;
                    return;
                }
            }
        } while (hr == MF_E_NOTACCEPTING);

        // Get the transformed data
        hr = pTransform->ProcessOutput(0, 1, &outputDataBuffer, &status);
        if (FAILED(hr)) {
            std::cerr << "Failed at line " << __LINE__ << std::endl;
            //return;
        }

        // second transform, YUY2/NV12 to H264
        IMFSample* pSecondInputSample = outputDataBuffer.pSample;

        hr = pEncoder->ProcessOutput(0, 1, &outputDataBuffer, &status);
        if (FAILED(hr) && hr != MF_E_TRANSFORM_NEED_MORE_INPUT) {
            std::cerr << "Failed at line " << __LINE__ << std::endl;
            return;
        }

        do {
            hr = pEncoder->ProcessInput(0, pSecondInputSample, 0);
            if (hr == MF_E_NOTACCEPTING) {
                // Not ready for input, attempt to process output
                hr = pEncoder->ProcessOutput(0, 1, &outputDataBuffer, &status);
                if (FAILED(hr)) {
                    std::cerr << "Failed at line " << __LINE__ << std::endl;
                    return;
                }
            }
        } while (hr == MF_E_NOTACCEPTING);

        hr = pEncoder->ProcessOutput(0, 1, &outputDataBuffer, &status);
        if (FAILED(hr)) {
            std::cerr << "Failed at line " << __LINE__ << " | ";
            //return; //dont worry about it :(
        }


        if (!isFirstSample) {
            // Get the timestamp and duration from the output sample

            hr = outputDataBuffer.pSample->GetSampleDuration(&llOutputSampleDuration);
            if (FAILED(hr)) {
                std::cerr << "Failed to get sample duration";
                return;
            }

            llOutputSampleTime = llOutputSampleTime + llOutputSampleDuration;
        }

        // outputDataBuffer.pSample now holds the output sample
        pOutputSample = outputDataBuffer.pSample;

        // Dont care, release
        if (outputDataBuffer.pEvents) {
            outputDataBuffer.pEvents->Release();
        }
    }
    else {
        pOutputSample = pInputSample;
    }

    // Write the sample to disk
    hr = pSinkWriter->WriteSample(streamIndex, pOutputSample);
    if (FAILED(hr)) {
        std::cerr << "Failed to write sample" << std::endl;
        return;
    }

    delete[] frameData.data;
}

void writeFrames(DxgiResources& resources, IMFSinkWriter* pSinkWriter, DWORD streamIndex, uint64_t framerate, bool isCompressed, IMFTransform* pTransform, IMFTransform* pEncoder) {
    while (!finished || !frameQueue.empty()) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [] {return !frameQueue.empty(); });
        FrameData frameData = frameQueue.popFrame();
        cv.notify_all();
        writeFrameToDisk(frameData, resources, pSinkWriter, pTransform, pEncoder, streamIndex, framerate, isCompressed);
        framesWritten++;
    }
}

void countdown(int seconds) {
    for (int i = seconds; i > 0; --i) {
        std::cout << i << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Go!\n" << std::endl;
}

bool generateOutputPath(const std::string& pathArg, bool isCompressed, std::filesystem::path& outputPath) {
    outputPath = pathArg;

    // validate obvious stuff
    if (isCompressed && outputPath.extension() == ".avi") {
        std::cerr << "Error: Invalid extension '.avi' for compressed output, please use .mp4 or .mov etc.\n";
        return false;
    }

    if (!isCompressed && (outputPath.extension() == ".mp4" || outputPath.extension() == ".mov")) {
        std::cerr << "Error: Invalid extension '" + outputPath.extension().string() + "' for uncompressed output, please use .avi\n";
        return false;
    }

    // If a path argument is given, use it directly
    if (!pathArg.empty()) {
        outputPath = pathArg;
    }
    else {
        // Otherwise, generate a filename in the current directory
        char filename[32];
        if (isCompressed) {
            sprintf_s(filename, "output.mp4");
        }
        else {
            sprintf_s(filename, "output.avi");
        }
        outputPath = std::filesystem::current_path() / filename;
    }

    return true;
}


std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
    std::map<std::string, std::string> arguments;

    for (int i = 1; i < argc; i += 2) {
        if (i + 1 < argc) {
            arguments[argv[i]] = argv[i + 1];
        }
        else {
            arguments[argv[i]] = "";
        }
    }

    return arguments;
}

CComPtr<IMFSinkWriter> setupSinkWriter(const std::wstring& outputFilePath, bool isCompressed) {
    HRESULT hr;

    CComPtr<IMFAttributes> pAttributes;
    hr = MFCreateAttributes(&pAttributes, 1);
    if (FAILED(hr))
    {
        std::cerr << "Failed to create attributes";
        return nullptr;
    }

    hr = pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set attribute";
        return nullptr;
    }

    IMFSinkWriter* pSinkWriter = nullptr;
    hr = MFCreateSinkWriterFromURL(outputFilePath.c_str(), NULL, pAttributes, &pSinkWriter);
    if (FAILED(hr))
    {
        std::cerr << "Failed to create sink writer";
        return nullptr;
    }

    return pSinkWriter;
}

CComPtr<IMFMediaType> setupMediaType(int width, int height, int pixfmt, int framerate) {
    HRESULT hr;
    GUID format;

    // Choose format
    switch (pixfmt) {
    case 0:
        format = MFVideoFormat_RGB24;
        break;
    case 1:
        format = MFVideoFormat_YUY2; // YUY2 or NV12
        break;
    case 2:
        format = MFVideoFormat_H264;
        break;
    default:
        std::cerr << "Invalid pixel format specified";
        return nullptr;
    }

    CComPtr<IMFMediaType> pMediaTypeOut;
    hr = MFCreateMediaType(&pMediaTypeOut);
    if (FAILED(hr))
    {
        std::cerr << "Failed to create media type";
        return nullptr;
    }

    hr = pMediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set MF_MT_MAJOR_TYPE\n";
        return nullptr;
    }

    hr = pMediaTypeOut->SetGUID(MF_MT_SUBTYPE, format);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set MF_MT_SUBTYPE\n";
        return nullptr;
    }

    hr = pMediaTypeOut->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set MF_MT_INTERLACE_MODE\n";
        return nullptr;
    }

    hr = MFSetAttributeSize(pMediaTypeOut, MF_MT_FRAME_SIZE, width, height);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set MF_MT_FRAME_SIZE\n";
        return nullptr;
    }

    hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_FRAME_RATE, framerate, 1);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set MF_MT_FRAME_RATE\n";
        return nullptr;
    }

    hr = MFSetAttributeRatio(pMediaTypeOut, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set MF_MT_PIXEL_ASPECT_RATIO\n";
        return nullptr;
    }

    return pMediaTypeOut;
}

HRESULT setupEncoder(IMFTransform** ppEncoder) {
    // Create the H.264 encoder MFT.
    HRESULT hr = CoCreateInstance(
        CLSID_CMSH264EncoderMFT,
        NULL,
        CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(ppEncoder)
    );
    if (FAILED(hr)) {
        std::cerr << "Failed to create encoder MFT";
        return hr;
    }

    return S_OK;
}

HRESULT configureEncoderQuality(IMFTransform* pEncoder, int qp, int quality) {
    // Configure the H.264 encoder as desired
    CComQIPtr<ICodecAPI> pCodecAPI(pEncoder);
    if (!pCodecAPI)
    {
        std::cerr << "Failed to get ICodecAPI interface from encoder object." << std::endl;
        return E_NOINTERFACE;
    }

    // Define VARIANT to set the desired value
    VARIANT var{};;
    var.vt = VT_UI4;

    var.ulVal = eAVEncCommonRateControlMode_Quality;
    HRESULT hr = pCodecAPI->SetValue(&CODECAPI_AVEncCommonRateControlMode, &var);
    if (FAILED(hr))
    {
        std::cerr << "Failed to set encoder rate control mode to quality." << std::endl;
        return hr;
    }

    var.ulVal = UINT32(80); // quality value. 0-100
    if (quality) { var.ulVal = UINT32(quality); }
    hr = pCodecAPI->SetValue(&CODECAPI_AVEncCommonQuality, &var); // dosent work it seems, at least for nvenc
    if (FAILED(hr))
    {
        std::cerr << "Failed to set encoder quality." << std::endl;
        return hr;
    }

    if (qp) {
        // QP, clamped to max 50
        uint16_t defaultQP = std::min<uint16_t>(qp, 50);
        uint16_t iFrameQP = std::min<uint16_t>(qp, 50);
        uint16_t pFrameQP = std::min<uint16_t>(qp + 2, 50);
        uint16_t bFrameQP = std::min<uint16_t>(qp + 4, 50);


        // Create the 64-bit value.
        ULONGLONG qpValue =
            (ULONGLONG(defaultQP) << 0) |
            (ULONGLONG(iFrameQP) << 16) |
            (ULONGLONG(pFrameQP) << 32) |
            (ULONGLONG(bFrameQP) << 48);

        // Set the value.
        var.vt = VT_UI8;
        var.ullVal = qpValue;
        hr = pCodecAPI->SetValue(&CODECAPI_AVEncVideoEncodeQP, &var);
        if (FAILED(hr))
        {
            std::cerr << "Failed to set QP." << std::endl;
            return hr;
        }
    }

    VariantClear(&var);

    return S_OK;
}

int main(int argc, char* argv[]) {
    auto arguments = parseArgs(argc, argv);
    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);

    int monitor = arguments.count("-monitor") > 0 ? std::atoi(arguments["-monitor"].c_str()) : 0; // 0 indexed
    std::string path = arguments.count("-path") > 0 ? arguments["-path"] : ""; // blank/empty means it will be handeled by the function
    int framerate = arguments.count("-fps") > 0 ? strtoull(arguments["-fps"].c_str(), nullptr, 10) : 60; // MUST match monitor refresh rate
    int delay = arguments.count("-delay") > 0 ? std::atoi(arguments["-delay"].c_str()) : 3; // 1 sec wait default
    bool isCompressed = arguments.count("-comp") > 0 ? std::atoi(arguments["-comp"].c_str()) > 0 : false; // h264 vs raw RGB24
    int qp = arguments.count("-qp") > 0 ? strtoull(arguments["-qp"].c_str(), nullptr, 10) : 0;
    int quality = arguments.count("-quality") > 0 ? strtoull(arguments["-quality"].c_str(), nullptr, 10) : 0;
    //std::string crop = arguments.count("-crop") > 0 ? arguments["-crop"] : "default_crop"; // unused ATM

    //path
    std::filesystem::path outputFilePath;
    if (!generateOutputPath(path, isCompressed, outputFilePath)) {
        std::cerr << "Error generating output file path, exiting...\n";
        return 1;
    }
    std::wcout << "Writing output to: " << outputFilePath.wstring() << std::endl;

    // frames/duration
    int runFor = 0;
    if (arguments.count("-frames") > 0) {
        runFor = std::atoi(arguments["-frames"].c_str());
    }
    else if (arguments.count("-duration") > 0) {
        int duration = std::atoi(arguments["-duration"].c_str());
        runFor = duration * framerate;
    }
    else {
        runFor = 60; // default value if neither option is set (60fps * 5 sec = 300)
    }

    // Set power state (dont sleep!)
    SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED | ES_DISPLAY_REQUIRED);

    // start work, DXGI
    DxgiResources resources = initializeDxgi(monitor);

    // Start Media Foundation
    hr = MFStartup(MF_VERSION);
    if (FAILED(hr))
    {
        std::cerr << "Failed to initialize Media Foundation";
        return -1;
    }

    // check avaliable encoders
    //EnumerateEncoders();

    // Init transformers
    IMFTransform* pTransform = NULL;
    IMFTransform* pEncoder = NULL;
    CComPtr<ICodecAPI> pCodecAPI;

    // Create the video processor MFT.
    hr = CoCreateInstance(
        CLSID_VideoProcessorMFT,
        NULL,
        CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(&pTransform)
    );
    if (FAILED(hr)) {
        std::cerr << "Failed to create transformer MFT";
        return -1;
    }

    // pixfmt: 0=RGB24, 1=NV12, 2=H264
    CComPtr<IMFMediaType> INtransform = setupMediaType(resources.desc.Width, resources.desc.Height, 0, framerate);
    if (!INtransform) {
        std::cerr << "Failed to create media type IN";
        return -1;
    }

    // Set up the output media type for the sink writer
    CComPtr<IMFMediaType> OUTtransform = setupMediaType(resources.desc.Width, resources.desc.Height, 1, framerate);
    if (!OUTtransform) {
        std::cerr << "Failed to create media type OUT";
        return -1;
    }

    // Set up the output media type for the transform
    CComPtr<IMFMediaType> ENCout = setupMediaType(resources.desc.Width, resources.desc.Height, isCompressed ? 2 : 0, framerate);
    if (!ENCout) {
        std::cerr << "Failed to create transform output media type";
        return -1;
    }

    hr = pTransform->SetOutputType(0, OUTtransform, 0);
    if (FAILED(hr)) {
        std::cerr << "Failed to set output type for transform" << __LINE__ << std::endl;
        return -1;
    }

    hr = pTransform->SetInputType(0, INtransform, 0);
    if (FAILED(hr)) {
        std::cerr << "Failed to set input type for transform" << __LINE__ << std::endl;
        return -1;
    }

    if (isCompressed) {
        hr = setupEncoder(&pEncoder);
        if (FAILED(hr)) {
            // Handle error
            return -1;
        }

        hr = configureEncoderQuality(pEncoder, qp, quality);
        if (FAILED(hr)) {
            // Handle error
            return -1;
        }

        // ALWAYS output before input. Its fucked up :(
        hr = pEncoder->SetOutputType(0, ENCout, 0);
        if (FAILED(hr)) {
            std::cerr << "Failed to set output type for Encoder" << __LINE__ << std::endl;
            return -1;
        }

        hr = pEncoder->SetInputType(0, OUTtransform, 0);
        if (FAILED(hr)) {
            std::cerr << "Failed to set input type for encoder" << __LINE__ << std::endl;
            return -1;
        }

        hr = configureEncoderQuality(pEncoder, qp, quality);
        if (FAILED(hr)) {
            // Handle error
            return -1;
        }

    }

    // PLEASE LET IT END

    CComPtr<IMFSinkWriter> pSinkWriter = setupSinkWriter(outputFilePath.wstring(), isCompressed);
    if (!pSinkWriter) {
        std::cerr << "Failed to create sink writer" << __LINE__ << std::endl;
        return -1;
    }

    DWORD streamIndex;
    hr = pSinkWriter->AddStream(ENCout, &streamIndex);
    if (FAILED(hr))
    {
        printError("AddStream", hr);
        return -1;
    }

    hr = pSinkWriter->SetInputMediaType(streamIndex, ENCout, NULL);
    if (FAILED(hr)) {
        std::cerr << "Failed to set input media type on SinkWriter" << __LINE__ << std::endl;
        return 1;
    }

    hr = pSinkWriter->BeginWriting();
    if (FAILED(hr))
    {
        printError("BeginWriting", hr);
        return -1;
    }

    // end media foundation prep

    //std::cout << "Usage: " << resources.desc.Usage << std::endl;
    std::cout << "Width: " << resources.desc.Width << std::endl;
    std::cout << "Height: " << resources.desc.Height << std::endl;
    //std::cout << "Format: " << resources.desc.Format << std::endl;
    std::cout << "Framerate: " << framerate << std::endl;
    std::cout << "Compression?: " << isCompressed << std::endl;
    std::cout << "\nHit ctrl+c to break early:" << std::endl;
    std::cout << "\nCapture starting in:" << std::endl;

    countdown(delay);

    if (!SetConsoleCtrlHandler(consoleCtrlHandler, TRUE)) {
        std::cerr << "Failed to set control handler\n";
        return 1;
    }

    // spawn worker threads
    std::thread captureThread(captureFrames, std::ref(resources), runFor);
    std::thread writeThread(writeFrames, std::ref(resources), pSinkWriter, streamIndex, framerate, isCompressed, pTransform, pEncoder); //pTransform pEncoder

    captureThread.join(); // rounds up the cap thread (stop)
    std::cout << "Capture ended!\n ------" << std::endl;
    finished = true; // Notify writeThread that no more frames will be pushed into the queue
    writeThread.join(); // rounds up the writer thread (stop)

    hr = pSinkWriter->Finalize();
    if (FAILED(hr))
    {
        printError("finalize sinkWriter", hr);
        return -1;
    }

    //std::cout << "Timeouts: " << timeOutCounter << std::endl;
    std::cout << "Frames written: " << framesWritten << std::endl;
    std::cout << "Frames requested: " << runFor << std::endl;
    std::cout << "\nDone! :)" << std::endl;

    // Release ME!
    pEncoder->Release();
    resources.pDebugTexture->Release();
    resources.frame->Release();
    resources.desktopResource->Release();
    resources.duplication->Release();
    resources.pContext->Release();
    resources.pDevice->Release();

    MFShutdown();
    SetThreadExecutionState(ES_CONTINUOUS);

    return 0;
}