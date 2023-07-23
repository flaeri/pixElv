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
#include "git_info.h" // for git tag + hash

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
#include <shared_mutex>

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
template <typename T>
void SafeRelease(T** ppT) {
    if (*ppT) {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

std::shared_mutex smtx;
std::condition_variable_any cv;
bool finished = false;
std::atomic<bool> runFlag{ true };
std::chrono::milliseconds threadTimeout(5000); // only used for panic if we hang/deadlock, or need 5 sec to finalize long queue

BOOL WINAPI consoleCtrlHandler(DWORD ctrlType) {
    switch (ctrlType) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
        std::cout << "Finishing... Please wait 5 sec" << std::endl;
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
    std::size_t getMaxSize() const {
        return max_size;
    }
    void swap(FrameQueue& other) {
        std::swap(queue, other.queue);
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
    UINT refreshRate;
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

    // get refresh rate
    DXGI_MODE_DESC1* displayModes = NULL;
    UINT numModes = 0;
    UINT flags = 0;

    hr = output1->GetDisplayModeList1(resources.desc.Format, flags, &numModes, NULL);
    CHECK_HR(hr);

    displayModes = new DXGI_MODE_DESC1[numModes];
    hr = output1->GetDisplayModeList1(resources.desc.Format, flags, &numModes, displayModes);
    CHECK_HR(hr);

    // Query IDXGIOutput2 or IDXGIOutput3 interface
    IDXGIOutput3* output3 = NULL;
    hr = output->QueryInterface(__uuidof(IDXGIOutput3), (void**)&output3);
    CHECK_HR(hr);

    // Prepare the desired mode description
    DXGI_MODE_DESC1 desiredModeDesc = {};
    desiredModeDesc.Format = resources.desc.Format;
    desiredModeDesc.Width = resources.desc.Width;
    desiredModeDesc.Height = resources.desc.Height;

    // Find the closest matching mode
    DXGI_MODE_DESC1 closestModeDesc;
    hr = output3->FindClosestMatchingMode1(&desiredModeDesc, &closestModeDesc, resources.pDevice);
    CHECK_HR(hr);

    // The refresh rate is given as a rational number (Numerator / Denominator)
    float refreshRate = static_cast<float>(closestModeDesc.RefreshRate.Numerator) / closestModeDesc.RefreshRate.Denominator;
    UINT roundedRefreshRate = static_cast<UINT>(std::round(refreshRate));
    resources.refreshRate = roundedRefreshRate;

    wprintf(L"Refresh Rate: %d Hz\n", resources.refreshRate);

    delete[] displayModes;
    // end get refreshrate

    hr = resources.pDevice->CreateTexture2D(&(resources.desc), NULL, &(resources.pDebugTexture));
    CHECK_HR(hr);

    resources.duplication->ReleaseFrame();
    resources.pContext->Unmap(resources.pDebugTexture, 0);

    return resources;
}

IMFSample* createMediaSample(BYTE* data, DWORD cbData, const std::string& error_msg) {
    HRESULT hr = S_OK;
    IMFSample* pSample = NULL;
    IMFMediaBuffer* pBuffer = NULL;

    // Create a new media sample
    hr = MFCreateSample(&pSample);

    // Create a new memory buffer
    if (SUCCEEDED(hr)) {
        hr = MFCreateMemoryBuffer(cbData, &pBuffer);
    }

    // If data is provided, copy it into the buffer
    if (SUCCEEDED(hr) && data != nullptr) {
        BYTE* pBufferStart = NULL;
        DWORD cbMaxLength = 0, cbCurrentLength = 0;

        hr = pBuffer->Lock(&pBufferStart, &cbMaxLength, &cbCurrentLength);
        if (SUCCEEDED(hr)) {
            CopyMemory(pBufferStart, data, cbData);
            hr = pBuffer->Unlock();
        }

        if (SUCCEEDED(hr)) {
            hr = pBuffer->SetCurrentLength(cbData);
        }
    }

    // Add the buffer to the sample
    if (SUCCEEDED(hr)) {
        hr = pSample->AddBuffer(pBuffer);
    }

    // If anything failed, release any resources we have and return null
    if (FAILED(hr)) {
        std::cerr << "Failed to " << error_msg << "\n";
        SafeRelease(&pSample);
        pSample = NULL;
    }

    SafeRelease(&pBuffer);
    return pSample;
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

int lastPrintedTs = -1;
int framesWritten = 0;

bool isFirstSample = true;

LONGLONG llOutputSampleTime{};
LONGLONG llOutputSampleDuration{};

std::chrono::duration<double, std::milli> maxTime(0); // variable to keep track of the maximum time
std::chrono::high_resolution_clock::time_point startTime; // variable to keep track of the start time

void start_timer() {
    // Get the current time
    startTime = std::chrono::high_resolution_clock::now();
}

void stop_timer(int framerate, int privateWriterFrameQueue, int sharedFrameQueue) {
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(stopTime - startTime);

    // If the elapsed time is greater than the maximum time, update maxTime
    if (elapsedTime > maxTime) { maxTime = elapsedTime; }

    // If frames is a multiple of 60, print maxTime
    int ts = framesWritten / framerate;
    if (ts > lastPrintedTs) {
        std::cout << "avgMax fGrab (ms): " << maxTime.count() <<
            " TS: " << ts <<
            " pQueue : " << privateWriterFrameQueue <<  //assuming you want to print the size
            " sQueue: " << sharedFrameQueue << std::endl; //assuming you want to print the size

        // Reset maxTime after printing
        maxTime = std::chrono::duration<double, std::milli>(0);

        // Update last printed ts
        lastPrintedTs = ts;
    }
}

void captureFrames(DxgiResources& resources, int runFor, int framerate, FrameQueue& frameQueue, FrameQueue& privateCaptureQueue) {
    int framesCaptured = 0;
    int queueSize = frameQueue.getMaxSize();
    int swapSize = queueSize/3;
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

            privateCaptureQueue.pushFrame(resources.mappedResource);
            framesCaptured++;
            if (finished) { return; }
        }

        // Swap the queues when privateCaptureQueue is full OR when it has more than x frames and frameQueue is empty
        if (privateCaptureQueue.isFull()) {
            std::cerr << "Warning: privateCaptureQueue is full. Waiting for frameQueue to empty...\n";
            bool swapped = false;
            while (!swapped) {
                std::unique_lock<std::shared_mutex> lock(smtx);
                if (frameQueue.empty()) {
                    frameQueue.swap(privateCaptureQueue);
                    cv.notify_all(); // notify all waiting threads
                    swapped = true;
                }
                else {
                    lock.unlock(); // release the lock before sleeping
                    // If not, sleep for a short period before checking again
                    std::this_thread::sleep_for(std::chrono::milliseconds(16));
                }
            }
        }

        else if (privateCaptureQueue.size() >= swapSize && [&] {
            std::shared_lock<std::shared_mutex> lock(smtx);
                return frameQueue.empty();
            }()) {
            std::unique_lock<std::shared_mutex> lock(smtx);
            frameQueue.swap(privateCaptureQueue);
            cv.notify_all(); // notify all waiting threads
        }
        stop_timer(framerate, privateCaptureQueue.size(), frameQueue.size());
        if (finished) { return; }
    }

    // At the end of capturing, if there are any frames left in the private queue, swap them into the shared queue
    if (!privateCaptureQueue.empty()) {
        std::cout << "capture done, swap remaning " << privateCaptureQueue.size() << " frames" << std::endl;
        std::unique_lock<std::shared_mutex> lock(smtx);
        frameQueue.swap(privateCaptureQueue);
    }

    std::cout << "Total frames captured: " << framesCaptured << std::endl;
}

void writeFrameToDisk(FrameData frameData, DxgiResources& resources, IMFSinkWriter* pSinkWriter, IMFTransform* pTransform, DWORD streamIndex, uint64_t framerate, bool isCompressed) {
    HRESULT hr;
    MFT_OUTPUT_DATA_BUFFER outputDataBuffer{};
    
    // Create the input sample
    IMFSample* pInputSample = createMediaSample(
        frameData.data,
        resources.desc.Width * resources.desc.Height * 4, // RGBA 
        "creating input sample"
    );

    // Create the output sample
    IMFSample* pOutputSample = createMediaSample(
        nullptr,  // No data to copy
        resources.desc.Width * resources.desc.Height * 4,
        "creating output sample"
    );

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

    outputDataBuffer = { 0, pOutputSample, 0, NULL };
    DWORD status;

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
            if (hr == MF_E_NOTACCEPTING || hr == MF_E_TRANSFORM_NEED_MORE_INPUT) {
                return;
            }
            if (FAILED(hr)) {
                std::cerr << "Failed at line " << __LINE__ << " " << hr << std::endl;
                return;
            }
        }
        if (finished) { return; }
    } while (hr == MF_E_NOTACCEPTING);

    // Get the transformed data
    hr = pTransform->ProcessOutput(0, 1, &outputDataBuffer, &status);
    if (FAILED(hr)) {
        std::cerr << "Failed at line " << __LINE__ << std::endl;
        return;
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


        if (!isFirstSample) {
            // Get the timestamp and duration from the output sample

            hr = outputDataBuffer.pSample->GetSampleDuration(&llOutputSampleDuration);
            if (FAILED(hr)) {
                std::cerr << "Failed to get sample duration";
                return;
            }

            llOutputSampleTime = llOutputSampleTime + llOutputSampleDuration;
        }

        // Dont care, release
        if (outputDataBuffer.pEvents) {
            outputDataBuffer.pEvents->Release();
        }
    }

    // Write the sample to disk
    hr = pSinkWriter->WriteSample(streamIndex, outputDataBuffer.pSample);
    if (hr == S_OK) {
        framesWritten++;
    }
    else {
        std::cerr << "Failed to write sample" << std::endl;
        return;
    }

    if (outputDataBuffer.pSample != nullptr) {
        outputDataBuffer.pSample->Release();
    }

    if (pInputSample != nullptr) {
        pInputSample->Release();
    }
    delete[] frameData.data;
}

void writeFrames(DxgiResources& resources, IMFSinkWriter* pSinkWriter, DWORD streamIndex, uint64_t framerate, bool isCompressed, IMFTransform* pTransform, FrameQueue& frameQueue, FrameQueue& privateCaptureQueue) {
    while (!finished || !frameQueue.empty()) {
        std::unique_lock<std::shared_mutex> lock(smtx);
        cv.wait_for(lock, threadTimeout, [&frameQueue] {return finished || !frameQueue.empty(); });
        while (!frameQueue.empty()) {
            FrameData frameData = frameQueue.popFrame();
            lock.unlock(); // unlock the mutex while writing the frame to disk
            writeFrameToDisk(frameData, resources, pSinkWriter, pTransform, streamIndex, framerate, isCompressed);
            lock.lock(); // reacquire the lock before the next iteration
        }
        if (finished && frameQueue.empty()) { 
            return; }
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

CComPtr<IMFMediaType> setupMediaType(int width, int height, int pixfmt, int framerate, int bitrate) {
    HRESULT hr;
    GUID format;

    // Choose format
    switch (pixfmt) {
    case 0:
        format = MFVideoFormat_RGB24; // {00000014-0000-0010-8000-00AA00389B71}
        break;
    case 1:
        format = MFVideoFormat_YUY2;
        break;
    case 2:
        format = MFVideoFormat_H264;
        break;
    case 3:
        format = MFVideoFormat_ARGB32; // {00000015-0000-0010-8000-00AA00389B71}
        break;
    case 4:
        format = MFVideoFormat_NV12;
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

    if (bitrate) {
    hr = pMediaTypeOut->SetUINT32(MF_MT_AVG_BITRATE, bitrate);
    if (FAILED(hr)) {
            std::cerr << "Failed to set MF_MT_INTERLACE_MODE\n";
            return nullptr;
        }
    }

    return pMediaTypeOut;
}

int main(int argc, char* argv[]) {
    std::cout << "Git Hash: " << GIT_HASH << std::endl;
    std::cout << "Version: " << GIT_TAG << std::endl;

    auto arguments = parseArgs(argc, argv);
    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);

    int monitor = arguments.count("-monitor") > 0 ? std::atoi(arguments["-monitor"].c_str()) : 0; // 0 indexed
    std::string path = arguments.count("-path") > 0 ? arguments["-path"] : ""; // blank/empty means it will be handeled by the function
    int framerate = arguments.count("-fps") > 0 ? strtoull(arguments["-fps"].c_str(), nullptr, 10) : -1; // MUST match monitor refresh rate
    int delay = arguments.count("-delay") > 0 ? std::atoi(arguments["-delay"].c_str()) : 3; // 1 sec wait default
    bool isCompressed = arguments.count("-comp") > 0 ? std::atoi(arguments["-comp"].c_str()) > 0 : false; // h264 vs raw RGB24
    bool version = (arguments.count("-version") > 0 || arguments.count("-v") > 0); // print ver and exit
    int bitrate = arguments.count("-bitrate") > 0 ? strtoull(arguments["-bitrate"].c_str(), nullptr, 10) : 30;
    //std::string crop = arguments.count("-crop") > 0 ? arguments["-crop"] : "default_crop"; // unused ATM
    
    if (version) { return 0; }

    std::cout << "\nDelay active. Starting in:" << std::endl;
    countdown(delay);

    // start work, DXGI. Has to be after countdown, due to context switch can happen during prepp.
    DxgiResources resources = initializeDxgi(monitor);

    //framerate and queues
    if (framerate == -1) {
        framerate = resources.refreshRate;
    }
    int maxQueueSize = framerate / 2; // 30 seems safe, ish. 60 works better, especially for 120fps.
    FrameQueue frameQueue(maxQueueSize);
    FrameQueue privateCaptureQueue(maxQueueSize);

    // frames/duration
    int runFor = 0;
    if (arguments.count("-frames") > 0) {
        runFor = std::atoi(arguments["-frames"].c_str());
        if (runFor == 0) {
            runFor = 12 * 60 * 60 * framerate; // 12 hours in frames
        }
    }
    else if (arguments.count("-duration") > 0) {
        int duration = std::atoi(arguments["-duration"].c_str());
        if (duration == 0) {
            duration = 12 * 60 * 60; // 12 hours in seconds
        }
        runFor = duration * framerate;
    }
    else {
        runFor = 300; // default value if neither option is set (60fps * 5 sec = 300)
    }

    // bitrate
    bitrate = bitrate * 1000 * 1000; //bitrate specified in mbps

    //path
    std::filesystem::path outputFilePath;
    if (!generateOutputPath(path, isCompressed, outputFilePath)) {
        std::cerr << "Error generating output file path, exiting...\n";
        return 1;
    }

    std::cout << "Width: " << resources.desc.Width << std::endl;
    std::cout << "Height: " << resources.desc.Height << std::endl;
    std::cout << "Format: " << resources.desc.Format << std::endl;
    std::cout << "Framerate: " << framerate << std::endl;
    std::cout << "Compression?: " << isCompressed << std::endl;
    std::cout << "\nHit ctrl+c to break early:" << std::endl;
    std::wcout << "\nWriting output to: " << outputFilePath.wstring() << std::endl;

    // Set power state (dont sleep!)
    SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED | ES_DISPLAY_REQUIRED);

    // Start Media Foundation
    hr = MFStartup(MF_VERSION);
    if (FAILED(hr))
    {
        std::cerr << "Failed to initialize Media Foundation";
        return -1;
    }

    // Init transformers
    IMFTransform* pTransform = NULL;
    IMFVideoProcessorControl* pVPC = NULL;

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

    hr = pTransform->QueryInterface(IID_PPV_ARGS(&pVPC));
    if (FAILED(hr)) {
        std::cerr << "Failed to query IMFVideoProcessorControl";
        return -1;
    }

    // Set mirror mode to vertical if compressed, yuv wants to not be upside down
    hr = pVPC->SetMirror(isCompressed ? MIRROR_VERTICAL : MIRROR_NONE);
    if (FAILED(hr)) {
        std::cerr << "Failed to set mirror mode";
        return -1;
    }
    pVPC->Release();

    // pixfmt: 0=RGB24, 1=YUY2, 2=H264, 3=ARGB32, 4=NV12
    CComPtr<IMFMediaType> ARBG = setupMediaType(resources.desc.Width, resources.desc.Height, 3, framerate, NULL);
    if (!ARBG) {
        std::cerr << "Failed to create media type IN";
        return -1;
    }

    CComPtr<IMFMediaType> NV12 = setupMediaType(resources.desc.Width, resources.desc.Height, 4, framerate, NULL);
    if (!NV12) {
        std::cerr << "Failed to create media type OUT";
        return -1;
    }

    CComPtr<IMFMediaType> RGB = setupMediaType(resources.desc.Width, resources.desc.Height, 0, framerate, NULL);
    if (!RGB) {
        std::cerr << "Failed to create media type OUT";
        return -1;
    }

    CComPtr<IMFMediaType> h264 = setupMediaType(resources.desc.Width, resources.desc.Height, 2, framerate, bitrate);
    if (!h264) {
        std::cerr << "Failed to create media type OUT";
        return -1;
    }

    hr = pTransform->SetOutputType(0, isCompressed ? NV12 : RGB, 0);
    if (FAILED(hr)) {
        std::cerr << "Failed to set output type for transform" << __LINE__ << std::endl;
        return -1;
    }

    hr = pTransform->SetInputType(0, ARBG, 0);
    if (FAILED(hr)) {
        std::cerr << "Failed to set input type for transform" << __LINE__ << std::endl;
    }

    // PLEASE LET IT END

    CComPtr<IMFSinkWriter> pSinkWriter = setupSinkWriter(outputFilePath.wstring(), isCompressed);
    if (!pSinkWriter) {
        std::cerr << "Failed to create sink writer" << __LINE__ << std::endl;
        return -1;
    }

    DWORD streamIndex;
    hr = pSinkWriter->AddStream(isCompressed ? h264 : RGB, &streamIndex);
    if (FAILED(hr)) {
        std::cerr << "Failed at line: " << __LINE__ << std::endl;
    }

    hr = pSinkWriter->SetInputMediaType(streamIndex, isCompressed ? NV12 : RGB, NULL);
    if (FAILED(hr)) {
        std::cerr << "Failed to set input media type on SinkWriter" << __LINE__ << std::endl;
        return 1;
    }

    hr = pSinkWriter->BeginWriting();
    if (FAILED(hr)) {
        std::cerr << "Failed at line: " << __LINE__ << std::endl;
    }

    // end media foundation prep

    std::cout << "\nCapture starting in:" << std::endl;

    if (!SetConsoleCtrlHandler(consoleCtrlHandler, TRUE)) {
        std::cerr << "Failed to set control handler\n";
        return 1;
    }

    if (finished) { return 0; }

    // spawn worker threads
    std::thread captureThread(captureFrames, std::ref(resources), runFor, framerate, std::ref(frameQueue), std::ref(privateCaptureQueue));
    std::thread writeThread(writeFrames, std::ref(resources), pSinkWriter, streamIndex, framerate, isCompressed, pTransform, std::ref(frameQueue), std::ref(privateCaptureQueue)); //pTransform pEncoder

    
    captureThread.join(); // rounds up the cap thread (stop)
    std::cout << "Capture ended!\n ------" << std::endl;
    // Notify writeThread that no more frames will be pushed into the queue
    if (!finished) { 
        std::cout << "Finishing... Please wait 5 sec" << std::endl;
        finished = true;
    }
    
    writeThread.join(); // rounds up the writer thread (stop)

    hr = pSinkWriter->Finalize();
    if (FAILED(hr)) {
        std::cerr << "Failed at line: " << __LINE__ << std::endl;
    }

    //std::cout << "Timeouts: " << timeOutCounter << std::endl;
    std::cout << "Frames written: " << framesWritten << std::endl;
    std::cout << "Frames requested: " << runFor << std::endl;
    std::cout << "\nDone! :)" << std::endl;

    // Release ME!
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