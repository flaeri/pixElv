#include "arg_parser.h"
#include <d3d11.h>
#include <dxgi1_4.h>
#include <dxgi.h>
#include <d3dkmthk.h>
#include <d3d9.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
#include <vector>
#include <string>
#include <cstring> // for params
#include <cstdlib> // for params
#include <map> // for params
#include <filesystem>
#include <algorithm> // maffs
#include <csignal> // ctrl+c
#include <atomic>
#include "git_info.h" // for git tag + hash
#include <shared_mutex>

//ff
#include "ffmpeg.hpp"
#include "encoder_settings.h"

#pragma comment(lib, "d3d11.lib")

#define CHECK_HR(hr) { if (FAILED(hr)) { std::cerr << "Failed at line " << __LINE__ << std::endl; return {}; } }
template <typename T>
void SafeRelease(T** ppT) {
    if (*ppT) {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

int threadTimeoutSec = 10;
std::shared_mutex smtx;
std::condition_variable_any cv;
bool finished = false;
std::atomic<bool> runFlag{ true };
std::chrono::milliseconds threadTimeout(threadTimeoutSec*1000); // only used for panic if we hang/deadlock, or need 5 sec to finalize long queue

BOOL WINAPI consoleCtrlHandler(DWORD ctrlType) {
    switch (ctrlType) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
        std::cout << "Finishing... Please wait " << threadTimeoutSec << " sec" << std::endl;
        finished = true;
        return TRUE;
    default:
        return FALSE;
    }
}

bool isMemoryUsageHigh() {
    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof(statex);

    GlobalMemoryStatusEx(&statex);

    return statex.dwMemoryLoad > 90; // dwMemoryLoad is a number between 0 and 100 representing the current memory usage
}

struct FrameData {
    unsigned int rowPitch;
    unsigned int depthPitch;
    unsigned char* data;  // The actual frame data, copied from GPU memory
    LONGLONG interval;  // Store interval in QPC units between this frame and the previous
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

    void pushFrame(const D3D11_MAPPED_SUBRESOURCE& frameData, LONGLONG interval) {
        // Create a new buffer and copy the frame data into it
        unsigned char* buffer = new unsigned char[frameData.DepthPitch];
        memcpy(buffer, frameData.pData, frameData.DepthPitch);

        // Create a FrameData instance
        FrameData fd;
        fd.rowPitch = frameData.RowPitch;
        fd.depthPitch = frameData.DepthPitch;
        fd.data = buffer;
        fd.interval = interval;

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

    int size() const {
        return static_cast<int>(queue.size());
    }

    int getMaxSize() const {
        return static_cast<int>(max_size);
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
    LARGE_INTEGER qpcFreq;
    LARGE_INTEGER lastPTS;
};

void listH264Encoders() {
    const AVCodec* codec = nullptr;
    void* iter = nullptr;

    while ((codec = av_codec_iterate(&iter))) {
        if (codec->type == AVMEDIA_TYPE_VIDEO && av_codec_is_encoder(codec)) {
            if (strstr(codec->name, "h264")) {
                std::cout << "Found encoder: " << codec->name << std::endl;
            }
        }
    }
}

const AVCodec* findSuitableCodec(bool isCompressed) {
    std::vector<std::string> codecs;

    if (isCompressed) {
        // Priority for compressed codecs
        codecs = { "h264_nvenc", "h264_amf", "libx264" };
    }
    else {
        // Priority for uncompressed codecs. Add more if needed.
        codecs = { "libx264rgb" }; // Add others like "rawvideo", "qtrle", etc. if necessary
    }

    AVCodecContext* tempCodecCtx = nullptr;

    for (const auto& codecName : codecs) {
        const AVCodec* codec = avcodec_find_encoder_by_name(codecName.c_str());
        if (codec) {
            tempCodecCtx = avcodec_alloc_context3(codec);
            if (!tempCodecCtx) {
                continue; // if allocation failed, move on to next codec
            }

            // Set up the codec context with dummy values
            tempCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
            tempCodecCtx->pix_fmt = isCompressed ? AV_PIX_FMT_NV12 : AV_PIX_FMT_BGR24; // Default to common formats
            tempCodecCtx->width = 1920;
            tempCodecCtx->height = 1080;
            tempCodecCtx->framerate = AVRational{ 60, 1 };
            tempCodecCtx->time_base = AVRational{ 1, 60 }; // 60fps

            if (avcodec_open2(tempCodecCtx, codec, nullptr) == 0) {
                avcodec_free_context(&tempCodecCtx); // Close and free the temporary codec context
                return codec; // Codec opened successfully, return it.
            }

            avcodec_free_context(&tempCodecCtx); // Close and free the temporary codec context
        }
    }

    return nullptr; // No suitable codec found
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

    // If frames is a multiple of fr, print maxTime
    int ts = framesWritten / framerate;
    if (ts > lastPrintedTs) {
        std::cout << "avgMax fGrab (ms): " << maxTime.count() <<
            " TS: " << ts <<
            " pQueue : " << privateWriterFrameQueue <<
            " sQueue: " << sharedFrameQueue << std::endl;

        // Reset maxTime after printing
        maxTime = std::chrono::duration<double, std::milli>(0);

        // Update last printed ts
        lastPrintedTs = ts;
    }
}

int framesCaptured = 0;
int skippedFrames = 0;
int missedFrames = 0;
int totalSkip = 0;
bool isFirstRealCapture = true;
void captureFrames(DxgiResources& resources, int runFor, int framerate, FrameQueue& frameQueue, FrameQueue& privateCaptureQueue, int swapThreshold) {
    QueryPerformanceFrequency(&resources.qpcFreq);

    int queueMaxSize = frameQueue.getMaxSize();
    while (framesCaptured < runFor) {
        if (acquireFrame(resources)) {
            start_timer();

            LARGE_INTEGER currentPTS = resources.frameInfo.LastPresentTime;
            LONGLONG interval = 0;

            if (resources.lastPTS.QuadPart != 0) {  // Avoid the first iteration where lastPTS is not yet set
                interval = currentPTS.QuadPart - resources.lastPTS.QuadPart;
            }
            resources.lastPTS = currentPTS;

            resources.pContext->CopyResource(resources.pDebugTexture, resources.frame);

            HRESULT hr = resources.pContext->Map(resources.pDebugTexture, 0, D3D11_MAP_READ, 0, &resources.mappedResource);
            if (FAILED(hr)) {
                std::cout << hr;
            }

            resources.pContext->Unmap(resources.pDebugTexture, 0);

            privateCaptureQueue.pushFrame(resources.mappedResource, interval);

            if (isFirstRealCapture) {
                missedFrames = -(int)resources.frameInfo.AccumulatedFrames + 1;
                isFirstRealCapture = false;
            }

            if (resources.frameInfo.AccumulatedFrames >= 2) {
                missedFrames = missedFrames + resources.frameInfo.AccumulatedFrames - 1;
                std::cout << "missed " << resources.frameInfo.AccumulatedFrames - 1 << " frame(s). Total: " << missedFrames << std::endl;
            }

            framesCaptured++;
            if (finished) { return; }
        }

        // Swap the queues when privateCaptureQueue is full OR when it has more than x frames and frameQueue is empty
        if (privateCaptureQueue.isFull()) {
            std::cerr << "Warning: CaptureQueue is full ("  << queueMaxSize << "). Waiting for drain!\n" ;
            bool swapped = false;
            while (!swapped) {
                std::unique_lock<std::shared_mutex> lock(smtx);
                skippedFrames++;
                if (frameQueue.empty()) {
                    frameQueue.swap(privateCaptureQueue);
                    cv.notify_all(); // notify all waiting threads
                    std::cerr << "Warning: skipped frames: " << skippedFrames << std::endl;
                    totalSkip = totalSkip + skippedFrames;
                    skippedFrames = 0;
                    swapped = true;
                }
                else {
                    lock.unlock(); // release the lock before sleeping
                    // If not, sleep for a short period before checking again
                    std::this_thread::sleep_for(std::chrono::milliseconds(16));
                }
            }
        }

        else if (privateCaptureQueue.size() >= swapThreshold && [&] {
            std::shared_lock<std::shared_mutex> lock(smtx);
                return frameQueue.empty();
            }()) {
            std::unique_lock<std::shared_mutex> lock(smtx);
            frameQueue.swap(privateCaptureQueue);
            cv.notify_one();
        }
        stop_timer(framerate, privateCaptureQueue.size(), frameQueue.size());
        
        resources.duplication->ReleaseFrame();
        if (finished) { 
            return; }
    }

    // At the end of capturing, if there are any frames left in the private queue, swap them into the shared queue
    if (!privateCaptureQueue.empty()) {
        std::cout << "capture done, swap remaning " << privateCaptureQueue.size() << " frames" << std::endl;
        std::unique_lock<std::shared_mutex> lock(smtx);
        frameQueue.swap(privateCaptureQueue);
    }
}

static int64_t current_pts = 0;
const int cfr_pts_increment = 0;
int totalDupes = 0;

int computeRoundedIncrement(LONGLONG interval, AVRational time_base, LONGLONG qpcFreq, int cfr_pts_increment) {
    int64_t intermediate_result = static_cast<int64_t>(interval) * time_base.den;
    int raw_increment = static_cast<int>(intermediate_result / qpcFreq);

    int ratio = static_cast<int>(std::round(static_cast<double>(raw_increment) / cfr_pts_increment));

    return ratio * cfr_pts_increment;
}

void writeFrameToDisk(FrameData frameData, AVFormatContext* outContext, AVStream* videoStream, AVCodecContext* codecCtx, DxgiResources& resources, uint64_t framerate, bool isCompressed) {
    uint8_t* out_planes[2] = { nullptr, nullptr };
    int ret = 0;

    uint8_t* inData[4] = { frameData.data, nullptr, nullptr, nullptr };
    int inLinesize[4] = { static_cast<int>(frameData.rowPitch), 0, 0, 0 };

    // Allocate a frame to hold the NV12 data
    AVFrame* frame = av_frame_alloc();
    frame->format = codecCtx->pix_fmt;
    frame->width = resources.desc.Width;
    frame->height = resources.desc.Height;

    if (isCompressed) {
        out_planes[0] = (uint8_t*)av_malloc(static_cast<size_t>(resources.desc.Width) * resources.desc.Height);     // Y plane
        out_planes[1] = (uint8_t*)av_malloc(static_cast<size_t>(resources.desc.Width) * resources.desc.Height / 2); // UV plane

        frame->data[0] = out_planes[0];
        frame->data[1] = out_planes[1];
        frame->linesize[0] = resources.desc.Width;
        frame->linesize[1] = resources.desc.Width; // NV12 UV plane has the same width but half the height
    }

    frame->pts = current_pts;
    frame->pkt_duration = cfr_pts_increment;

#ifdef _DEBUG
    std::cout << "frame: " << framesWritten << " | pts: " << frame->pts << " | dur: " << frame->pkt_duration << std::endl;
#endif // DEBUG

    if (av_frame_get_buffer(frame, 0) < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Could not allocate the video frame data: " << errbuf << std::endl;
        av_frame_free(&frame);
        return;
    }

    // Create a scaling context
    SwsContext* swsCtx = sws_getContext(
        frame->width, frame->height, AV_PIX_FMT_BGRA,
        frame->width, frame->height, codecCtx->pix_fmt, //AV_PIX_FMT_RGB24 AV_PIX_FMT_BGR24
        isCompressed ? SWS_ACCURATE_RND | SWS_FULL_CHR_H_INT | SWS_FULL_CHR_H_INP : SWS_BILINEAR, nullptr, nullptr, nullptr);

    if (!swsCtx) {
        std::cerr << "Could not initialize the conversion context";
        av_frame_free(&frame);
        return;
    }

    if (isCompressed) {
        sws_setColorspaceDetails(
            swsCtx,
            sws_getCoefficients(SWS_CS_ITU601),
            AVCOL_RANGE_JPEG, // source range: JPEG denotes full range RGB
            sws_getCoefficients(SWS_CS_ITU601),
            AVCOL_RANGE_JPEG, // target range: JPEG denotes full range YUV
            0, 1 << 16, 1 << 16
        );
    }

    // Convert BGRA
    sws_scale(swsCtx,
        inData, inLinesize,
        0, frame->height,
        frame->data, frame->linesize);

    // Initialize packet
    AVPacket pkt = { 0 };

    //check frame
    if (!frame || !frame->data[0]) {
        std::cerr << "Frame is not correctly initialized!";
        return;
    }

    // Send the frame to the encoder
    ret = avcodec_send_frame(codecCtx, frame);

    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Error sending frame to encoder: " << errbuf << std::endl;
        return;
    }

    // Receive packets from the encoder and write them to the output file
    while (ret >= 0) {
        ret = avcodec_receive_packet(codecCtx, &pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        else if (ret < 0) {
            std::cerr << "Error receiving packet from encoder" << std::endl;
            return;
        }

        pkt.stream_index = videoStream->index;
        av_interleaved_write_frame(outContext, &pkt);
        av_packet_unref(&pkt);
    }

    framesWritten++;

    // free stuff
    av_frame_free(&frame);
    if (out_planes[0]) av_freep(&out_planes[0]);
    if (out_planes[1]) av_freep(&out_planes[1]);

    sws_freeContext(swsCtx);
}

void writeFrames(AVFormatContext* outContext, AVStream* videoStream, AVCodecContext* codecCtx, DxgiResources& resources, uint64_t framerate, bool isCompressed, bool isVFR, FrameQueue& frameQueue, FrameQueue& privateCaptureQueue) {
    int cfr_pts_increment = static_cast<int>(videoStream->time_base.den / static_cast<int>(framerate));
    while (!finished || !frameQueue.empty()) {
        std::unique_lock<std::shared_mutex> lock(smtx);
        cv.wait_for(lock, threadTimeout, [&frameQueue] {return finished || !frameQueue.empty(); });
        while (!frameQueue.empty()) {
            FrameData frameData1 = frameQueue.popFrame();
            lock.unlock(); // unlock the mutex while writing the frames to disk

            if (!isVFR) {
                // Calculate individual PTS increments and round them
                int rounded_pts_increment1 = computeRoundedIncrement(frameData1.interval, videoStream->time_base, resources.qpcFreq.QuadPart, cfr_pts_increment);

                // Use rounded PTS increments to calculate the total PTS increment
                int duplication_count = rounded_pts_increment1 / cfr_pts_increment - 1;

                if (duplication_count >= 1) {
                    totalDupes = totalDupes + duplication_count;
                    std::cerr << "inserting " << duplication_count << " dupes at PTS: " << current_pts << std::endl;
                }

                for (int i = 0; i <= duplication_count; i++) {
                    writeFrameToDisk(frameData1, outContext, videoStream, codecCtx, resources, framerate, isCompressed);
                    current_pts += cfr_pts_increment;
                }
            } else {
                // Convert frameData.interval from QPC units to the timebase of the video stream
                int64_t pts_increment = (frameData1.interval * videoStream->time_base.den) / resources.qpcFreq.QuadPart;
                writeFrameToDisk(frameData1, outContext, videoStream, codecCtx, resources, framerate, isCompressed);
                current_pts += pts_increment;
            }

            // Free the memory of frameData1 once it's written and dupes are done
            delete[] frameData1.data;
            
            lock.lock(); // reacquire the lock before the next iteration
            if (finished) {
                std::cout << "finishing writes, frames remaining: " << frameQueue.size() << std::endl;
                if (frameQueue.empty()) { return; }
            }
            if (isMemoryUsageHigh()) {
                std::cerr << "\nMemory usage above 90%! Stopping! This might take a minute...\n" << std::endl;
                finished = true;
            }
        }
        if (finished && frameQueue.empty()) { return; }
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
            sprintf_s(filename, "output-ll.mov"); //avi
        }
        outputPath = std::filesystem::current_path() / filename;
    }

    return true;
}

int getArgAsInt(const std::map<std::string, std::string>& arguments, const std::string& argKey, int defaultValue = -1) {
    if (arguments.count(argKey) > 0) {
        unsigned __int64 temp = strtoull(arguments.at(argKey).c_str(), nullptr, 10);
        return static_cast<int>(temp);
    }
    return defaultValue;
}

int main(int argc, char* argv[]) {
    std::cout << "Git Hash: " << GIT_HASH << std::endl;
    std::cout << "Version: " << GIT_TAG << std::endl;
    printf("Linked with FFmpeg version: %s\n", av_version_info());

    auto arguments = parseArgs(argc, argv);

    if (arguments.count("-help") > 0 || arguments.count("-h") > 0) {
        std::cout << "\n" << generateHelpText() << std::endl;
        return 0;
    }

    HRESULT hr;
    hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED); //COINIT_APARTMENTTHREADED

    bool version = arguments.count("-version") > 0; // print ver and exit
    int monitor = arguments.count("-monitor") > 0 ? std::atoi(arguments["-monitor"].c_str()) : 0; // 0 indexed
    std::string path = arguments.count("-path") > 0 ? arguments["-path"] : ""; // blank/empty means it will be handeled by the function
    int framerate = getArgAsInt(arguments, "-framerate"); // MUST match monitor refresh rate
    int delay = arguments.count("-delay") > 0 ? std::atoi(arguments["-delay"].c_str()) : 3; // 1 sec wait default
    bool isCompressed = arguments.count("-compression") > 0 ? std::atoi(arguments["-compression"].c_str()) > 0 : false; // h264 vs raw RGB24
    int bitrate = getArgAsInt(arguments, "-bitrate", 30);
    int queueLengthParam = getArgAsInt(arguments, "-queuelength"); //frames in the queue
    int swapSizeParam = getArgAsInt(arguments, "-queuethreshold"); //queue fullness when swap
    bool isVFR = arguments.count("-vfr") > 0 ? std::atoi(arguments["-vfr"].c_str()) > 0 : false; // h264 vs raw RGB24

    //std::string crop = arguments.count("-crop") > 0 ? arguments["-crop"] : "default_crop"; // unused ATM
    
    if (version) { return 0; }

    std::cout << "\nDelay active. Starting in:" << std::endl;
    countdown(delay);

    // start work, DXGI. Has to be after countdown, due to context switch can happen
    DxgiResources resources = initializeDxgi(monitor);
    resources.qpcFreq = { 0 };
    resources.lastPTS = { 0 };

    //framerate and queues
    if (framerate == -1) {
        framerate = resources.refreshRate;
    }

    //queue length (max frames in a queue)
    int maxQueueLength;
    if (queueLengthParam == -1) {
        maxQueueLength = framerate / 2; // 30 seems safe, ish. 60 works better, especially for 120fps.
    } else {
        maxQueueLength = queueLengthParam;
    }

    FrameQueue frameQueue(maxQueueLength);
    FrameQueue privateCaptureQueue(maxQueueLength);

    // queue swap divisor/fullness. 60 / 4 = 15. Try to swap on 25% (15 out of 60). Half would be 2 (60/2)
    int swapThreshold;
    int swapDiv = 10; //test 3 (old) and 4,5. Smaller seems better (check overhead)
    if (swapSizeParam == -1) {
        swapThreshold = std::clamp(framerate / swapDiv, 1, (maxQueueLength - 1));
    } else {
        swapThreshold = std::clamp(swapSizeParam, 1, (maxQueueLength-1));
    }

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
    std::cout << "Queue size (frames): " << maxQueueLength << " | Swaps @ " << swapThreshold << " frames" << std::endl;
    std::cout << "Compression: " << (isCompressed ? "TRUE" : "FALSE") << std::endl;
    std::cout << "VFR: " << (isVFR ? "TRUE" : "FALSE") << std::endl;

    std::cout << "\nHit ctrl+c to break early:" << std::endl;
    std::wcout << "\nWriting output to: " << outputFilePath.wstring() << std::endl;

    // Set power state (dont sleep!)
    SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_AWAYMODE_REQUIRED | ES_DISPLAY_REQUIRED);

    // ffmpeg start
#ifdef _DEBUG
    av_log_set_level(AV_LOG_DEBUG);
#endif

    //listH264Encoders();

    AVFormatContext* outContext = nullptr;
    AVStream* videoStream = nullptr;

    std::string outputFilePathStr = outputFilePath.string();
    avformat_alloc_output_context2(&outContext, nullptr, isCompressed ? "mp4" : "mov", outputFilePathStr.c_str()); //avi
    if (!outContext) {
        std::cerr << "Could not create output context";
        return -1;
    }

    const AVCodec* codec = findSuitableCodec(isCompressed);
    if (!codec) {
        std::cerr << "Suitable codec not found!";
        return -1;
    }

    printf("\nCodec chosen: %s\n", codec->name);

    videoStream = avformat_new_stream(outContext, codec);
    if (!videoStream) {
        std::cerr << "Failed allocating video stream";
        return -1;
    }

    videoStream->time_base = AVRational{ 1, 90000 };
    const int pts_increment = videoStream->time_base.den / framerate;
    videoStream->avg_frame_rate = AVRational{ framerate, 1 };

    //Allocate the codec context
    AVCodecContext* codecCtx = avcodec_alloc_context3(codec);
    if (!codecCtx) {
        std::cerr << "Failed to allocate codec context!";
        return -1;
    }

    codecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    codecCtx->pix_fmt = isCompressed ? AV_PIX_FMT_NV12 : AV_PIX_FMT_BGR24; // AV_PIX_FMT_RGB24 AV_PIX_FMT_BGR24  AV_PIX_FMT_GBRP AV_PIX_FMT_GBRP12LE
    codecCtx->width = resources.desc.Width;
    codecCtx->height = resources.desc.Height;
    codecCtx->framerate = AVRational{ framerate, 1 };
    codecCtx->time_base = videoStream->time_base;

    //color for compressed
    if (isCompressed) {
        codecCtx->colorspace = AVCOL_SPC_BT470BG; // AVCOL_SPC_BT470BG AVCOL_SPC_BT709
        codecCtx->color_primaries = AVCOL_PRI_BT470BG;
        codecCtx->color_trc = AVCOL_TRC_SMPTE170M; //AVCOL_TRC_IEC61966_2_1
        codecCtx->color_range = AVCOL_RANGE_JPEG;
    }

    // If the output format needs global headers, set the flag
    if (outContext->oformat->flags & AVFMT_GLOBALHEADER) {
        codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    //Open the codec
    AVDictionary* codec_options = nullptr;
    av_dict_set(&codec_options, "bf", "0", 0);
    
    if (!isCompressed && strcmp(codec->name, "libx264rgb") == 0) {
        av_dict_set(&codec_options, "qp", "0", 0);
        av_dict_set(&codec_options, "preset", "ultrafast", 0); //superfast
    }

    if (isCompressed) {
        if (encoderOptions.find(codec->name) != encoderOptions.end()) {
            auto& options = encoderOptions[codec->name];
            for (const auto& [key, value] : options) {
                av_dict_set(&codec_options, key.c_str(), value.c_str(), 0);
            }
        }
    }

    if (avcodec_open2(codecCtx, codec, &codec_options) < 0) {
        std::cerr << "Failed to open codec!";
        avcodec_free_context(&codecCtx);
        return -1;
    }

    int ret = avcodec_parameters_from_context(videoStream->codecpar, codecCtx);
    if (ret < 0) {
        std::cerr << "Failed param from context";
        return -1;
    }

    // Open output file and write the header
    if (!(outContext->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&outContext->pb, outputFilePathStr.c_str(), AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Failed to open output file";
            return -1;
        }
    }

    if (avformat_write_header(outContext, nullptr) < 0) {
        std::cerr << "Error occurred when opening output file.";
        return -1;
    }

    //tests
    if (!codecCtx || (codecCtx->codec_type != AVMEDIA_TYPE_VIDEO)) {
        std::cerr << "Codec context is not correctly initialized!";
        return -1;
    }

    // ffmpeg end

    std::cout << "\nCapture starting in:" << std::endl;

    if (!SetConsoleCtrlHandler(consoleCtrlHandler, TRUE)) {
        std::cerr << "Failed to set control handler\n";
        return 1;
    }

    if (finished) { return 0; }

    // spawn worker threads
    std::thread captureThread(captureFrames, std::ref(resources), runFor, framerate, std::ref(frameQueue), std::ref(privateCaptureQueue), swapThreshold);
    std::thread writeThread(writeFrames, outContext, videoStream, codecCtx, std::ref(resources), framerate, isCompressed, isVFR, std::ref(frameQueue), std::ref(privateCaptureQueue));
    
    captureThread.join(); // rounds up the cap thread (stop)
    std::cout << "Capture ended!\n ------" << std::endl;
    // Notify writeThread that no more frames will be pushed into the queue
    if (!finished) { 
        std::cout << "Finishing... Please wait 10 sec" << std::endl;
        finished = true;
    }
    
    writeThread.join(); // rounds up the writer thread (stop)

    // finalize file and close ffmpeg stuff
    av_write_trailer(outContext);
    avcodec_close(codecCtx);
    avcodec_free_context(&codecCtx);
    if (!(outContext->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&outContext->pb);
    }
    avformat_free_context(outContext);

    int actualFrames = framesCaptured - totalSkip - missedFrames;
    std::cout << "\n--- Stats ---" << std::endl;
    std::cout << "Frames requested: " << runFor << std::endl;
    std::cout << "Attempted frames: " << framesCaptured << std::endl;
    std::cout << "Total frames missed : " << missedFrames << " | skipped : " << totalSkip << std::endl;
    std::cout << "Total frames captured: " << actualFrames << std::endl;
    std::cout << "Unique written : " << framesWritten - totalDupes << std::endl;
    std::cout << "Frames written: " << framesWritten << " | Duplicates: " << totalDupes << std::endl;
    std::cout << "\nDone! :)" << std::endl;

    // Release ME!
    resources.pDebugTexture->Release();
    resources.frame->Release();
    resources.desktopResource->Release();
    resources.duplication->Release();
    resources.pContext->Release();
    resources.pDevice->Release();

    SetThreadExecutionState(ES_CONTINUOUS);

    return 0;
}