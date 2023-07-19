# pixElv
![image](https://github.com/flaeri/pixElv/assets/50419942/49e1d2b0-7c6f-4104-bb44-2cdf5c5fc9d0)

PixElv (tries to be) a high-performance screen capture tool, leveraging the DXGI duplication functionality and hardware encoding support via on Media Foundation. The program is designed for efficient capture of raw and compressed output from your monitor, with an emphasis of *trying* to get all of the frames, which is quite a challenge.

## Features
- Screen capture using DXGI monitor duplication
- Output in raw (B8G8R8A8_UNORM (RGB24) with stripped alpha) and compressed formats (h264, nv12)
- Hardware encoder support using Media Foundation (milage may vary)

## Usage

```shell
PixElv.exe -monitor [index] -fps [framerate] -delay [seconds] -comp [0 or 1] -qp [value] -quality [value] -frames [number] -duration [seconds] -path [output path]
```

## Parameters
```
-monitor: 0-based index of the monitor to capture. Default is 0 (primary monitor).
-path: Output file path. If blank or not specified, it will write the file to the location your current directory (pwd to check).
-fps: Framerate for the output. This must match the monitor's refresh rate. Default is 60. Anythiung but 60 is untested, and unlikely to work well.
-delay: Delay in seconds before starting capture. Default is 3 seconds.
-comp: Set to 1 for compressed (H.264) output, 0 for raw RGB24 output. Default is raw output.
-qp: Quantization parameter for the encoder. QP takes precedence over quality if both set.
-quality: Quality setting for the encoder. Default, assuming no QP set. Defaults to 80 (0-100, higher = better quality and bigger file). About qp 22.
-frames: Number of frames to capture. In frames, obviously. 60 frames in a sec usually.
-duration: Duration of capture in seconds. Multiplied by fps to determine the number of frames to capture. 
```
If neither -frames nor -duration are specified, the default is 300 frames (5 seconds at 60fps).
If you specify either to 0, it should run for 12 hours, leaving you free to stop it at your whim (ctrl+c).

You should be able to cancel the capture early by hitting ctrl+c.
