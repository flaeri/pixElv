#include "arg_parser.h"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <iomanip>

std::vector<Argument> arguments = {
    {"param", "short", "", " Description"},
    {"-help", "-h", "", " Prints the help and exits"},
    {"-version", "-v", "", " Prints the version and exits"},
    {"-monitor", "-m", "0", " 0-based index of the monitor to capture. 0 is primary"},
    {"-path", "-p", ".", " Output file path, if unspecified uses your current directory"},
    {"-framerate", "-fps", "-1", " Output FPS. Should match the refresh rate, which is default beahvior"},
    {"-delay", "-dl", "3", " Delay (sec) before starting capture"},
    {"-compression", "-comp", "0", " 1 for compressed (H.264), 0 for RGB24"},
    {"-bitrate", "-br", "30", " Bitrate (mbps)"},
    {"-frames", "-f", "300", " Frames to capture"},
    {"-duration", "-dur", "", " Duration (sec). If unspecified, uses <frames>"},
    {"-queuelength", "-ql", "-1", " Length/size of queue. How many frames can be buffered before writing. -1 means FPS/2"},
    {"-queuethreshold", "-qt", "-1", " Controls how many frames in the queue before it is swapped to write queue"},
};

std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
    std::map<std::string, std::string> argMap;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // Check if argument is in our list of valid arguments
        auto it = std::find_if(arguments.begin(), arguments.end(), [&arg](const Argument& a) {
            return a.name == arg || a.shortName == arg;
            });
        if (it != arguments.end() && i + 1 < argc && argv[i + 1][0] != '-') {
            argMap[it->name] = argv[++i];
        }
        else if (it != arguments.end()) {
            argMap[it->name] = "";
        }
    }
    return argMap;
}

std::string generateHelpText() {
    std::ostringstream oss;
    for (const auto& arg : arguments) {
        oss << std::setw(23) << std::left << arg.name + " (" + arg.shortName + "):"
            << "Default: " << std::setw(5) << (arg.defaultValue.empty() ? "N/A" : arg.defaultValue)
            << "|" << arg.description << "\n";
    }
    std::cout << "link to wiki. SoonTM.." << std::endl;
    return oss.str();
}
