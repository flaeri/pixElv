#include "encoder_settings.h"

std::map<std::string, std::map<std::string, std::string>> encoderOptions = {
    { "h264_nvenc", {   {"ratecontrol", "constqp"}, 
                        {"qp", "16"} } },

    { "h264_amf",   {   {"ratecontrol", "cqp"}, 
                        {"qp_i", "14"}, 
                        {"qp_p", "16"},
                        {"quality", "1"}
} }
};