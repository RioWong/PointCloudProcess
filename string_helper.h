#pragma once
#include <string>
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
class StringHelper
{
public:
    StringHelper();
    ~StringHelper();
    static std::vector<std::string> split(std::string& str, const std::string find);
    static std::string get_time_string();
};
CLOUD_BLEND_DOUBLE_NAMESPACE_END