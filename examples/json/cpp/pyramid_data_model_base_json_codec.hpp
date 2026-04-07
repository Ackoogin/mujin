// Auto-generated JSON codec — do not edit
// Backend: json | Namespace: pyramid::data_model::base::json_codec
#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace pyramid::data_model::base::json_codec {

struct Angle {
    double radians = 0.0;
};

struct Length {
    double meters = 0.0;
};

struct Timestamp {
    Timestamp value = {};
};

struct Identifier {
    std::string value = {};
};

struct Speed {
    double meters_per_second = 0.0;
};

struct Percentage {
    double value = 0.0;
};

std::string toJson(const Angle& msg);
Angle angleFromJson(const std::string& s);
std::string toJson(const Length& msg);
Length lengthFromJson(const std::string& s);
std::string toJson(const Timestamp& msg);
Timestamp timestampFromJson(const std::string& s);
std::string toJson(const Identifier& msg);
Identifier identifierFromJson(const std::string& s);
std::string toJson(const Speed& msg);
Speed speedFromJson(const std::string& s);
std::string toJson(const Percentage& msg);
Percentage percentageFromJson(const std::string& s);

} // namespace pyramid::data_model::base::json_codec
