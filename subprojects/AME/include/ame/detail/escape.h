#pragma once

#include <string>

namespace ame::detail {

inline std::string jsonEscape(const std::string& input) {
    static constexpr char kHex[] = "0123456789abcdef";

    std::string out;
    out.reserve(input.size() + 4);
    for (unsigned char c : input) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    out += "\\u00";
                    out += kHex[(c >> 4) & 0x0f];
                    out += kHex[c & 0x0f];
                } else {
                    out += static_cast<char>(c);
                }
                break;
        }
    }
    return out;
}

inline std::string xmlAttrEscape(const std::string& input) {
    std::string out;
    out.reserve(input.size() + 4);
    for (char c : input) {
        switch (c) {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default:   out += c;        break;
        }
    }
    return out;
}

} // namespace ame::detail
