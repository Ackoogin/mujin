#include "owp_frame.hpp"

#include <cctype>
#include <stdexcept>

namespace pyramid::owp {
namespace {
bool delimiter(char c) { return c == ' ' || c == '\t'; }
size_t skip(const std::string& s, size_t p) { while (p < s.size() && delimiter(s[p])) ++p; return p; }
std::string token(const std::string& s, size_t& p) {
  p = skip(s, p); const size_t start = p;
  while (p < s.size() && !delimiter(s[p])) ++p;
  return s.substr(start, p - start);
}
std::string remainder(const std::string& s, size_t p) { return s.substr(skip(s, p)); }
std::string escape(const std::string& value) {
  std::string out;
  for (char c : value) { if (c == '"' || c == '\\') out += '\\'; out += c; }
  return out;
}
void requireIdentifier(const std::string& value, const char* name) {
  if (!isIdentifier(value)) throw std::invalid_argument(std::string("invalid ") + name);
}

class Json {
 public:
  explicit Json(const std::string& text) : text_(text) {}
  std::string stringField(const std::string& key, bool required = true) {
    const size_t p = find(key, required); if (p == std::string::npos) return {};
    return stringAt(p);
  }
  bool boolField(const std::string& key, bool default_value) {
    const size_t p = find(key, false); if (p == std::string::npos) return default_value;
    if (text_.compare(p, 4, "true") == 0) return true;
    if (text_.compare(p, 5, "false") == 0) return false;
    throw std::invalid_argument("invalid JSON boolean");
  }
  std::vector<std::string> arrayField(const std::string& key, bool required = true) {
    size_t p = find(key, required); if (p == std::string::npos) return {}; p = ws(p);
    if (p == text_.size() || text_[p++] != '[') throw std::invalid_argument("invalid JSON array");
    std::vector<std::string> result; p = ws(p);
    while (p < text_.size() && text_[p] != ']') { result.push_back(stringAt(p)); p = ws(cursor_); if (p < text_.size() && text_[p] == ',') { ++p; p = ws(p); } else break; }
    if (p == text_.size() || text_[p] != ']') throw std::invalid_argument("invalid JSON array");
    return result;
  }
  Json objectField(const std::string& key) {
    size_t p = ws(find(key, true)); if (p == text_.size() || text_[p] != '{') throw std::invalid_argument("invalid JSON object");
    const size_t end = matching(p); return Json(text_.substr(p, end - p + 1));
  }
  std::map<std::string, std::string> stringMapField(const std::string& key) {
    const size_t value = find(key, false);
    if (value == std::string::npos) return {};
    Json object = objectField(key); std::map<std::string, std::string> result;
    size_t p = object.ws(1);
    while (p < object.text_.size() && object.text_[p] != '}') {
      const std::string name = object.stringAt(p); p = object.ws(object.cursor_);
      if (p == object.text_.size() || object.text_[p++] != ':') throw std::invalid_argument("invalid JSON map");
      result.emplace(name, object.stringAt(p)); p = object.ws(object.cursor_);
      if (p < object.text_.size() && object.text_[p] == ',') p = object.ws(p + 1); else break;
    }
    return result;
  }
 private:
  size_t ws(size_t p) const { while (p < text_.size() && std::isspace(static_cast<unsigned char>(text_[p]))) ++p; return p; }
  size_t find(const std::string& key, bool required) const {
    const std::string quoted = "\"" + key + "\""; size_t p = text_.find(quoted);
    if (p == std::string::npos) { if (required) throw std::invalid_argument("missing JSON field: " + key); return p; }
    p = text_.find(':', p + quoted.size()); if (p == std::string::npos) throw std::invalid_argument("invalid JSON object"); return ws(p + 1);
  }
  size_t matching(size_t p) const { int depth = 0; bool quote = false; for (; p < text_.size(); ++p) { if (text_[p] == '"' && (p == 0 || text_[p - 1] != '\\')) quote = !quote; if (!quote) { if (text_[p] == '{') ++depth; if (text_[p] == '}' && --depth == 0) return p; } } throw std::invalid_argument("unclosed JSON object"); }
  std::string stringAt(size_t p) { p = ws(p); if (p == text_.size() || text_[p] != '"') throw std::invalid_argument("expected JSON string"); std::string out; for (++p; p < text_.size(); ++p) { if (text_[p] == '"') { cursor_ = p + 1; return out; } if (text_[p] == '\\' && ++p < text_.size()) { const char c = text_[p]; out += c == 'n' ? '\n' : c == 'r' ? '\r' : c == 't' ? '\t' : c; } else out += text_[p]; } throw std::invalid_argument("unclosed JSON string"); }
  std::string text_; size_t cursor_ = 0;
};
Info parseInfo(const std::string& text) {
  Json json(text); Info info; info.version = json.stringField("version"); info.server_id = json.stringField("server_id"); info.system_label = json.stringField("system_label"); info.connect_urls = json.arrayField("connect_urls", false);
  requireIdentifier(info.server_id, "server_id"); Json ids = json.objectField("uuids"); info.uuids.system = ids.stringField("system"); info.uuids.service = ids.stringField("service"); info.uuids.subsystem = ids.stringField("subsystem", false); info.uuids.capabilities = ids.stringMapField("capabilities"); info.uuids.components = ids.stringMapField("components"); return info;
}
}

bool isIdentifier(const std::string& value) { if (value.empty()) return false; for (unsigned char c : value) if (!(std::isalnum(c) || c == '_' || c == '-' || c == '.')) return false; return true; }
std::string serializeInit(const Init& init) { requireIdentifier(init.service_id, "service_id"); if (init.versions.empty()) throw std::invalid_argument("versions must not be empty"); std::string versions; for (size_t i = 0; i < init.versions.size(); ++i) { if (i) versions += ','; versions += '"' + escape(init.versions[i]) + '"'; } return "INIT {\"versions\":[" + versions + "],\"schema\":\"" + escape(init.schema) + "\",\"service_id\":\"" + escape(init.service_id) + "\",\"verbose\":" + (init.verbose ? "true}" : "false}"); }
std::string serializeSubscribe(const std::string& sid, const std::string& name, const std::string& topic, const std::string& group) { requireIdentifier(sid, "subscription_id"); requireIdentifier(topic, "topic"); if (name.empty()) throw std::invalid_argument("empty message_name"); if (!group.empty()) requireIdentifier(group, "group"); return "SUB " + sid + " " + name + " " + topic + (group.empty() ? "" : " " + group); }
std::string serializeUnsubscribe(const std::string& sid) { requireIdentifier(sid, "subscription_id"); return "UNSUB " + sid; }
std::string serializePublish(const std::string& topic, const std::string& body) { requireIdentifier(topic, "topic"); return "PUB " + topic + " " + body; }
ServerFrame parseServerFrame(const std::string& frame) { size_t p = 0; const std::string op = token(frame, p); if (op == "INFO") return {ServerFrameType::Info, parseInfo(remainder(frame, p))}; if (op == "MSG") { ServerFrame f; f.type = ServerFrameType::Message; f.subscription_id = token(frame, p); requireIdentifier(f.subscription_id, "subscription_id"); f.body = remainder(frame, p); return f; } if (op == "+OK" && remainder(frame, p).empty()) return {ServerFrameType::Ok}; if (op == "-ERR") { ServerFrame f; f.type = ServerFrameType::Error; f.error_name = token(frame, p); f.error_details = remainder(frame, p); if (f.error_name.empty()) throw std::invalid_argument("missing error name"); return f; } throw std::invalid_argument("unknown or malformed server frame"); }
}  // namespace pyramid::owp
