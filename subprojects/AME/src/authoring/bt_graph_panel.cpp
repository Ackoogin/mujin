#include "bt_graph_panel.h"

#include "imgui.h"

#include <algorithm>
#include <cctype>
#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kNodeBase = 12000;
constexpr int kInputPinBase = 13000;
constexpr int kOutputPinBase = 14000;
constexpr int kLinkBase = 15000;

struct ParsedTag {
  std::string name;
  std::vector<std::pair<std::string, std::string>> attributes;
  bool closing = false;
  bool selfClosing = false;
  bool special = false;
};

struct RawNode {
  std::string kind;
  std::vector<std::pair<std::string, std::string>> attributes;
  std::vector<int> children;
  int parent = -1;
};

bool isNameChar(char ch) {
  const unsigned char uch = static_cast<unsigned char>(ch);
  return std::isalnum(uch) != 0 || ch == '_' || ch == '-' || ch == ':' || ch == '.';
}

void skipWhitespace(const std::string& text, size_t& pos) {
  while (pos < text.size() &&
         std::isspace(static_cast<unsigned char>(text[pos])) != 0) {
    ++pos;
  }
}

std::string parseName(const std::string& text, size_t& pos) {
  const size_t begin = pos;
  while (pos < text.size() && isNameChar(text[pos])) {
    ++pos;
  }
  return text.substr(begin, pos - begin);
}

bool parseAttributeValue(const std::string& text,
                         size_t& pos,
                         std::string& value,
                         std::string& error) {
  skipWhitespace(text, pos);
  if (pos >= text.size()) {
    error = "missing attribute value";
    return false;
  }

  if (text[pos] == '"' || text[pos] == '\'') {
    const char quote = text[pos++];
    const size_t begin = pos;
    const size_t end = text.find(quote, pos);
    if (end == std::string::npos) {
      error = "unterminated quoted attribute";
      return false;
    }
    value = text.substr(begin, end - begin);
    pos = end + 1U;
    return true;
  }

  const size_t begin = pos;
  while (pos < text.size() &&
         std::isspace(static_cast<unsigned char>(text[pos])) == 0 &&
         text[pos] != '/' && text[pos] != '>') {
    ++pos;
  }
  if (begin == pos) {
    error = "missing attribute value";
    return false;
  }
  value = text.substr(begin, pos - begin);
  return true;
}

bool parseTag(const std::string& text,
              size_t& pos,
              ParsedTag& tag,
              std::string& error) {
  if (pos >= text.size() || text[pos] != '<') {
    error = "expected '<'";
    return false;
  }
  ++pos;

  if (pos < text.size() && text[pos] == '!') {
    if (text.compare(pos, 3U, "!--") == 0) {
      const size_t close = text.find("-->", pos + 3U);
      if (close == std::string::npos) {
        error = "unterminated XML comment";
        return false;
      }
      pos = close + 3U;
      tag.special = true;
      return true;
    }

    const size_t close = text.find('>', pos);
    if (close == std::string::npos) {
      error = "unterminated special XML tag";
      return false;
    }
    pos = close + 1U;
    tag.special = true;
    return true;
  }

  if (pos < text.size() && text[pos] == '?') {
    const size_t close = text.find("?>", pos + 1U);
    if (close == std::string::npos) {
      error = "unterminated XML declaration";
      return false;
    }
    pos = close + 2U;
    tag.special = true;
    return true;
  }

  if (pos < text.size() && text[pos] == '/') {
    tag.closing = true;
    ++pos;
  }

  skipWhitespace(text, pos);
  tag.name = parseName(text, pos);
  if (tag.name.empty()) {
    error = "missing tag name";
    return false;
  }

  if (tag.closing) {
    skipWhitespace(text, pos);
    if (pos >= text.size() || text[pos] != '>') {
      error = "malformed closing tag </" + tag.name + ">";
      return false;
    }
    ++pos;
    return true;
  }

  while (pos < text.size()) {
    skipWhitespace(text, pos);
    if (pos >= text.size()) {
      error = "unterminated tag <" + tag.name + ">";
      return false;
    }
    if (text[pos] == '>') {
      ++pos;
      return true;
    }
    if (text[pos] == '/') {
      ++pos;
      skipWhitespace(text, pos);
      if (pos >= text.size() || text[pos] != '>') {
        error = "expected '>' after '/' in <" + tag.name + ">";
        return false;
      }
      ++pos;
      tag.selfClosing = true;
      return true;
    }

    const std::string attrName = parseName(text, pos);
    if (attrName.empty()) {
      error = "missing attribute name in <" + tag.name + ">";
      return false;
    }
    skipWhitespace(text, pos);
    if (pos >= text.size() || text[pos] != '=') {
      error = "missing '=' after attribute " + attrName;
      return false;
    }
    ++pos;
    std::string attrValue;
    if (!parseAttributeValue(text, pos, attrValue, error)) {
      return false;
    }
    tag.attributes.push_back({attrName, attrValue});
  }

  error = "unterminated tag <" + tag.name + ">";
  return false;
}

std::string attrValue(const std::vector<std::pair<std::string, std::string>>& attrs,
                      const char* name) {
  const auto it = std::find_if(attrs.begin(), attrs.end(), [name](const auto& attr) {
    return attr.first == name;
  });
  return it == attrs.end() ? std::string() : it->second;
}

std::string makeLabel(const RawNode& node) {
  std::string label = node.kind;
  std::string suffix = attrValue(node.attributes, "name");
  if (suffix.empty()) {
    suffix = attrValue(node.attributes, "ID");
  }
  if (suffix.empty()) {
    suffix = attrValue(node.attributes, "id");
  }
  if (!suffix.empty()) {
    label += " ";
    label += suffix;
  }
  return label;
}

ImVec4 nodeColor(const std::string& kind) {
  if (kind == "Sequence" || kind == "ReactiveSequence" || kind == "Parallel" ||
      kind == "ReactiveFallback" || kind == "Fallback") {
    return ImVec4(0.10f, 0.20f, 0.40f, 1.0f);
  }
  if (kind == "CheckWorldPredicate" || kind.find("Check") != std::string::npos) {
    return ImVec4(0.05f, 0.28f, 0.10f, 1.0f);
  }
  if (kind == "SetWorldPredicate" || kind.find("Set") != std::string::npos) {
    return ImVec4(0.20f, 0.10f, 0.40f, 1.0f);
  }
  return ImVec4(0.45f, 0.25f, 0.05f, 1.0f);
}

}  // namespace

BtGraphPanel::BtGraphPanel() {
  ed::Config cfg;
  cfg.SettingsFile = nullptr;
  m_context = ed::CreateEditor(&cfg);
}

BtGraphPanel::~BtGraphPanel() {
  ed::DestroyEditor(m_context);
}

void BtGraphPanel::setXml(const std::string& xml) {
  if (m_context != nullptr) {
    ed::SetCurrentEditor(m_context);
    ed::ClearSelection();
    ed::SetCurrentEditor(nullptr);
  }
  m_nodes.clear();
  m_lastError.clear();
  m_selectedNode = -1;
  m_layoutDone = false;

  if (xml.empty()) {
    return;
  }

  std::vector<RawNode> rawNodes;
  std::vector<int> stack;
  size_t pos = 0;
  while (pos < xml.size()) {
    const size_t open = xml.find('<', pos);
    if (open == std::string::npos) {
      break;
    }
    pos = open;

    ParsedTag tag;
    if (!parseTag(xml, pos, tag, m_lastError)) {
      m_nodes.clear();
      return;
    }
    if (tag.special) {
      continue;
    }

    if (tag.closing) {
      if (stack.empty()) {
        m_lastError = "unexpected closing tag </" + tag.name + ">";
        m_nodes.clear();
        return;
      }
      const int closingIdx = stack.back();
      if (rawNodes[static_cast<size_t>(closingIdx)].kind != tag.name) {
        m_lastError = "mismatched closing tag </" + tag.name + ">";
        m_nodes.clear();
        return;
      }
      stack.pop_back();
      continue;
    }

    RawNode node;
    node.kind = tag.name;
    node.attributes = std::move(tag.attributes);
    node.parent = stack.empty() ? -1 : stack.back();
    const int nodeIdx = static_cast<int>(rawNodes.size());
    rawNodes.push_back(std::move(node));
    if (!stack.empty()) {
      rawNodes[static_cast<size_t>(stack.back())].children.push_back(nodeIdx);
    }
    if (!tag.selfClosing) {
      stack.push_back(nodeIdx);
    }
  }

  if (!stack.empty()) {
    m_lastError =
        "unclosed tag <" + rawNodes[static_cast<size_t>(stack.back())].kind + ">";
    m_nodes.clear();
    return;
  }
  if (rawNodes.empty()) {
    m_lastError = "BT XML contained no elements";
    return;
  }

  int rootIdx = -1;
  for (int i = 0; i < static_cast<int>(rawNodes.size()); ++i) {
    if (rawNodes[static_cast<size_t>(i)].kind == "BehaviorTree" &&
        !rawNodes[static_cast<size_t>(i)].children.empty()) {
      rootIdx = rawNodes[static_cast<size_t>(i)].children.front();
      break;
    }
  }
  if (rootIdx < 0) {
    for (int i = 0; i < static_cast<int>(rawNodes.size()); ++i) {
      if (rawNodes[static_cast<size_t>(i)].parent < 0) {
        rootIdx = i;
        break;
      }
    }
  }
  if (rootIdx < 0) {
    m_lastError = "BT XML has no root element";
    return;
  }

  std::function<int(int, int)> copySubtree = [&](int rawIdx, int parentIdx) {
    const RawNode& raw = rawNodes[static_cast<size_t>(rawIdx)];
    BtNode node;
    node.kind = raw.kind;
    node.label = makeLabel(raw);
    node.attributes = raw.attributes;
    node.parent = parentIdx;
    const int dstIdx = static_cast<int>(m_nodes.size());
    m_nodes.push_back(std::move(node));
    for (const int childRawIdx : raw.children) {
      const int childDstIdx = copySubtree(childRawIdx, dstIdx);
      m_nodes[static_cast<size_t>(dstIdx)].children.push_back(childDstIdx);
    }
    return dstIdx;
  };
  copySubtree(rootIdx, -1);
}

void BtGraphPanel::render() {
  if (m_nodes.empty()) {
    ImGui::TextDisabled("No BT compiled yet");
    if (!m_lastError.empty()) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f),
                         "Parse error: %s",
                         m_lastError.c_str());
    }
    return;
  }

  ed::SetCurrentEditor(m_context);
  ed::Begin("##BtGraph");

  if (!m_layoutDone) {
    std::vector<int> depthCounts;
    for (int i = 0; i < static_cast<int>(m_nodes.size()); ++i) {
      BtNode& node = m_nodes[static_cast<size_t>(i)];
      node.depth = node.parent < 0 ? 0 : m_nodes[static_cast<size_t>(node.parent)].depth + 1;
      if (depthCounts.size() <= static_cast<size_t>(node.depth)) {
        depthCounts.resize(static_cast<size_t>(node.depth + 1), 0);
      }
      node.siblingIdx = depthCounts[static_cast<size_t>(node.depth)]++;
      ed::SetNodePosition(kNodeBase + i,
                          ImVec2(80.0f + static_cast<float>(node.siblingIdx) * 220.0f,
                                 80.0f + static_cast<float>(node.depth) * 130.0f));
    }
    m_layoutDone = true;
  }

  for (int i = 0; i < static_cast<int>(m_nodes.size()); ++i) {
    const BtNode& node = m_nodes[static_cast<size_t>(i)];
    const ImVec4 bg = nodeColor(node.kind);
    ed::PushStyleColor(ed::StyleColor_NodeBg, bg);
    ed::PushStyleColor(ed::StyleColor_NodeBorder, ImVec4(0.75f, 0.75f, 0.80f, 0.95f));

    ed::BeginNode(kNodeBase + i);
    ed::BeginPin(kInputPinBase + i, ed::PinKind::Input);
    ImGui::TextUnformatted("");
    ed::EndPin();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextUnformatted(node.label.c_str());
    if (!node.attributes.empty()) {
      const auto& attr = node.attributes.front();
      ImGui::Text("%s=%s", attr.first.c_str(), attr.second.c_str());
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ed::BeginPin(kOutputPinBase + i, ed::PinKind::Output);
    ImGui::TextUnformatted("");
    ed::EndPin();
    ed::EndNode();

    ed::PopStyleColor(2);
  }

  for (int i = 0; i < static_cast<int>(m_nodes.size()); ++i) {
    const BtNode& node = m_nodes[static_cast<size_t>(i)];
    if (node.parent < 0) {
      continue;
    }
    ed::Link(kLinkBase + i,
             kOutputPinBase + node.parent,
             kInputPinBase + i,
             ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
  }

  ed::Suspend();
  ed::Resume();
  ed::End();

  m_selectedNode = -1;
  const int total = ed::GetSelectedObjectCount();
  if (total > 0) {
    std::vector<ed::NodeId> selectedNodes(static_cast<size_t>(total));
    const int count = ed::GetSelectedNodes(selectedNodes.data(), total);
    for (int i = 0; i < count; ++i) {
      const int id = static_cast<int>(selectedNodes[static_cast<size_t>(i)].Get());
      if (id >= kNodeBase && id < kNodeBase + static_cast<int>(m_nodes.size())) {
        m_selectedNode = id - kNodeBase;
      }
    }
  }

  ed::SetCurrentEditor(nullptr);
}

size_t BtGraphPanel::nodeCount() const {
  return m_nodes.size();
}
