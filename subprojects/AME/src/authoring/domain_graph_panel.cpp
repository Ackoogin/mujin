#include "domain_graph_panel.h"

#include "imgui.h"

DomainGraphPanel::DomainGraphPanel() {
  m_context = ed::CreateEditor(nullptr);
}

DomainGraphPanel::~DomainGraphPanel() {
  ed::DestroyEditor(m_context);
}

void DomainGraphPanel::render() {
  ed::SetCurrentEditor(m_context);
  ed::Begin("DomainGraphCanvas");

  ed::BeginNode(1);
  ImGui::Text("Action (placeholder)");
  ed::BeginPin(2, ed::PinKind::Input);
  ImGui::Text("-> In");
  ed::EndPin();
  ed::BeginPin(3, ed::PinKind::Output);
  ImGui::Text("Out ->");
  ed::EndPin();
  ed::EndNode();

  ed::End();
  ed::SetCurrentEditor(nullptr);
}
