#include "type_hierarchy_panel.h"

#include "imgui.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

void TypeHierarchyPanel::render(ProjectModel& model, CommandStack& stack) {
  auto typeExists = [&](const std::string& name) {
    if (name == "object") {
      return true;
    }

    return std::any_of(model.types.begin(), model.types.end(), [&](const TypeDef& type) {
      return type.name == name;
    });
  };

  if (m_newParentName[0] == '\0') {
    std::snprintf(m_newParentName, sizeof(m_newParentName), "%s", "object");
  }

  if (ImGui::CollapsingHeader("Types", ImGuiTreeNodeFlags_DefaultOpen)) {
    std::function<void(const std::string&)> drawSubtree;
    drawSubtree = [&](const std::string& typeName) {
      ImGui::PushID(typeName.c_str());
      const bool open = ImGui::TreeNodeEx(typeName.c_str(), ImGuiTreeNodeFlags_DefaultOpen);

      auto isChildOfCurrentType = [&](const TypeDef& type) {
        if (typeName == "object") {
          return type.parent.empty() || type.parent == "object";
        }

        return type.parent == typeName;
      };

      const bool hasChildType = std::any_of(model.types.begin(), model.types.end(), isChildOfCurrentType);
      const bool hasObject =
        std::any_of(model.objects.begin(), model.objects.end(), [&](const ObjectDef& object) {
          return object.type == typeName;
        });
      const bool canDelete = typeName != "object" && !hasChildType && !hasObject;

      ImGui::SameLine();
      if (!canDelete) {
        ImGui::BeginDisabled();
        ImGui::SmallButton("x");
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
          ImGui::SetTooltip("Type in use");
        }
      } else if (ImGui::SmallButton("x")) {
        stack.execute(model, "Delete type", [typeName](ProjectModel& m) {
          m.types.erase(
            std::remove_if(m.types.begin(),
                           m.types.end(),
                           [&](const TypeDef& type) { return type.name == typeName; }),
            m.types.end());
        });
      }

      if (open) {
        std::vector<std::string> childTypeNames;
        for (const TypeDef& type : model.types) {
          if (isChildOfCurrentType(type)) {
            childTypeNames.push_back(type.name);
          }
        }

        for (const std::string& childTypeName : childTypeNames) {
          drawSubtree(childTypeName);
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    };

    drawSubtree("object");

    ImGui::InputText("Name##type", m_newTypeName, sizeof(m_newTypeName));
    ImGui::InputText("Parent##type", m_newParentName, sizeof(m_newParentName));
    if (ImGui::Button("Add Type")) {
      const std::string name = m_newTypeName;
      const std::string parent = m_newParentName;

      if (name.empty()) {
        m_validationMsg = "Type name is required";
      } else if (typeExists(name)) {
        m_validationMsg = "Type already exists";
      } else if (!typeExists(parent)) {
        m_validationMsg = "Parent type does not exist";
      } else {
        stack.execute(model, "Add type", [name, parent](ProjectModel& m) {
          m.types.push_back({name, parent});
        });
        m_newTypeName[0] = '\0';
        m_newParentName[0] = '\0';
        m_validationMsg.clear();
      }
    }
  }

  if (ImGui::CollapsingHeader("Objects", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("Objects##table",
                          3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Name");
      ImGui::TableSetupColumn("Type");
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 24.0F);
      ImGui::TableHeadersRow();

      for (std::size_t i = 0; i < model.objects.size();) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(model.objects[i].name.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(model.objects[i].type.c_str());
        ImGui::TableSetColumnIndex(2);
        const std::string deleteButtonId = "x##obj" + std::to_string(i);
        if (ImGui::SmallButton(deleteButtonId.c_str())) {
          stack.execute(model, "Delete object", [i](ProjectModel& m) {
            m.objects.erase(m.objects.begin() + static_cast<std::ptrdiff_t>(i));
          });
          continue;
        }
        ++i;
      }

      ImGui::EndTable();
    }

    ImGui::InputText("Name##obj", m_newObjName, sizeof(m_newObjName));
    ImGui::InputText("Type##obj", m_newObjType, sizeof(m_newObjType));
    if (ImGui::Button("Add Object")) {
      const std::string name = m_newObjName;
      const std::string type = m_newObjType;
      const bool duplicateName =
        std::any_of(model.objects.begin(), model.objects.end(), [&](const ObjectDef& object) {
          return object.name == name;
        });

      if (name.empty() || type.empty()) {
        m_validationMsg = "Object name and type are required";
      } else if (duplicateName) {
        m_validationMsg = "Object already exists";
      } else if (!typeExists(type)) {
        m_validationMsg = "Object type does not exist";
      } else {
        stack.execute(model, "Add object", [name, type](ProjectModel& m) {
          m.objects.push_back({name, type});
        });
        m_newObjName[0] = '\0';
        m_newObjType[0] = '\0';
        m_validationMsg.clear();
      }
    }
  }

  if (!m_validationMsg.empty()) {
    ImGui::TextColored({1.0F, 0.3F, 0.3F, 1.0F}, "%s", m_validationMsg.c_str());
  }
}
