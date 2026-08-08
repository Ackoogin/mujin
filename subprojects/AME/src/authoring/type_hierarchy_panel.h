#pragma once
#include "command_stack.h"
#include "project_model.h"
#include <string>

class TypeHierarchyPanel {
public:
    void render(ProjectModel& model, CommandStack& stack);
private:
    std::string m_renameTypeName;
    char m_renameTypeInput[64] = {};
    bool m_openRenamePopup = false;
    char m_newTypeName[64] = {};
    char m_newParentName[64] = {};
    char m_newObjName[64] = {};
    char m_newObjType[64] = {};
    std::string m_validationMsg;
};
