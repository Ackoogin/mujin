#pragma once
#include "project_model.h"
#include <string>

class TypeHierarchyPanel {
public:
    void render(ProjectModel& model);
private:
    char m_newTypeName[64] = {};
    char m_newParentName[64] = {};
    char m_newObjName[64] = {};
    char m_newObjType[64] = {};
    std::string m_validationMsg;
};
