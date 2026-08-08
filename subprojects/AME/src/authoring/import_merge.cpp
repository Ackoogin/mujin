#include "import_merge.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

bool sameParameters(const std::vector<Parameter>& a,
                    const std::vector<Parameter>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].name != b[i].name || a[i].type != b[i].type ||
        a[i].eitherTypes != b[i].eitherTypes) {
      return false;
    }
  }
  return true;
}

bool sameReferences(const std::vector<EffectRef>& a,
                    const std::vector<EffectRef>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].predicateName != b[i].predicateName ||
        a[i].argNames != b[i].argNames || a[i].negated != b[i].negated ||
        a[i].alternative != b[i].alternative) {
      return false;
    }
  }
  return true;
}

bool sameFact(const PredicateDef& a, const PredicateDef& b) {
  return a.name == b.name && a.confirmed == b.confirmed &&
         sameParameters(a.params, b.params);
}

bool sameAction(const ActionDef& a, const ActionDef& b) {
  return a.name == b.name && sameParameters(a.params, b.params) &&
         sameReferences(a.preconditions, b.preconditions) &&
         a.hasConditionExpression == b.hasConditionExpression &&
         (!a.hasConditionExpression ||
          nlohmann::json(a.conditionExpression) ==
              nlohmann::json(b.conditionExpression)) &&
         sameReferences(a.addEffects, b.addEffects) &&
         sameReferences(a.delEffects, b.delEffects);
}

std::string describeLoss(const ActionDef& action) {
  const size_t conditions = actionConditionFacts(action).size();
  const size_t outcomes = action.addEffects.size() + action.delEffects.size();
  return std::to_string(conditions) + " condition" +
         (conditions == 1U ? "" : "s") + " and " + std::to_string(outcomes) +
         " outcome" + (outcomes == 1U ? "" : "s");
}

std::string describeLoss(const PredicateDef& fact) {
  return std::to_string(fact.params.size()) + " name" +
         (fact.params.size() == 1U ? "" : "s") + " it involves";
}

template <typename T, typename Match>
const T* findBy(const std::vector<T>& items, const Match& match) {
  const auto it = std::find_if(items.begin(), items.end(), match);
  return it == items.end() ? nullptr : &(*it);
}

}  // namespace

bool MergeChoices::replaces(MergeKind kind) const {
  switch (kind) {
  case MergeKind::Type:
    return replaceTypes;
  case MergeKind::Fact:
    return replaceFacts;
  case MergeKind::Action:
    return replaceActions;
  case MergeKind::Object:
    return replaceObjects;
  }
  return false;
}

size_t MergePlan::countAdded() const {
  return static_cast<size_t>(
      std::count_if(items.begin(), items.end(), [](const MergeItem& item) {
        return item.disposition == MergeDisposition::Added;
      }));
}

size_t MergePlan::countReplaced() const {
  return static_cast<size_t>(
      std::count_if(items.begin(), items.end(), [](const MergeItem& item) {
        return item.disposition == MergeDisposition::Replaced;
      }));
}

size_t MergePlan::countUnchanged() const {
  return static_cast<size_t>(
      std::count_if(items.begin(), items.end(), [](const MergeItem& item) {
        return item.disposition == MergeDisposition::Unchanged;
      }));
}

std::vector<std::string> MergePlan::wouldOverwrite() const {
  std::vector<std::string> names;
  for (const MergeItem& item : items) {
    if (item.disposition == MergeDisposition::Replaced) {
      names.push_back(item.name);
    }
  }
  return names;
}

MergePlan ImportMerge::plan(const ProjectModel& current,
                            const ProjectModel& incoming) {
  MergePlan plan;

  for (const TypeDef& type : incoming.types) {
    const TypeDef* existing = findBy(current.types, [&type](const TypeDef& t) {
      return t.name == type.name;
    });
    MergeItem item;
    item.kind = MergeKind::Type;
    item.name = type.name;
    if (existing == nullptr) {
      item.disposition = MergeDisposition::Added;
    } else if (existing->parent == type.parent) {
      item.disposition = MergeDisposition::Unchanged;
    } else {
      item.disposition = MergeDisposition::Replaced;
      item.whatWouldBeLost = "its parent, " + existing->parent;
    }
    plan.items.push_back(std::move(item));
  }

  for (const PredicateDef& fact : incoming.predicates) {
    const PredicateDef* existing =
        findBy(current.predicates, [&fact](const PredicateDef& f) {
          return f.name == fact.name;
        });
    MergeItem item;
    item.kind = MergeKind::Fact;
    item.name = fact.name;
    if (existing == nullptr) {
      item.disposition = MergeDisposition::Added;
    } else if (sameFact(*existing, fact)) {
      item.disposition = MergeDisposition::Unchanged;
    } else {
      item.disposition = MergeDisposition::Replaced;
      item.whatWouldBeLost = describeLoss(*existing);
    }
    plan.items.push_back(std::move(item));
  }

  for (const ActionDef& action : incoming.actions) {
    const ActionDef* existing =
        findBy(current.actions, [&action](const ActionDef& a) {
          return a.name == action.name;
        });
    MergeItem item;
    item.kind = MergeKind::Action;
    item.name = action.name;
    if (existing == nullptr) {
      item.disposition = MergeDisposition::Added;
    } else if (sameAction(*existing, action)) {
      item.disposition = MergeDisposition::Unchanged;
    } else {
      item.disposition = MergeDisposition::Replaced;
      item.whatWouldBeLost = describeLoss(*existing);
    }
    plan.items.push_back(std::move(item));
  }

  for (const ObjectDef& object : incoming.objects) {
    const ObjectDef* existing =
        findBy(current.objects, [&object](const ObjectDef& o) {
          return o.name == object.name;
        });
    MergeItem item;
    item.kind = MergeKind::Object;
    item.name = object.name;
    if (existing == nullptr) {
      item.disposition = MergeDisposition::Added;
    } else if (existing->type == object.type) {
      item.disposition = MergeDisposition::Unchanged;
    } else {
      item.disposition = MergeDisposition::Replaced;
      item.whatWouldBeLost = "its type, " + existing->type;
    }
    plan.items.push_back(std::move(item));
  }
  for (const ObjectDef& constant : incoming.constants) {
    const ObjectDef* existing =
        findBy(current.constants, [&constant](const ObjectDef& item) {
          return item.name == constant.name;
        });
    MergeItem item;
    item.kind = MergeKind::Object;
    item.name = constant.name;
    if (existing == nullptr) {
      item.disposition = MergeDisposition::Added;
    } else if (existing->type == constant.type) {
      item.disposition = MergeDisposition::Unchanged;
    } else {
      item.disposition = MergeDisposition::Replaced;
      item.whatWouldBeLost = "its type, " + existing->type;
    }
    plan.items.push_back(std::move(item));
  }

  return plan;
}

ProjectModel ImportMerge::apply(const ProjectModel& current,
                                const ProjectModel& incoming,
                                const MergeChoices& choices) {
  ProjectModel merged = current;

  for (const TypeDef& type : incoming.types) {
    const auto it = std::find_if(merged.types.begin(), merged.types.end(),
                                 [&type](const TypeDef& t) {
                                   return t.name == type.name;
                                 });
    if (it == merged.types.end()) {
      merged.types.push_back(type);
    } else if (choices.replaces(MergeKind::Type)) {
      *it = type;
    }
  }

  for (const PredicateDef& fact : incoming.predicates) {
    const auto it =
        std::find_if(merged.predicates.begin(), merged.predicates.end(),
                     [&fact](const PredicateDef& f) {
                       return f.name == fact.name;
                     });
    if (it == merged.predicates.end()) {
      merged.predicates.push_back(fact);
    } else if (choices.replaces(MergeKind::Fact)) {
      // The position on the canvas belongs to this project, not to the file
      // being imported, so it stays where the user put it.
      const float x = it->posX;
      const float y = it->posY;
      *it = fact;
      it->posX = x;
      it->posY = y;
    }
  }

  for (const ActionDef& action : incoming.actions) {
    const auto it = std::find_if(merged.actions.begin(), merged.actions.end(),
                                 [&action](const ActionDef& a) {
                                   return a.name == action.name;
                                 });
    if (it == merged.actions.end()) {
      merged.actions.push_back(action);
    } else if (choices.replaces(MergeKind::Action)) {
      const float x = it->posX;
      const float y = it->posY;
      // The behaviour-tree binding and the run settings are this project's
      // work, and the imported PDDL says nothing about either, so keeping them
      // loses nothing and saves rebinding every action after every import.
      const BtBinding binding = it->btBinding;
      const SimulationSettings simulation = it->simulation;
      *it = action;
      it->posX = x;
      it->posY = y;
      it->btBinding = binding;
      it->simulation = simulation;
    }
  }

  for (const ObjectDef& object : incoming.objects) {
    const auto it = std::find_if(merged.objects.begin(), merged.objects.end(),
                                 [&object](const ObjectDef& o) {
                                   return o.name == object.name;
                                 });
    if (it == merged.objects.end()) {
      merged.objects.push_back(object);
    } else if (choices.replaces(MergeKind::Object)) {
      *it = object;
    }
  }
  for (const ObjectDef& constant : incoming.constants) {
    const auto it = std::find_if(
        merged.constants.begin(), merged.constants.end(),
        [&constant](const ObjectDef& item) {
          return item.name == constant.name;
        });
    if (it == merged.constants.end()) {
      merged.constants.push_back(constant);
    } else if (choices.replaces(MergeKind::Object)) {
      *it = constant;
    }
  }

  return merged;
}

void ImportMerge::layoutByRelationships(ProjectModel& model) {
  constexpr float kColumnWidth = 340.0F;
  constexpr float kRowHeight = 130.0F;
  constexpr float kStartX = 60.0F;
  constexpr float kStartY = 60.0F;

  // Each action goes in a column with the facts it uses beside it, so what an
  // action has to do with a fact is visible before anything is moved by hand.
  std::set<std::string> placed_facts;
  float column = 0.0F;

  for (ActionDef& action : model.actions) {
    action.posX = kStartX + column * kColumnWidth;
    action.posY = kStartY;

    float row = 1.0F;
    const auto place = [&](const std::vector<EffectRef>& references) {
      for (const EffectRef& reference : references) {
        if (placed_facts.count(reference.predicateName) != 0) {
          continue;
        }
        const auto fact =
            std::find_if(model.predicates.begin(), model.predicates.end(),
                         [&reference](const PredicateDef& predicate) {
                           return predicate.name == reference.predicateName;
                         });
        if (fact == model.predicates.end()) {
          continue;
        }
        fact->posX = kStartX + column * kColumnWidth + 40.0F;
        fact->posY = kStartY + row * kRowHeight;
        placed_facts.insert(reference.predicateName);
        row += 1.0F;
      }
    };
    place(actionConditionFacts(action));
    place(action.addEffects);
    place(action.delEffects);
    column += 1.0F;
  }

  // Facts no action mentions sit together below, where they read as what they
  // are: something nothing in this domain does anything with.
  float orphan = 0.0F;
  for (PredicateDef& fact : model.predicates) {
    if (placed_facts.count(fact.name) != 0) {
      continue;
    }
    fact.posX = kStartX + orphan * 240.0F;
    fact.posY = kStartY + 6.0F * kRowHeight;
    orphan += 1.0F;
  }
}
