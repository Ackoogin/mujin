#include <gtest/gtest.h>

#include "fact_chooser.h"
#include "problem_list.h"
#include "project_model.h"
#include "pddl_validator.h"
#include "structural_validator.h"

#include <algorithm>
#include <string>

namespace {

/// A small project: a vehicle that can move between typed places.
ProjectModel makeModel() {
  ProjectModel model;
  model.projectName = "problems";
  model.types = {{"location", "object"},
                 {"sector", "location"},
                 {"robot", "object"}};
  model.predicates.push_back(
      {"at", {{"?r", "robot"}, {"?l", "location"}}, 0.0F, 0.0F});
  model.predicates.push_back({"searched", {{"?s", "sector"}}, 0.0F, 0.0F});

  ActionDef move;
  move.name = "move";
  move.params = {{"?r", "robot"}, {"?from", "location"}, {"?to", "location"}};
  move.preconditions = {{"at", {"?r", "?from"}}};
  move.addEffects = {{"at", {"?r", "?to"}}};
  move.delEffects = {{"at", {"?r", "?from"}}};
  model.actions.push_back(move);

  model.objects = {{"uav1", "robot"},
                   {"base", "location"},
                   {"sector-a", "sector"}};
  return model;
}

const ProblemEntry* firstMentioning(const std::vector<ProblemEntry>& problems,
                                    const std::string& text) {
  const auto it = std::find_if(problems.begin(), problems.end(),
                               [&text](const ProblemEntry& problem) {
                                 return problem.sentence.find(text) !=
                                        std::string::npos;
                               });
  return it == problems.end() ? nullptr : &(*it);
}

}  // namespace

TEST(ProblemList, AWellFormedProjectHasNothingWrongWithIt) {
  const ProjectModel model = makeModel();
  const std::vector<ProblemEntry> problems =
      ProblemList::build(model, StructuralValidator::check(model),
                         ValidationReport{});

  for (const ProblemEntry& problem : problems) {
    EXPECT_FALSE(problem.isError) << problem.sentence;
  }
}

TEST(ProblemList, AProblemAboutAnActionPointsAtThatAction) {
  ProjectModel model = makeModel();
  // The action now requires a fact the project does not have.
  model.actions[0].preconditions.push_back({"refuelled", {"?r"}});

  const std::vector<ProblemEntry> problems =
      ProblemList::build(model, StructuralValidator::check(model),
                         ValidationReport{});

  const ProblemEntry* entry = firstMentioning(problems, "refuelled");
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE(entry->isError);
  EXPECT_TRUE(entry->canReveal());
  EXPECT_EQ(entry->target, ProblemTarget::Action);
  EXPECT_EQ(entry->targetName, "move");
  EXPECT_EQ(entry->targetIndex, 0);
}

TEST(ProblemList, TheListSaysFactRatherThanPredicate) {
  ProjectModel model = makeModel();
  model.predicates.push_back({"at", {{"?r", "robot"}}, 0.0F, 0.0F});

  const std::vector<ProblemEntry> problems =
      ProblemList::build(model, StructuralValidator::check(model),
                         ValidationReport{});

  ASSERT_FALSE(problems.empty());
  for (const ProblemEntry& problem : problems) {
    EXPECT_EQ(problem.sentence.find("predicate"), std::string::npos)
        << problem.sentence;
    EXPECT_EQ(problem.sentence.find("Predicate"), std::string::npos)
        << problem.sentence;
  }
}

TEST(ProblemList, AParserFailureIsASentenceWithTheRawTextKept) {
  const ProjectModel model = makeModel();
  ValidationReport validation;
  ValidationError error;
  error.message = "line 12: unexpected token ')' while reading (:action move";
  error.actionNames = {"move"};
  validation.errors.push_back(error);

  const std::vector<ProblemEntry> problems =
      ProblemList::build(model, StructuralReport{}, validation);

  ASSERT_EQ(problems.size(), 1U);
  EXPECT_NE(problems[0].sentence.find("The action 'move'"), std::string::npos);
  EXPECT_EQ(problems[0].sentence.find("unexpected token"), std::string::npos)
      << "the parser's own words belong in the detail, not the sentence";
  EXPECT_EQ(problems[0].detail, error.message);
  EXPECT_TRUE(problems[0].canReveal());
}

TEST(ProblemList, ErrorsAreListedBeforeWarnings) {
  ProjectModel model = makeModel();
  model.actions[0].preconditions.push_back({"refuelled", {"?r"}});

  const std::vector<ProblemEntry> problems =
      ProblemList::build(model, StructuralValidator::check(model),
                         ValidationReport{});

  bool seen_warning = false;
  for (const ProblemEntry& problem : problems) {
    if (!problem.isError) {
      seen_warning = true;
    } else {
      EXPECT_FALSE(seen_warning) << "an error came after a warning";
    }
  }
}

TEST(ProblemList, AProblemAboutSomethingDeletedCannotBeRevealed) {
  const ProjectModel model = makeModel();
  ValidationReport validation;
  ValidationError error;
  error.message = "something about an action that has since gone";
  error.actionNames = {"land"};
  validation.errors.push_back(error);

  const std::vector<ProblemEntry> problems =
      ProblemList::build(model, StructuralReport{}, validation);

  ASSERT_EQ(problems.size(), 1U);
  EXPECT_FALSE(problems[0].canReveal());
}

// ---------------------------------------------------------------------------
// Choosing a fact rather than typing it
// ---------------------------------------------------------------------------

TEST(FactChooser, OnlyObjectsOfTheRightTypeCanBeChosen) {
  const ProjectModel model = makeModel();

  const std::vector<FactChoice> robots =
      FactChooser::objectsFor(model, "at", 0);
  ASSERT_EQ(robots.size(), model.objects.size());
  for (const FactChoice& choice : robots) {
    EXPECT_EQ(choice.allowed, choice.name == "uav1") << choice.name;
  }
}

TEST(FactChooser, AChoiceThatIsNotAllowedSaysWhy) {
  const ProjectModel model = makeModel();

  const std::vector<FactChoice> choices =
      FactChooser::objectsFor(model, "searched", 0);
  const auto base = std::find_if(choices.begin(), choices.end(),
                                 [](const FactChoice& choice) {
                                   return choice.name == "base";
                                 });
  ASSERT_NE(base, choices.end());
  EXPECT_FALSE(base->allowed);
  EXPECT_NE(base->reason.find("location"), std::string::npos);
  EXPECT_NE(base->reason.find("sector"), std::string::npos);
}

TEST(FactChooser, ASubtypeIsAcceptedWhereItsParentIsWanted) {
  const ProjectModel model = makeModel();

  // A sector is a location, so it may stand where a location is wanted.
  const std::vector<FactChoice> places = FactChooser::objectsFor(model, "at", 1);
  const auto sector = std::find_if(places.begin(), places.end(),
                                   [](const FactChoice& choice) {
                                     return choice.name == "sector-a";
                                   });
  ASSERT_NE(sector, places.end());
  EXPECT_TRUE(sector->allowed);
  EXPECT_TRUE(FactChooser::typeMatches(model, "sector", "location"));
  EXPECT_FALSE(FactChooser::typeMatches(model, "location", "sector"));
}

TEST(FactChooser, AFactThatDoesNotExistCannotBeExpressed) {
  const ProjectModel model = makeModel();

  EXPECT_FALSE(FactChooser::whyNotValid(model, {"flying", {"uav1"}}).empty());
  EXPECT_NE(FactChooser::whyNotValid(model, {"flying", {"uav1"}})
                .find("no fact called"),
            std::string::npos);
  EXPECT_TRUE(FactChooser::whyNotValid(model, {"at", {"uav1", "base"}}).empty());
}

TEST(FactChooser, TheWrongNumberOfThingsIsExplained) {
  const ProjectModel model = makeModel();

  const std::string problem = FactChooser::whyNotValid(model, {"at", {"uav1"}});
  EXPECT_NE(problem.find("involves 2"), std::string::npos) << problem;
  EXPECT_NE(problem.find("1 was given"), std::string::npos) << problem;
}

TEST(FactChooser, TypedTextIsHeldToTheSameRules) {
  const ProjectModel model = makeModel();

  const FactRef bracketed = FactChooser::parse("(at uav1 base)");
  EXPECT_EQ(bracketed.predicateName, "at");
  EXPECT_EQ(bracketed.objectNames, (std::vector<std::string>{"uav1", "base"}));
  EXPECT_TRUE(FactChooser::whyNotValid(model, bracketed).empty());

  const FactRef bare = FactChooser::parse("at uav1 base");
  EXPECT_EQ(bare.objectNames.size(), 2U);

  const FactRef commas = FactChooser::parse("at(uav1, base)");
  EXPECT_EQ(commas.predicateName, "at");
  EXPECT_EQ(commas.objectNames.size(), 2U);

  // The same rules: a typed fact with the wrong type is refused too.
  const FactRef wrong = FactChooser::parse("(searched base)");
  EXPECT_FALSE(FactChooser::whyNotValid(model, wrong).empty());
}

TEST(FactChooser, HowManyThingsAFactInvolves) {
  const ProjectModel model = makeModel();

  EXPECT_EQ(FactChooser::arity(model, "at"), 2U);
  EXPECT_EQ(FactChooser::arity(model, "searched"), 1U);
  EXPECT_EQ(FactChooser::arity(model, "nothing-like-this"), 0U);
}
