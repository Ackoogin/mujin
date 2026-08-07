#pragma once

#include "project_model.h"

#include <string>

/// \brief Evidence about a mission model, written for somebody not using the tool.
///
/// The report ties a model to the claims made about it: which scenarios were
/// run and how they behaved, which contingencies were checked and what stayed
/// reachable, which facts nothing produces, and which actions are bound to
/// something that can execute them.
///
/// It is generated rather than written because evidence written by hand drifts
/// from the model it describes, and a safety case built on drifted evidence is
/// worse than one with a gap in it, which at least shows.
///
/// It says as plainly what has **not** been checked as what has. A reviewer's
/// first question about any evidence is what it does not cover, and a report
/// that answers only the flattering half of that question is not evidence.
class AssuranceReport {
public:
  /// \brief The whole report as Markdown, ready to read or to attach.
  ///
  /// Running it simulates every scenario and analyses every scenario that
  /// declares a contingency, so it takes as long as those do.
  static std::string generate(const ProjectModel& model,
                              const std::string& generatedOn = "");
};
