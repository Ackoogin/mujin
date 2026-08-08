include(GoogleTest)

set(AME_SOURCE_DIR "${AME_ROOT_DIR}")

add_executable(test_world_model test_world_model.cpp)
target_link_libraries(test_world_model PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_world_model DISCOVERY_MODE PRE_TEST)

# Keeps AME's own contract tree honest: every file AME needs is present, its
# imports resolve within the tree, and the files shared with PYRAMID's proof
# contracts have not drifted apart. Needs no PYRAMID target, so it runs even in
# a build with the proof layer switched off — but it does need PYRAMID's proof
# contracts on disk to compare against, so a standalone AME build leaves it out.
if(EXISTS "${PYRAMID_ROOT_DIR}/proofs/contracts/proto")
    add_executable(test_contract_tree test_contract_tree.cpp)
    target_link_libraries(test_contract_tree PRIVATE GTest::gtest_main)
    target_compile_definitions(test_contract_tree
        PRIVATE
            "AME_CONTRACT_PROTO_DIR=\"${AME_CONTRACT_PROTO_DIR}\""
            "PYRAMID_PROOF_PROTO_DIR=\"${PYRAMID_ROOT_DIR}/proofs/contracts/proto\""
    )
    gtest_discover_tests(test_contract_tree DISCOVERY_MODE PRE_TEST)
endif()

add_executable(test_action_registry test_action_registry.cpp)
target_link_libraries(test_action_registry PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_action_registry DISCOVERY_MODE PRE_TEST)

# Guards the third-party sources checked in under external/, which an
# air-gapped build compiles instead of downloading anything.
add_executable(test_vendored_dependencies test_vendored_dependencies.cpp)
target_link_libraries(test_vendored_dependencies
    PRIVATE nlohmann_json::nlohmann_json GTest::gtest_main)
target_compile_definitions(test_vendored_dependencies
    PRIVATE "AME_EXTERNAL_DIR=\"${AME_EXTERNAL_DIR}\"")
gtest_discover_tests(test_vendored_dependencies DISCOVERY_MODE PRE_TEST)

# ---------------------------------------------------------------------------
# Tests that cover the PCL component layer and the PYRAMID bridge. They link
# ame_pcl_components, which a standalone AME build does not produce, so they
# are built only when that layer is.
# ---------------------------------------------------------------------------
if(AME_BUILD_PCL_COMPONENTS)

add_executable(test_autonomy_backend test_autonomy_backend.cpp)
target_link_libraries(test_autonomy_backend
  PRIVATE
    ame_pcl_components
    GTest::gtest_main
)
gtest_discover_tests(test_autonomy_backend DISCOVERY_MODE PRE_TEST)

add_executable(test_execution_sink test_execution_sink.cpp)
target_link_libraries(test_execution_sink
  PRIVATE
    ame_pcl_components
    GTest::gtest_main
)
gtest_discover_tests(test_execution_sink DISCOVERY_MODE PRE_TEST)


if(UNMANNED_BUILD_PYRAMID)
    add_executable(test_ame_autonomy_contract
        test_ame_autonomy_contract.cpp
        ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_autonomy_backend_provided.cpp
    )
    add_dependencies(test_ame_autonomy_contract
        pyramid_cpp_bindings_codegen
        pyramid_codec_json_autonomy_backend
    )
    target_link_libraries(test_ame_autonomy_contract
        PRIVATE
            ame_pcl_components
            GTest::gtest_main
    )
    target_compile_definitions(test_ame_autonomy_contract PRIVATE
        AME_AUTONOMY_JSON_CODEC_PLUGIN_PATH="$<TARGET_FILE:pyramid_codec_json_autonomy_backend>"
    )
    # This test links PYRAMID libraries that are built as DLLs and left in
    # PYRAMID's own output directory. Windows looks for a DLL beside the
    # executable rather than anywhere in the build tree, so without this copy
    # the test cannot start, and CTest fails while listing the tests in it.
    # Linux finds the libraries through the run path and needs no copy.
    if(WIN32)
        add_custom_command(TARGET test_ame_autonomy_contract POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:test_ame_autonomy_contract>
                    $<TARGET_FILE_DIR:test_ame_autonomy_contract>
            COMMAND_EXPAND_LISTS
            COMMENT "Copying the DLLs test_ame_autonomy_contract needs beside it"
        )
    endif()
    gtest_discover_tests(test_ame_autonomy_contract DISCOVERY_MODE PRE_TEST)

    if(AME_BUILD_AGRA_MA_BRIDGE)
        add_executable(test_agra_ma_bridge test_agra_ma_bridge.cpp)
        add_dependencies(test_agra_ma_bridge
            pyramid_agra_p3_bindings_codegen)
        target_link_libraries(test_agra_ma_bridge
            PRIVATE
                ame_pcl_components
                GTest::gtest_main
        )
        gtest_discover_tests(test_agra_ma_bridge DISCOVERY_MODE PRE_TEST)

        add_executable(test_agra_ma_grounding_component
            test_agra_ma_grounding_component.cpp)
        add_dependencies(test_agra_ma_grounding_component
            pyramid_agra_p3_bindings_codegen
            pyramid_cpp_bindings_codegen)
        target_link_libraries(test_agra_ma_grounding_component
            PRIVATE
                ame_pcl_components
                GTest::gtest_main
        )
        gtest_discover_tests(test_agra_ma_grounding_component DISCOVERY_MODE PRE_TEST)

        add_executable(test_agra_ma_grounding_port_round_trip
            test_agra_ma_grounding_port_round_trip.cpp
            ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_agra_ma_grounding_provided.cpp)
        add_dependencies(test_agra_ma_grounding_port_round_trip
            pyramid_agra_p3_bindings_codegen
            pyramid_cpp_bindings_codegen)
        # pyramid_services_agra_ma_grounding_provided.cpp references
        # TaskGrounding/PlanGrounding/etc without including the header that
        # declares them (see agra_ma_grounding_component.hpp's comment on
        # the same gap) -- force it in here too, since this generated .cpp
        # is compiled standalone, not through that header.
        set_source_files_properties(
            ${PYRAMID_CPP_BINDINGS_DIR}/pyramid_services_agra_ma_grounding_provided.cpp
            PROPERTIES COMPILE_OPTIONS
            "-include;${PYRAMID_CPP_BINDINGS_DIR}/pyramid_components_agra_ma_grounding_services_provided_types.hpp")
        target_link_libraries(test_agra_ma_grounding_port_round_trip
            PRIVATE
                ame_pcl_components
                GTest::gtest_main
        )
        target_compile_definitions(test_agra_ma_grounding_port_round_trip PRIVATE
            AGRA_MA_GROUNDING_JSON_CODEC_PLUGIN_PATH="$<TARGET_FILE:pyramid_codec_json_agra_ma_grounding>"
        )
        gtest_discover_tests(test_agra_ma_grounding_port_round_trip DISCOVERY_MODE PRE_TEST)
    endif()
endif()

endif()  # AME_BUILD_PCL_COMPONENTS

# Header-only seam between AME's native identifiers and the OMS-JSON wire
# form; needs no A-GRA bindings, so it is not gated on the MA bridge.
add_executable(test_agra_oms_uuid test_agra_oms_uuid.cpp)
target_link_libraries(test_agra_oms_uuid PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_agra_oms_uuid DISCOVERY_MODE PRE_TEST)

# Header-only seam between AME's native identifiers and the "native JSON"
# pivot dialect ame_py.cpp's protobuf<->native conversion produces; needs no
# A-GRA bindings or Python, so it is not gated on the MA bridge either.
add_executable(test_agra_native_json_uuid test_agra_native_json_uuid.cpp)
target_link_libraries(test_agra_native_json_uuid PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_agra_native_json_uuid DISCOVERY_MODE PRE_TEST)

add_executable(test_plan_compiler test_plan_compiler.cpp)
target_link_libraries(test_plan_compiler PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_plan_compiler DISCOVERY_MODE PRE_TEST)

add_executable(test_pddl_parser test_pddl_parser.cpp)
target_link_libraries(test_pddl_parser PRIVATE ame_core GTest::gtest_main)
target_compile_definitions(test_pddl_parser PRIVATE "PROJECT_SOURCE_DIR=\"${AME_SOURCE_DIR}\"")
gtest_discover_tests(test_pddl_parser DISCOVERY_MODE PRE_TEST)

add_executable(test_planner test_planner.cpp)
target_link_libraries(test_planner PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_planner DISCOVERY_MODE PRE_TEST)

# The contingency search is shared by the verifier, the authoring tool and the
# generated assurance report, so it is tested once here rather than in each.
add_executable(test_contingency_search test_contingency_search.cpp)
target_link_libraries(test_contingency_search PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_contingency_search DISCOVERY_MODE PRE_TEST)

add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_integration DISCOVERY_MODE PRE_TEST)

add_executable(test_e2e_pipeline test_e2e_pipeline.cpp)
target_link_libraries(test_e2e_pipeline PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_e2e_pipeline DISCOVERY_MODE PRE_TEST)

add_executable(test_observability test_observability.cpp)
target_link_libraries(test_observability PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_observability DISCOVERY_MODE PRE_TEST)

add_executable(test_e2e_spatial_routing test_e2e_spatial_routing.cpp)
target_link_libraries(test_e2e_spatial_routing PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_e2e_spatial_routing DISCOVERY_MODE PRE_TEST)

add_executable(test_multi_agent test_multi_agent.cpp)
target_link_libraries(test_multi_agent PRIVATE ame_core GTest::gtest_main)
gtest_discover_tests(test_multi_agent DISCOVERY_MODE PRE_TEST)

if(AME_BUILD_PCL_COMPONENTS)
    add_executable(test_extensions test_extensions.cpp)
    target_link_libraries(test_extensions PRIVATE ame_pcl_components GTest::gtest_main)
    gtest_discover_tests(test_extensions DISCOVERY_MODE PRE_TEST)

    add_executable(test_world_model_component test_world_model_component.cpp)
    target_link_libraries(test_world_model_component PRIVATE ame_pcl_components GTest::gtest_main)
    gtest_discover_tests(test_world_model_component DISCOVERY_MODE PRE_TEST)

    add_executable(test_planner_component test_planner_component.cpp)
    target_link_libraries(test_planner_component PRIVATE ame_pcl_components GTest::gtest_main)
    target_include_directories(test_planner_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    gtest_discover_tests(test_planner_component DISCOVERY_MODE PRE_TEST)

    add_executable(test_executor_component test_executor_component.cpp)
    target_link_libraries(test_executor_component PRIVATE ame_pcl_components GTest::gtest_main)
    target_include_directories(test_executor_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    gtest_discover_tests(test_executor_component DISCOVERY_MODE PRE_TEST)

    add_executable(test_pcl_integration test_pcl_integration.cpp)
    target_link_libraries(test_pcl_integration PRIVATE ame_pcl_components GTest::gtest_main)
    target_include_directories(test_pcl_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    gtest_discover_tests(test_pcl_integration DISCOVERY_MODE PRE_TEST)
endif()

add_executable(test_contingency_planning test_contingency_planning.cpp)
target_link_libraries(test_contingency_planning PRIVATE ame_core GTest::gtest_main)
target_compile_definitions(test_contingency_planning PRIVATE "PROJECT_SOURCE_DIR=\"${AME_SOURCE_DIR}\"")
gtest_discover_tests(test_contingency_planning DISCOVERY_MODE PRE_TEST)

add_executable(test_contingency_exhaustive test_contingency_exhaustive.cpp)
target_link_libraries(test_contingency_exhaustive PRIVATE ame_core GTest::gtest_main)
target_compile_definitions(test_contingency_exhaustive PRIVATE "PROJECT_SOURCE_DIR=\"${AME_SOURCE_DIR}\"")
gtest_discover_tests(test_contingency_exhaustive DISCOVERY_MODE PRE_TEST)

if(AME_NEURO)
    add_executable(test_neuro_backend test_neuro_backend.cpp)
    target_link_libraries(test_neuro_backend PRIVATE ame_neuro GTest::gtest_main)
    target_compile_definitions(test_neuro_backend PRIVATE AME_NEURO=1)
    gtest_discover_tests(test_neuro_backend DISCOVERY_MODE PRE_TEST)

    add_executable(test_neuro_advisor test_neuro_advisor.cpp)
    target_link_libraries(test_neuro_advisor PRIVATE ame_neuro GTest::gtest_main)
    target_compile_definitions(test_neuro_advisor PRIVATE AME_NEURO=1)
    gtest_discover_tests(test_neuro_advisor DISCOVERY_MODE PRE_TEST)

    add_executable(test_neuro_verifiers test_neuro_verifiers.cpp)
    target_link_libraries(test_neuro_verifiers PRIVATE ame_neuro GTest::gtest_main)
    target_compile_definitions(test_neuro_verifiers PRIVATE AME_NEURO=1)
    gtest_discover_tests(test_neuro_verifiers DISCOVERY_MODE PRE_TEST)

    add_executable(test_neuro_audit test_neuro_audit.cpp)
    target_link_libraries(test_neuro_audit PRIVATE ame_neuro GTest::gtest_main)
    target_compile_definitions(test_neuro_audit PRIVATE AME_NEURO=1)
    gtest_discover_tests(test_neuro_audit DISCOVERY_MODE PRE_TEST)

    # Covers configuration of the executor component, so it needs the PCL
    # component layer as well as the neuro library.
    if(AME_BUILD_PCL_COMPONENTS)
        add_executable(test_neuro_config test_neuro_config.cpp)
        target_link_libraries(test_neuro_config PRIVATE ame_neuro ame_pcl_components GTest::gtest_main)
        target_compile_definitions(test_neuro_config PRIVATE AME_NEURO=1)
        gtest_discover_tests(test_neuro_config DISCOVERY_MODE PRE_TEST)
    endif()

    add_executable(test_neuro_contract test_neuro_contract.cpp)
    target_link_libraries(test_neuro_contract PRIVATE ame_neuro GTest::gtest_main)
    target_compile_definitions(test_neuro_contract PRIVATE AME_NEURO=1)
    gtest_discover_tests(test_neuro_contract DISCOVERY_MODE PRE_TEST)

    add_executable(test_neuro_replay test_neuro_replay.cpp)
    target_link_libraries(test_neuro_replay PRIVATE ame_neuro GTest::gtest_main)
    target_compile_definitions(test_neuro_replay PRIVATE AME_NEURO=1)
    gtest_discover_tests(test_neuro_replay DISCOVERY_MODE PRE_TEST)
endif()

if(AME_BUILD_AUTHORING)
  add_executable(test_project_model test_project_model.cpp)
  target_link_libraries(test_project_model PRIVATE ame_authoring_project_model GTest::gtest_main)
  gtest_discover_tests(test_project_model DISCOVERY_MODE PRE_TEST)

  add_executable(test_relation_index test_relation_index.cpp)
  target_link_libraries(test_relation_index
    PRIVATE
      ame_authoring_project_model
      GTest::gtest_main
  )
  target_compile_definitions(test_relation_index
    PRIVATE "AME_DOMAINS_DIR=\"${AME_SOURCE_DIR}/domains\"")
  gtest_discover_tests(test_relation_index DISCOVERY_MODE PRE_TEST)

  add_executable(test_presentation_groups test_presentation_groups.cpp)
  target_link_libraries(test_presentation_groups
    PRIVATE
      ame_authoring_project_model
      GTest::gtest_main
  )
  gtest_discover_tests(test_presentation_groups DISCOVERY_MODE PRE_TEST)

  add_executable(test_authoring_phase6 test_authoring_phase6.cpp)
  target_link_libraries(test_authoring_phase6
    PRIVATE
      ame_authoring_project_model
      GTest::gtest_main
  )
  target_compile_definitions(test_authoring_phase6
    PRIVATE "AME_DOMAINS_DIR=\"${AME_SOURCE_DIR}/domains\"")
  gtest_discover_tests(test_authoring_phase6 DISCOVERY_MODE PRE_TEST)

  add_executable(test_command_stack
    test_command_stack.cpp
    ${AME_ROOT_DIR}/src/authoring/command_stack.cpp
  )
  target_link_libraries(test_command_stack PRIVATE ame_authoring_project_model GTest::gtest_main)
  gtest_discover_tests(test_command_stack DISCOVERY_MODE PRE_TEST)

  add_executable(test_pddl_generator test_pddl_generator.cpp)
  target_link_libraries(test_pddl_generator
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_pddl_generator DISCOVERY_MODE PRE_TEST)

  add_executable(test_pddl_importer test_pddl_importer.cpp)
  target_link_libraries(test_pddl_importer
    PRIVATE
      ame_authoring_project_model
      GTest::gtest_main
  )
  gtest_discover_tests(test_pddl_importer DISCOVERY_MODE PRE_TEST)

  add_executable(test_pddl_validator test_pddl_validator.cpp)
  target_link_libraries(test_pddl_validator
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_pddl_validator DISCOVERY_MODE PRE_TEST)

  add_executable(test_structural_validator test_structural_validator.cpp)
  target_link_libraries(test_structural_validator
    PRIVATE
      ame_authoring_project_model
      GTest::gtest_main
  )
  gtest_discover_tests(test_structural_validator DISCOVERY_MODE PRE_TEST)

  add_executable(test_feasibility test_feasibility.cpp)
  target_link_libraries(test_feasibility
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_feasibility DISCOVERY_MODE PRE_TEST)

  add_executable(test_scenario_runner test_scenario_runner.cpp)
  target_link_libraries(test_scenario_runner
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_scenario_runner DISCOVERY_MODE PRE_TEST)

  # Drives the mission command line: the library functions behind it, and the
  # built executable itself, because the exit code is what a build agent reads.
  # The problem list and the fact chooser decide wording and what is legal, so
  # both are checked without a window.
  # Edits that reach beyond the thing being edited, the folding of a run of
  # keystrokes into one undoable step, and the recent-projects list.
  # Merging a second domain into a project that already has one, and the
  # project fields that groups, saved views and contingencies are stored in.
  # The generated assurance evidence: what it says, and what it admits it
  # does not cover.
  add_executable(test_assurance_report test_assurance_report.cpp)
  target_link_libraries(test_assurance_report
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_assurance_report DISCOVERY_MODE PRE_TEST)

  add_executable(test_import_merge test_import_merge.cpp)
  target_link_libraries(test_import_merge
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_import_merge DISCOVERY_MODE PRE_TEST)

  add_executable(test_model_edits
    test_model_edits.cpp
    ${AME_ROOT_DIR}/src/authoring/command_stack.cpp
  )
  target_link_libraries(test_model_edits
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_model_edits DISCOVERY_MODE PRE_TEST)

  add_executable(test_problem_list test_problem_list.cpp)
  target_link_libraries(test_problem_list
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_problem_list DISCOVERY_MODE PRE_TEST)

  add_executable(test_mission_commands test_mission_commands.cpp)
  target_link_libraries(test_mission_commands
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  target_compile_definitions(test_mission_commands
    PRIVATE "AME_MISSION_CLI_PATH=\"$<TARGET_FILE:ame_mission_cli>\"")
  add_dependencies(test_mission_commands ame_mission_cli)
  gtest_discover_tests(test_mission_commands DISCOVERY_MODE PRE_TEST)

  add_executable(test_simulation_engine test_simulation_engine.cpp)
  target_link_libraries(test_simulation_engine
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_simulation_engine DISCOVERY_MODE PRE_TEST)

  add_executable(test_run_records test_run_records.cpp)
  target_link_libraries(test_run_records
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  add_dependencies(test_run_records ame_mission_cli)
  target_compile_definitions(test_run_records PRIVATE
    "AME_MISSION_CLI_PATH=\"$<TARGET_FILE:ame_mission_cli>\""
  )
  gtest_discover_tests(test_run_records DISCOVERY_MODE PRE_TEST)

  add_executable(test_contingency_analyser test_contingency_analyser.cpp)
  target_link_libraries(test_contingency_analyser
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  gtest_discover_tests(test_contingency_analyser DISCOVERY_MODE PRE_TEST)

  # End-to-end workflow: import a real-world PDDL domain + problem set
  # through the authoring tool's pipeline (PddlImporter -> Validator ->
  # PddlGenerator round-trip -> Planner), exercising the same code paths
  # the UI uses but headlessly.
  add_executable(test_vehicle_autonomy_workflow test_vehicle_autonomy_workflow.cpp)
  target_link_libraries(test_vehicle_autonomy_workflow
    PRIVATE
      ame_authoring_project_model
      ame_core
      GTest::gtest_main
  )
  target_compile_definitions(test_vehicle_autonomy_workflow
    PRIVATE "AME_DOMAINS_DIR=\"${AME_SOURCE_DIR}/domains\"")
  gtest_discover_tests(test_vehicle_autonomy_workflow DISCOVERY_MODE PRE_TEST)
endif()
