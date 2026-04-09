#include <gtest/gtest.h>

#include <ame/pcl_msg_json.h>
#include <ame/world_model_component.h>
#include <ame/executor_component.h>
#include <ame/planner_component.h>
#include <ame/agent_dispatcher.h>

#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>

#include <test_component_utils.hpp>

// ---------------------------------------------------------------------------
// Helper: build a pcl_msg_t from a string that stays alive for the call
// ---------------------------------------------------------------------------

static pcl_msg_t makePclMsg(const std::string& json, const char* type_name) {
    pcl_msg_t msg{};
    msg.data      = json.c_str();
    msg.size      = static_cast<uint32_t>(json.size());
    msg.type_name = type_name;
    return msg;
}

// ---------------------------------------------------------------------------
// Phase 1: WorldModelComponent PCL-level integration
// ---------------------------------------------------------------------------

///< REQ_PCL_001: WorldModelComponent shall handle get_fact service via PCL executor.
TEST(PclIntegration, WorldModelGetFactService) {
    ame::WorldModelComponent wm_comp;
    wm_comp.setParam("audit_log.enabled", false);
    wm_comp.setParam("perception.enabled", false);
    wm_comp.setParam("publish_rate_hz", 10.0);

    pcl::Executor executor;
    executor.add(wm_comp);

    ASSERT_EQ(wm_comp.configure(), PCL_OK);

    // Manually add a fluent and set it via the component API
    // (in-process access; outside agent would use PCL service)
    auto& wm = wm_comp.worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "object");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"object", "location"});
    wm.setFact("(at uav1 base)", true, "test");

    ASSERT_EQ(wm_comp.activate(), PCL_OK);

    // Invoke the "get_fact" service via the PCL executor
    std::string req_json = ame::ame_pack_get_fact_request("(at uav1 base)");
    pcl_msg_t req_msg    = makePclMsg(req_json, "ame/GetFact_Request");
    pcl_msg_t resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        executor.handle(), "get_fact", &req_msg, &resp_msg);

    ASSERT_EQ(rc, PCL_OK);

    auto result = ame::ame_unpack_get_fact_response(&resp_msg);
    EXPECT_TRUE(result.found);
    EXPECT_TRUE(result.value);

    EXPECT_EQ(wm_comp.deactivate(), PCL_OK);
    EXPECT_EQ(wm_comp.cleanup(), PCL_OK);
    EXPECT_EQ(wm_comp.shutdown(), PCL_OK);
}

///< REQ_PCL_002: WorldModelComponent shall handle set_fact service via PCL executor.
TEST(PclIntegration, WorldModelSetFactService) {
    ame::WorldModelComponent wm_comp;
    wm_comp.setParam("audit_log.enabled", false);
    wm_comp.setParam("perception.enabled", false);

    pcl::Executor executor;
    executor.add(wm_comp);
    ASSERT_EQ(wm_comp.configure(), PCL_OK);

    auto& wm = wm_comp.worldModel();
    wm.typeSystem().addType("object");
    wm.addObject("uav1", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"object", "location"});

    ASSERT_EQ(wm_comp.activate(), PCL_OK);

    ame::SetFactRequest sreq;
    sreq.key    = "(at uav1 base)";
    sreq.value  = true;
    sreq.source = "integration_test";
    std::string req_json = ame::ame_pack_set_fact_request(sreq);
    pcl_msg_t   req_msg  = makePclMsg(req_json, "ame/SetFact_Request");
    pcl_msg_t   resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        executor.handle(), "set_fact", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto result = ame::ame_unpack_set_fact_response(&resp_msg);
    EXPECT_TRUE(result.success);

    // Verify fact was actually set
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));

    EXPECT_EQ(wm_comp.deactivate(), PCL_OK);
    EXPECT_EQ(wm_comp.cleanup(), PCL_OK);
}

///< REQ_PCL_003: WorldModelComponent shall handle query_state service via PCL executor.
TEST(PclIntegration, WorldModelQueryStateService) {
    ame::WorldModelComponent wm_comp;
    wm_comp.setParam("audit_log.enabled", false);
    wm_comp.setParam("perception.enabled", false);

    pcl::Executor executor;
    executor.add(wm_comp);
    ASSERT_EQ(wm_comp.configure(), PCL_OK);

    auto& wm = wm_comp.worldModel();
    wm.typeSystem().addType("object");
    wm.addObject("uav1", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"object", "location"});
    wm.setFact("(at uav1 base)", true, "test");

    ASSERT_EQ(wm_comp.activate(), PCL_OK);

    std::string req_json = ame::ame_pack_query_state_request({});
    pcl_msg_t   req_msg  = makePclMsg(req_json, "ame/QueryState_Request");
    pcl_msg_t   resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        executor.handle(), "query_state", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto snap = ame::ame_unpack_query_state_response(&resp_msg);
    EXPECT_TRUE(snap.success);
    ASSERT_GE(snap.facts.size(), 1u);

    bool found = false;
    for (const auto& f : snap.facts) {
        if (f.key == "(at uav1 base)" && f.value) { found = true; break; }
    }
    EXPECT_TRUE(found);

    EXPECT_EQ(wm_comp.deactivate(), PCL_OK);
    EXPECT_EQ(wm_comp.cleanup(), PCL_OK);
}

// ---------------------------------------------------------------------------
// Phase 1: on_tick publishes world_state (via PCL port — just verifies no crash)
// ---------------------------------------------------------------------------

///< REQ_PCL_004: WorldModelComponent::on_tick shall execute without error when in executor.
TEST(PclIntegration, WorldModelOnTickNoError) {
    ame::WorldModelComponent wm_comp;
    wm_comp.setParam("audit_log.enabled", false);
    wm_comp.setParam("perception.enabled", false);
    wm_comp.setParam("publish_rate_hz", 100.0);

    pcl::Executor executor;
    executor.add(wm_comp);
    ASSERT_EQ(wm_comp.configure(), PCL_OK);
    ASSERT_EQ(wm_comp.activate(), PCL_OK);

    // spinOnce should trigger on_tick and publish (to no subscribers — fine)
    EXPECT_EQ(executor.spinOnce(0), PCL_OK);

    EXPECT_EQ(wm_comp.deactivate(), PCL_OK);
    EXPECT_EQ(wm_comp.cleanup(), PCL_OK);
}

// ---------------------------------------------------------------------------
// Phase 1: Detection subscriber (verify applyDetection path)
// ---------------------------------------------------------------------------

///< REQ_PCL_005: WorldModelComponent shall enqueue perception mutations from PCL subscriber.
TEST(PclIntegration, WorldModelDetectionSubscriber) {
    ame::WorldModelComponent wm_comp;
    wm_comp.setParam("audit_log.enabled", false);
    wm_comp.setParam("perception.enabled", true);
    wm_comp.setParam("perception.confidence_threshold", 0.5);
    wm_comp.setParam("publish_rate_hz", 1000.0);  // fast tick so on_tick fires in first spinOnce

    // Prime WM with a known fluent so the detection has a target
    pcl::Executor executor;
    executor.add(wm_comp);
    ASSERT_EQ(wm_comp.configure(), PCL_OK);

    auto& wm = wm_comp.worldModel();
    wm.typeSystem().addType("object");
    wm.addObject("uav1", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"object", "location"});

    ASSERT_EQ(wm_comp.activate(), PCL_OK);

    // Post an incoming detection message
    ame::Detection det;
    det.confidence      = 0.9f;
    det.sensor_source   = "camera";
    det.entity_id       = "uav1";
    det.property_keys   = {"at"};
    det.property_values = {"base"};

    std::string det_json = ame::ame_pack_detection(det);
    pcl_msg_t det_msg    = makePclMsg(det_json, "ame/Detection");
    EXPECT_EQ(executor.postIncoming("detections", &det_msg), PCL_OK);

    // Spin to process the queued incoming message and run on_tick
    executor.spinOnce(10);

    // on_tick should have called applyQueuedMutations; fact should be set
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));

    EXPECT_EQ(wm_comp.deactivate(), PCL_OK);
    EXPECT_EQ(wm_comp.cleanup(), PCL_OK);
}

// ---------------------------------------------------------------------------
// Phase 2: ExecutorComponent PCL-level integration
// ---------------------------------------------------------------------------

///< REQ_PCL_006: ExecutorComponent shall accept BT XML via PCL subscriber and tick to SUCCESS.
TEST(PclIntegration, ExecutorReceivesBTXmlAndTicks) {
    auto wm = buildUavWorldModel();

    ame::PlannerComponent pl_comp;
    pl_comp.setParam("plan_audit.enabled", false);
    pl_comp.setInProcessWorldModel(&wm);
    registerUavActions(pl_comp.actionRegistry());
    ASSERT_EQ(pl_comp.configure(), PCL_OK);
    ASSERT_EQ(pl_comp.activate(), PCL_OK);

    const auto plan = pl_comp.solveGoal({"(searched sector_a)", "(classified sector_a)"});
    ASSERT_TRUE(plan.success);

    ame::ExecutorComponent ex_comp;
    ex_comp.setParam("bt_log.enabled", false);
    ex_comp.setParam("tick_rate_hz", 1000.0);  // fast tick so on_tick fires in first spinOnce
    ex_comp.setInProcessWorldModel(&wm);
    registerUavStubNodes(ex_comp.factory());

    // Collect events to verify they flow through emitEvent
    std::vector<std::string> events;
    ex_comp.setEventSink([&events](const std::string& line) {
        events.push_back(line);
    });

    pcl::Executor executor;
    executor.add(ex_comp);
    ASSERT_EQ(ex_comp.configure(), PCL_OK);
    ASSERT_EQ(ex_comp.activate(), PCL_OK);

    // Post BT XML as an incoming PCL message
    pcl_msg_t bt_msg;
    bt_msg.data      = plan.bt_xml.c_str();
    bt_msg.size      = static_cast<uint32_t>(plan.bt_xml.size());
    bt_msg.type_name = "ame/BTXML";
    EXPECT_EQ(executor.postIncoming("executor/bt_xml", &bt_msg), PCL_OK);

    // Spin until SUCCESS or timeout (max 100 ticks at 50 Hz = 2 s budget)
    for (int i = 0; i < 100; ++i) {
        executor.spinOnce(20);
        if (!ex_comp.isExecuting()) break;
    }

    // The BT should have completed via on_tick
    EXPECT_FALSE(ex_comp.isExecuting());
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_TRUE(wm.getFact("(classified sector_a)"));
    EXPECT_FALSE(events.empty());

    EXPECT_EQ(ex_comp.deactivate(), PCL_OK);
    EXPECT_EQ(ex_comp.cleanup(), PCL_OK);
    EXPECT_EQ(pl_comp.deactivate(), PCL_OK);
    EXPECT_EQ(pl_comp.cleanup(), PCL_OK);
}

// ---------------------------------------------------------------------------
// Phase 3: PlannerComponent PCL-level integration
// ---------------------------------------------------------------------------

///< REQ_PCL_007: PlannerComponent shall solve goals via PCL plan service.
TEST(PclIntegration, PlannerPlanService) {
    auto wm = buildUavWorldModel();

    ame::PlannerComponent pl_comp;
    pl_comp.setParam("plan_audit.enabled", false);
    pl_comp.setInProcessWorldModel(&wm);
    registerUavActions(pl_comp.actionRegistry());

    pcl::Executor executor;
    executor.add(pl_comp);
    ASSERT_EQ(pl_comp.configure(), PCL_OK);
    ASSERT_EQ(pl_comp.activate(), PCL_OK);

    ame::PlanRequest req;
    req.goal_fluents = {"(searched sector_a)", "(classified sector_a)"};
    std::string req_json = ame::ame_pack_plan_request(req);
    pcl_msg_t req_msg    = makePclMsg(req_json, "ame/Plan_Request");
    pcl_msg_t resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        executor.handle(), "plan", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto resp = ame::ame_unpack_plan_response(&resp_msg);
    EXPECT_TRUE(resp.success);
    EXPECT_FALSE(resp.bt_xml.empty());
    EXPECT_GE(resp.plan_actions.size(), 3u);

    EXPECT_EQ(pl_comp.deactivate(), PCL_OK);
    EXPECT_EQ(pl_comp.cleanup(), PCL_OK);
}

///< REQ_PCL_008: PlannerComponent shall load a domain via PCL load_domain service.
TEST(PclIntegration, PlannerLoadDomainService) {
    ame::PlannerComponent pl_comp;
    pl_comp.setParam("plan_audit.enabled", false);

    pcl::Executor executor;
    executor.add(pl_comp);
    ASSERT_EQ(pl_comp.configure(), PCL_OK);
    ASSERT_EQ(pl_comp.activate(), PCL_OK);

    // Minimal trivial PDDL domain for testing
    const char* domain_pddl =
        "(define (domain test)\n"
        "  (:predicates (done))\n"
        "  (:action finish :parameters () :precondition () :effect (done))\n"
        ")\n";
    const char* problem_pddl =
        "(define (problem p) (:domain test) (:init) (:goal (done)))\n";

    ame::LoadDomainRequest lreq;
    lreq.domain_id    = "test";
    lreq.domain_pddl  = domain_pddl;
    lreq.problem_pddl = problem_pddl;

    std::string req_json = ame::ame_pack_load_domain_request(lreq);
    pcl_msg_t req_msg    = makePclMsg(req_json, "ame/LoadDomain_Request");
    pcl_msg_t resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        executor.handle(), "load_domain", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto resp = ame::ame_unpack_load_domain_response(&resp_msg);
    EXPECT_TRUE(resp.success);
    EXPECT_GT(resp.num_fluents, 0u);
    EXPECT_TRUE(pl_comp.hasDomain());
    EXPECT_EQ(pl_comp.domainId(), "test");

    EXPECT_EQ(pl_comp.deactivate(), PCL_OK);
    EXPECT_EQ(pl_comp.cleanup(), PCL_OK);
}

// ---------------------------------------------------------------------------
// JSON serialisation round-trip sanity tests
// ---------------------------------------------------------------------------

///< REQ_PCL_009: pcl_msg_json pack/unpack shall round-trip WorldStateSnapshot.
TEST(PclMsgJson, WorldStateRoundTrip) {
    ame::WorldStateSnapshot snap;
    snap.success    = true;
    snap.wm_version = 99;

    ame::WorldFactValue f;
    f.key        = "(at uav1 base)";
    f.value      = true;
    f.source     = "test:sensor";
    f.wm_version = 99;
    snap.facts.push_back(f);
    snap.goal_fluents = {"(at uav1 base)"};

    std::string json = ame::ame_pack_world_state(snap);
    ASSERT_FALSE(json.empty());

    pcl_msg_t msg{};
    ame::ame_make_pcl_msg(json, "ame/WorldState", msg);

    auto snap2 = ame::ame_unpack_world_state(&msg);
    EXPECT_EQ(snap2.success,    snap.success);
    EXPECT_EQ(snap2.wm_version, snap.wm_version);
    ASSERT_EQ(snap2.facts.size(), 1u);
    EXPECT_EQ(snap2.facts[0].key,    "(at uav1 base)");
    EXPECT_TRUE(snap2.facts[0].value);
    EXPECT_EQ(snap2.facts[0].source, "test:sensor");
    ASSERT_EQ(snap2.goal_fluents.size(), 1u);
    EXPECT_EQ(snap2.goal_fluents[0], "(at uav1 base)");
}

///< REQ_PCL_010: pcl_msg_json pack/unpack shall round-trip PlanResponse (including bt_xml).
TEST(PclMsgJson, PlanResponseRoundTrip) {
    ame::PlanResponse resp;
    resp.success       = true;
    resp.bt_xml        = "<root><BehaviorTree ID=\"T\"/></root>";
    resp.plan_actions  = {"move uav1 base sector_a", "search uav1 sector_a"};
    resp.solve_time_ms = 12.5;

    std::string json = ame::ame_pack_plan_response(resp);
    ASSERT_FALSE(json.empty());

    pcl_msg_t msg{};
    ame::ame_make_pcl_msg(json, "ame/Plan_Response", msg);

    auto resp2 = ame::ame_unpack_plan_response(&msg);
    EXPECT_EQ(resp2.success,       resp.success);
    EXPECT_EQ(resp2.bt_xml,        resp.bt_xml);
    EXPECT_EQ(resp2.solve_time_ms, resp.solve_time_ms);
    ASSERT_EQ(resp2.plan_actions.size(), 2u);
    EXPECT_EQ(resp2.plan_actions[0], resp.plan_actions[0]);
    EXPECT_EQ(resp2.plan_actions[1], resp.plan_actions[1]);
}

///< REQ_PCL_011: pcl_msg_json pack/unpack shall handle JSON-escaped characters in strings.
TEST(PclMsgJson, JsonEscapeRoundTrip) {
    ame::PlanResponse resp;
    resp.success  = false;
    resp.error_msg = "failed: \"quotation\" and\\backslash\nnewline";

    std::string json = ame::ame_pack_plan_response(resp);
    pcl_msg_t msg{};
    ame::ame_make_pcl_msg(json, "ame/Plan_Response", msg);

    auto resp2 = ame::ame_unpack_plan_response(&msg);
    EXPECT_EQ(resp2.error_msg, resp.error_msg);
}
