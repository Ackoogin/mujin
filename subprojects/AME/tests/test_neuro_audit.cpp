// Phase 3 tests: NeuroAuditLog, PlanAuditLog provenance, AuditIndex
// WI-3.1, WI-3.2, WI-3.3

#include "ame/neuro/audit_index.h"
#include "ame/neuro/neuro_audit_log.h"
#include "ame/plan_audit_log.h"

#include <gtest/gtest.h>
#include <sstream>

using namespace ame::neuro;

// ---------------------------------------------------------------------------
// NeuroAuditLog (WI-3.1)
// ---------------------------------------------------------------------------

TEST(NeuroAuditLog, AppendAssignsId) {
    NeuroAuditLog log;
    NeuroAuditRecord r;
    r.integration_kind = "goal_interpreter";
    r.outcome = "Accepted";
    uint64_t id = log.append(r);
    EXPECT_GT(id, 0u);
    EXPECT_EQ(log.size(), 1u);
    EXPECT_EQ(log.records()[0].record_id, id);
}

TEST(NeuroAuditLog, MonotonicallyIncreasingIds) {
    NeuroAuditLog log;
    std::vector<uint64_t> ids;
    for (int i = 0; i < 5; ++i) {
        NeuroAuditRecord r;
        r.outcome = "Disabled";
        ids.push_back(log.append(r));
    }
    for (size_t i = 1; i < ids.size(); ++i) {
        EXPECT_GT(ids[i], ids[i - 1]);
    }
}

TEST(NeuroAuditLog, AffectedBehaviourSetForAccepted) {
    NeuroAuditLog log;
    NeuroAuditRecord r;
    r.outcome = "Accepted";
    r.affected_behaviour = true;
    log.append(r);
    EXPECT_TRUE(log.records()[0].affected_behaviour);
}

TEST(NeuroAuditLog, FileSinkWritesJsonl) {
    const std::string path = "/tmp/test_neuro_audit_log.jsonl";
    {
        NeuroAuditLog log(path);
        NeuroAuditRecord r;
        r.record_id = 0; // will be assigned
        r.integration_kind = "test_integration";
        r.outcome = "RejectedFellBack";
        r.verdict_reason = "test_reason";
        r.evidence = {"key1", "key2"};
        r.latency_ms = 42.5;
        r.affected_behaviour = false;
        log.append(r);
    }
    // Verify file contains valid JSON
    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());
    std::string line;
    ASSERT_TRUE(std::getline(f, line));
    EXPECT_NE(line.find("\"outcome\":\"RejectedFellBack\""), std::string::npos);
    EXPECT_NE(line.find("\"verdict_reason\":\"test_reason\""), std::string::npos);
    EXPECT_NE(line.find("\"latency_ms\":42.5"), std::string::npos);
}

// ---------------------------------------------------------------------------
// PlanAuditLog provenance fields (WI-3.2)
// ---------------------------------------------------------------------------

TEST(PlanAuditLog, ProvDefaultsPreserveShape) {
    ame::PlanAuditLog log;
    ame::PlanAuditLog::Episode ep;
    ep.solver = "BRFS";
    ep.success = true;
    // Neuro fields at defaults
    EXPECT_EQ(ep.heuristic_source, "symbolic");
    EXPECT_EQ(ep.goal_source, "symbolic");
    EXPECT_EQ(ep.repair_source, "symbolic");
    EXPECT_TRUE(ep.neuro_record_ids.empty());
    log.recordEpisode(std::move(ep));
    EXPECT_EQ(log.size(), 1u);
}

TEST(PlanAuditLog, ProvFieldsRecorded) {
    ame::PlanAuditLog log;
    ame::PlanAuditLog::Episode ep;
    ep.solver = "BRFS";
    ep.heuristic_source = "neural_hook";
    ep.neuro_record_ids = {42, 99};
    log.recordEpisode(std::move(ep));
    const auto& stored = log.episodes()[0];
    EXPECT_EQ(stored.heuristic_source, "neural_hook");
    ASSERT_EQ(stored.neuro_record_ids.size(), 2u);
    EXPECT_EQ(stored.neuro_record_ids[0], 42u);
    EXPECT_EQ(stored.neuro_record_ids[1], 99u);
}

// ---------------------------------------------------------------------------
// AuditIndex (WI-3.3)
// ---------------------------------------------------------------------------

static std::vector<std::string> make_plan_lines() {
    return {
        R"({"episode_id":1,"ts_us":1000,"success":true,"cost":2.0,"init_facts":"[]","goal_fluents":"[]","plan_actions":"[]","bt_xml":""})",
        R"({"episode_id":2,"ts_us":3000,"success":false,"cost":0.0,"init_facts":"[]","goal_fluents":"[]","plan_actions":"[]","bt_xml":""})",
        R"({"episode_id":3,"ts_us":5000,"success":true,"cost":1.0,"init_facts":"[]","goal_fluents":"[]","plan_actions":"[]","bt_xml":""})",
    };
}

static std::vector<std::string> make_neuro_lines() {
    return {
        R"({"record_id":10,"ts_us":2000,"integration_kind":"goal_interpreter","outcome":"Accepted","affected_behaviour":true,"latency_ms":5.0,"retries":0,"evidence":[]})",
        R"({"record_id":11,"ts_us":4000,"integration_kind":"plan_repair","outcome":"RejectedFellBack","affected_behaviour":false,"latency_ms":3.0,"retries":0,"evidence":[]})",
    };
}

TEST(AuditIndex, WindowReturnsCorrectRecords) {
    AuditIndex idx;
    idx.load_lines(make_plan_lines(), "plan");
    idx.load_lines(make_neuro_lines(), "neuro");

    auto w = idx.window(1500, 3500);
    ASSERT_EQ(w.size(), 2u); // episode_id=2 (ts=3000) and record_id=10 (ts=2000)
    // Verify ts range
    for (const auto& r : w) {
        EXPECT_GE(r.ts_us, 1500u);
        EXPECT_LE(r.ts_us, 3500u);
    }
}

TEST(AuditIndex, WindowEmpty_NoRecordsInRange) {
    AuditIndex idx;
    idx.load_lines(make_plan_lines(), "plan");
    auto w = idx.window(9000, 10000);
    EXPECT_TRUE(w.empty());
}

TEST(AuditIndex, AroundReturnsNeighbours) {
    AuditIndex idx;
    idx.load_lines(make_plan_lines(), "plan");
    // "2" is episode_id of second record
    auto around = idx.around("2", 1);
    EXPECT_GE(around.size(), 1u);
    EXPECT_LE(around.size(), 3u);
}

// ---------------------------------------------------------------------------
// Thread 26: around() uses timestamp order, not load (stream-grouping) order
// ---------------------------------------------------------------------------

TEST(AuditIndex, AroundUsesTimestampOrderAcrossStreams) {
    AuditIndex idx;
    // Load plan stream first, then neuro stream.
    // Plan ts: 1000, 3000, 5000  (episode_ids 1,2,3)
    // Neuro ts: 2000, 4000       (record_ids 10,11)
    // On the shared timeline:  1000(plan1), 2000(neuro10), 3000(plan2), 4000(neuro11), 5000(plan3)
    idx.load_lines(make_plan_lines(), "plan");
    idx.load_lines(make_neuro_lines(), "neuro");

    // Around plan episode "2" (ts=3000) with k=1:
    // Timestamp neighbours are neuro10 (ts=2000) and neuro11 (ts=4000).
    // Without timestamp sorting, load-order neighbours would be plan1 and plan3.
    auto result = idx.around("2", 1);
    ASSERT_EQ(result.size(), 3u);  // 3 records: one before, centre, one after

    // The three records in timestamp order must span neuro10, plan2, neuro11.
    std::vector<uint64_t> ts_values;
    for (const auto& r : result) ts_values.push_back(r.ts_us);
    EXPECT_EQ(ts_values[0], 2000u);  // neuro10
    EXPECT_EQ(ts_values[1], 3000u);  // plan2
    EXPECT_EQ(ts_values[2], 4000u);  // neuro11
}

TEST(AuditIndex, AroundRespectsSizeCap) {
    AuditIndex idx;
    // Load many large records
    std::vector<std::string> big_lines;
    for (int i = 1; i <= 20; ++i) {
        std::string line = R"({"episode_id":)" + std::to_string(i) +
            R"(,"ts_us":)" + std::to_string(i * 100) +
            R"(,"success":true,"cost":1.0,"init_facts":")" +
            std::string(500, 'x') + R"(","goal_fluents":"[]","plan_actions":"[]","bt_xml":""})";
        big_lines.push_back(line);
    }
    idx.load_lines(big_lines, "plan");
    size_t cap = 1000;
    auto around = idx.around("10", 10, cap);
    size_t total = 0;
    for (const auto& r : around) total += r.raw.size();
    EXPECT_LE(total, cap + big_lines[0].size()); // one extra record at most
}

TEST(AuditIndex, TrainingExportSuccessful) {
    AuditIndex idx;
    idx.load_lines(make_plan_lines(), "plan");
    auto samples = idx.training_export();
    ASSERT_EQ(samples.size(), 3u);
    EXPECT_TRUE(samples[0].success);
    EXPECT_FALSE(samples[1].success);
    EXPECT_FLOAT_EQ(samples[0].cost, 2.0f);
}

// ---------------------------------------------------------------------------
// Thread 22: training_export parses JSON string arrays into individual elements
// ---------------------------------------------------------------------------

TEST(AuditIndex, TrainingExportParsesArrayElements) {
    AuditIndex idx;
    std::vector<std::string> lines = {
        R"json({"episode_id":1,"ts_us":100,"success":true,"cost":1.0,)json"
        R"json("init_facts":["(at uav1 base)","(searched s1)"],)json"
        R"json("goal_fluents":["(at uav1 target)"],)json"
        R"json("plan_actions":["(move uav1 base target)","(survey uav1 target)"],)json"
        R"json("bt_xml":""})json",
    };
    idx.load_lines(lines, "plan");
    auto samples = idx.training_export();
    ASSERT_EQ(samples.size(), 1u);
    const auto& s = samples[0];
    ASSERT_EQ(s.init_facts.size(), 2u);
    EXPECT_EQ(s.init_facts[0], "(at uav1 base)");
    EXPECT_EQ(s.init_facts[1], "(searched s1)");
    ASSERT_EQ(s.goal_fluents.size(), 1u);
    EXPECT_EQ(s.goal_fluents[0], "(at uav1 target)");
    ASSERT_EQ(s.plan_actions.size(), 2u);
    EXPECT_EQ(s.plan_actions[0], "(move uav1 base target)");
    EXPECT_EQ(s.plan_actions[1], "(survey uav1 target)");
}

TEST(AuditIndex, CiteFindsReferredRecords) {
    AuditIndex idx;
    idx.load_lines(make_neuro_lines(), "neuro");
    auto ids = idx.cite("goal_interpreter");
    EXPECT_FALSE(ids.empty());
    EXPECT_EQ(ids[0], 10u);
}
