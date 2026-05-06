#include <gtest/gtest.h>

extern "C" {
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_types.h>
}

#include "flatbuffers/cpp/pyramid_services_sensor_data_interpretation_flatbuffers_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_sensors_codec.hpp"
#include "pyramid_services_sensor_data_interpretation_consumed.hpp"
#include "pyramid_services_sensor_data_interpretation_provided.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace prov = pyramid::components::sensor_data_interpretation::services::provided;
namespace cons = pyramid::components::sensor_data_interpretation::services::consumed;
namespace common_codec = pyramid::domain_model::common;
namespace sensor_codec = pyramid::domain_model::sensors;
namespace service_flatbuffers = pyramid::services::sensor_data_interpretation::flatbuffers_codec;
namespace types = pyramid::domain_model;

namespace {

types::InterpretationRequirement makeInterpretationRequirement() {
    types::InterpretationRequirement req;
    req.base.id = "interp-req-1";
    req.base.source = "sensor-test";
    req.status.status = types::Progress::InProgress;
    req.policy = types::InterpretationPolicy::IncludeObjects;
    req.type = types::InterpretationType::LocateSeaSurfaceObjects;
    req.point = types::Point{};
    req.point->position.latitude = 51.5;
    req.point->position.longitude = -0.12;
    return req;
}

types::ObjectEvidenceProvisionRequirement makeProvisionRequirement() {
    types::ObjectEvidenceProvisionRequirement req;
    req.base.id = "prov-req-1";
    req.base.source = "sensor-test";
    req.circle_area = types::CircleArea{};
    req.circle_area->position.latitude = 50.1;
    req.circle_area->position.longitude = 1.2;
    req.circle_area->radius = 250.0;
    return req;
}

types::ObjectAquisitionRequirement makeProcessingRequirement() {
    types::ObjectAquisitionRequirement req;
    req.base.id = "proc-req-1";
    req.base.source = "sensor-test";
    req.poly_area = types::PolyArea{};
    req.poly_area->points.push_back(types::GeodeticPosition{10.0, 20.0});
    req.poly_area->points.push_back(types::GeodeticPosition{11.0, 21.0});
    return req;
}

types::Query makeQuery() {
    types::Query query;
    query.id.push_back("lookup-1");
    query.one_shot = true;
    return query;
}

void expectInterpretationRequirementEqual(
    const types::InterpretationRequirement& actual,
    const types::InterpretationRequirement& expected) {
    EXPECT_EQ(actual.base.id, expected.base.id);
    EXPECT_EQ(actual.base.source, expected.base.source);
    EXPECT_EQ(actual.status.status, expected.status.status);
    EXPECT_EQ(actual.policy, expected.policy);
    EXPECT_EQ(actual.type, expected.type);
    ASSERT_EQ(actual.point.has_value(), expected.point.has_value());
    ASSERT_TRUE(actual.point.has_value());
    EXPECT_DOUBLE_EQ(actual.point->position.latitude,
                     expected.point->position.latitude);
    EXPECT_DOUBLE_EQ(actual.point->position.longitude,
                     expected.point->position.longitude);
}

void expectProvisionRequirementEqual(
    const types::ObjectEvidenceProvisionRequirement& actual,
    const types::ObjectEvidenceProvisionRequirement& expected) {
    EXPECT_EQ(actual.base.id, expected.base.id);
    EXPECT_EQ(actual.base.source, expected.base.source);
    ASSERT_EQ(actual.circle_area.has_value(), expected.circle_area.has_value());
    ASSERT_TRUE(actual.circle_area.has_value());
    EXPECT_DOUBLE_EQ(actual.circle_area->position.latitude,
                     expected.circle_area->position.latitude);
    EXPECT_DOUBLE_EQ(actual.circle_area->position.longitude,
                     expected.circle_area->position.longitude);
    EXPECT_DOUBLE_EQ(actual.circle_area->radius, expected.circle_area->radius);
}

void expectProcessingRequirementEqual(
    const types::ObjectAquisitionRequirement& actual,
    const types::ObjectAquisitionRequirement& expected) {
    EXPECT_EQ(actual.base.id, expected.base.id);
    EXPECT_EQ(actual.base.source, expected.base.source);
    ASSERT_EQ(actual.poly_area.has_value(), expected.poly_area.has_value());
    ASSERT_TRUE(actual.poly_area.has_value());
    ASSERT_EQ(actual.poly_area->points.size(), expected.poly_area->points.size());
    for (size_t i = 0; i < actual.poly_area->points.size(); ++i) {
        EXPECT_DOUBLE_EQ(actual.poly_area->points[i].latitude,
                         expected.poly_area->points[i].latitude);
        EXPECT_DOUBLE_EQ(actual.poly_area->points[i].longitude,
                         expected.poly_area->points[i].longitude);
    }
}

std::string encodeInterpretationRequest(const types::InterpretationRequirement& req,
                                        const char* content_type) {
    if (std::strcmp(content_type, prov::kJsonContentType) == 0) {
        return sensor_codec::toJson(req);
    }
    return service_flatbuffers::toBinary(req);
}

std::string encodeQueryRequest(const types::Query& req, const char* content_type) {
    if (std::strcmp(content_type, cons::kJsonContentType) == 0) {
        return common_codec::toJson(req);
    }
    return service_flatbuffers::toBinary(req);
}

struct ProvidedInvokeCtx {
    int service_count = 0;
    int callback_count = 0;
    bool decoded_response = false;
    types::InterpretationRequirement captured_req{};
    types::Identifier response_id{};
    std::string response_buffer{};
};

struct ConsumedInvokeCtx {
    int service_count = 0;
    int callback_count = 0;
    bool decoded_response = false;
    types::ObjectEvidenceProvisionRequirement captured_req{};
    types::Identifier response_id{};
    std::string response_buffer{};
};

struct MultiServiceHandler : public cons::ServiceHandler {
    int provision_reads = 0;
    int processing_reads = 0;

    std::vector<types::ObjectEvidenceProvisionRequirement>
    handleDataProvisionDependencyReadRequirement(const types::Query&) override {
        ++provision_reads;
        return {makeProvisionRequirement()};
    }

    std::vector<types::ObjectAquisitionRequirement>
    handleDataProcessingDependencyReadRequirement(const types::Query&) override {
        ++processing_reads;
        return {makeProcessingRequirement()};
    }
};

static pcl_status_t handle_provided_create_requirement(
    pcl_container_t*, const pcl_msg_t* request, pcl_msg_t* response,
    pcl_svc_context_t*, void* user_data) {
    auto* ctx = static_cast<ProvidedInvokeCtx*>(user_data);

    struct CapturingHandler : public prov::ServiceHandler {
        explicit CapturingHandler(ProvidedInvokeCtx& ctx) : ctx(ctx) {}

        types::Identifier handleInterpretationRequirementCreateRequirement(
            const types::InterpretationRequirement& req) override {
            ++ctx.service_count;
            ctx.captured_req = req;
            return "provided-created-id";
        }

        ProvidedInvokeCtx& ctx;
    };

    CapturingHandler handler(*ctx);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    prov::dispatch(
        handler, prov::ServiceChannel::InterpretationRequirementCreateRequirement,
                   request ? request->data : nullptr,
                   request ? request->size : 0u,
                   request ? request->type_name : prov::kJsonContentType,
                   &resp_buf, &resp_size);

    ctx->response_buffer.clear();
    if (resp_buf && resp_size > 0) {
        ctx->response_buffer.assign(static_cast<const char*>(resp_buf), resp_size);
        std::free(resp_buf);
    }

    response->data = ctx->response_buffer.empty()
                         ? nullptr
                         : const_cast<char*>(ctx->response_buffer.data());
    response->size = static_cast<uint32_t>(ctx->response_buffer.size());
    response->type_name = request ? request->type_name : prov::kJsonContentType;
    return PCL_OK;
}

static pcl_status_t configure_provided_create_requirement(
    pcl_container_t* c, void* ud) {
    auto* port = pcl_container_add_service(
        c, prov::kSvcInterpretationRequirementCreateRequirement,
        prov::kJsonContentType,
        handle_provided_create_requirement, ud);
    return port ? PCL_OK : PCL_ERR_NOMEM;
}

static void provided_create_requirement_response(
    const pcl_msg_t* response, void* user_data) {
    auto* ctx = static_cast<ProvidedInvokeCtx*>(user_data);
    ctx->decoded_response =
        prov::decodeInterpretationRequirementCreateRequirementResponse(
            response, &ctx->response_id);
    ++ctx->callback_count;
}

static pcl_status_t handle_consumed_provision_create_requirement(
    pcl_container_t*, const pcl_msg_t* request, pcl_msg_t* response,
    pcl_svc_context_t*, void* user_data) {
    auto* ctx = static_cast<ConsumedInvokeCtx*>(user_data);

    struct CapturingHandler : public cons::ServiceHandler {
        explicit CapturingHandler(ConsumedInvokeCtx& ctx) : ctx(ctx) {}

        types::Identifier handleDataProvisionDependencyCreateRequirement(
            const types::ObjectEvidenceProvisionRequirement& req) override {
            ++ctx.service_count;
            ctx.captured_req = req;
            return "consumed-created-id";
        }

        ConsumedInvokeCtx& ctx;
    };

    CapturingHandler handler(*ctx);
    void* resp_buf = nullptr;
    size_t resp_size = 0;
    cons::dispatch(handler,
                   cons::ServiceChannel::DataProvisionDependencyCreateRequirement,
                   request ? request->data : nullptr,
                   request ? request->size : 0u,
                   request ? request->type_name : cons::kJsonContentType,
                   &resp_buf, &resp_size);

    ctx->response_buffer.clear();
    if (resp_buf && resp_size > 0) {
        ctx->response_buffer.assign(static_cast<const char*>(resp_buf), resp_size);
        std::free(resp_buf);
    }

    response->data = ctx->response_buffer.empty()
                         ? nullptr
                         : const_cast<char*>(ctx->response_buffer.data());
    response->size = static_cast<uint32_t>(ctx->response_buffer.size());
    response->type_name = request ? request->type_name : cons::kJsonContentType;
    return PCL_OK;
}

static pcl_status_t configure_consumed_provision_create_requirement(
    pcl_container_t* c, void* ud) {
    auto* port = pcl_container_add_service(
        c, cons::kSvcDataProvisionDependencyCreateRequirement,
        cons::kJsonContentType,
        handle_consumed_provision_create_requirement, ud);
    return port ? PCL_OK : PCL_ERR_NOMEM;
}

static void consumed_provision_create_requirement_response(
    const pcl_msg_t* response, void* user_data) {
    auto* ctx = static_cast<ConsumedInvokeCtx*>(user_data);
    ctx->decoded_response =
        cons::decodeDataProvisionDependencyCreateRequirementResponse(
            response, &ctx->response_id);
    ++ctx->callback_count;
}

}  // namespace

TEST(SensorDataInterpretationBindings, ProvidedDispatchCreateRequirement) {
    const auto request = makeInterpretationRequirement();
    const char* content_types[] = {
        prov::kJsonContentType,
        prov::kFlatBuffersContentType,
    };

    for (const char* content_type : content_types) {
        struct CapturingHandler : public prov::ServiceHandler {
            int call_count = 0;
            types::InterpretationRequirement captured{};

            types::Identifier handleInterpretationRequirementCreateRequirement(
                const types::InterpretationRequirement& req) override {
                ++call_count;
                captured = req;
                return "dispatch-created-id";
            }
        } handler;

        const std::string payload = encodeInterpretationRequest(request, content_type);
        void* response_buf = nullptr;
        size_t response_size = 0;
        prov::dispatch(
            handler,
            prov::ServiceChannel::InterpretationRequirementCreateRequirement,
                       payload.data(), payload.size(), content_type,
                       &response_buf, &response_size);

        ASSERT_EQ(handler.call_count, 1) << content_type;
        expectInterpretationRequirementEqual(handler.captured, request);

        pcl_msg_t response{};
        response.data = response_buf;
        response.size = static_cast<uint32_t>(response_size);
        response.type_name = content_type;

        types::Identifier id;
        ASSERT_TRUE(prov::decodeInterpretationRequirementCreateRequirementResponse(
            &response, &id))
            << content_type;
        EXPECT_EQ(id, "dispatch-created-id");

        std::free(response_buf);
    }
}

TEST(SensorDataInterpretationBindings, ProvidedInvokeCreateRequirement) {
    const auto request = makeInterpretationRequirement();
    const char* content_types[] = {
        prov::kJsonContentType,
        prov::kFlatBuffersContentType,
    };

    for (const char* content_type : content_types) {
        ProvidedInvokeCtx ctx;
        pcl_callbacks_t cbs{};
        cbs.on_configure = configure_provided_create_requirement;

        pcl_executor_t* exec = pcl_executor_create();
        ASSERT_NE(exec, nullptr);

        pcl_container_t* c =
            pcl_container_create("sensor_provided_create_requirement", &cbs, &ctx);
        ASSERT_NE(c, nullptr);
        ASSERT_EQ(pcl_container_configure(c), PCL_OK);
        ASSERT_EQ(pcl_container_activate(c), PCL_OK);
        ASSERT_EQ(pcl_executor_add(exec, c), PCL_OK);

        const pcl_status_t rc =
            prov::invokeInterpretationRequirementCreateRequirement(
            exec, request, provided_create_requirement_response, &ctx,
            nullptr, content_type);

        EXPECT_EQ(rc, PCL_OK) << content_type;
        EXPECT_EQ(ctx.service_count, 1) << content_type;
        EXPECT_EQ(ctx.callback_count, 1) << content_type;
        EXPECT_TRUE(ctx.decoded_response) << content_type;
        EXPECT_EQ(ctx.response_id, "provided-created-id");
        expectInterpretationRequirementEqual(ctx.captured_req, request);

        pcl_executor_destroy(exec);
        pcl_container_destroy(c);
    }
}

TEST(SensorDataInterpretationBindings, ConsumedInvokeProvisionCreateRequirement) {
    const auto request = makeProvisionRequirement();
    const char* content_types[] = {
        cons::kJsonContentType,
        cons::kFlatBuffersContentType,
    };

    for (const char* content_type : content_types) {
        ConsumedInvokeCtx ctx;
        pcl_callbacks_t cbs{};
        cbs.on_configure = configure_consumed_provision_create_requirement;

        pcl_executor_t* exec = pcl_executor_create();
        ASSERT_NE(exec, nullptr);

        pcl_container_t* c =
            pcl_container_create("sensor_consumed_create_requirement", &cbs, &ctx);
        ASSERT_NE(c, nullptr);
        ASSERT_EQ(pcl_container_configure(c), PCL_OK);
        ASSERT_EQ(pcl_container_activate(c), PCL_OK);
        ASSERT_EQ(pcl_executor_add(exec, c), PCL_OK);

        const pcl_status_t rc =
            cons::invokeDataProvisionDependencyCreateRequirement(
                exec, request, consumed_provision_create_requirement_response,
                &ctx, nullptr, content_type);

        EXPECT_EQ(rc, PCL_OK) << content_type;
        EXPECT_EQ(ctx.service_count, 1) << content_type;
        EXPECT_EQ(ctx.callback_count, 1) << content_type;
        EXPECT_TRUE(ctx.decoded_response) << content_type;
        EXPECT_EQ(ctx.response_id, "consumed-created-id");
        expectProvisionRequirementEqual(ctx.captured_req, request);

        pcl_executor_destroy(exec);
        pcl_container_destroy(c);
    }
}

TEST(SensorDataInterpretationBindings, ConsumedDispatchDistinguishesDependencyServices) {
    MultiServiceHandler handler;
    const auto query = makeQuery();
    const char* content_types[] = {
        cons::kJsonContentType,
        cons::kFlatBuffersContentType,
    };

    for (const char* content_type : content_types) {
        const std::string payload = encodeQueryRequest(query, content_type);

        void* provision_buf = nullptr;
        size_t provision_size = 0;
        cons::dispatch(handler,
                       cons::ServiceChannel::DataProvisionDependencyReadRequirement,
                       payload.data(), payload.size(), content_type,
                       &provision_buf, &provision_size);

        pcl_msg_t provision_response{};
        provision_response.data = provision_buf;
        provision_response.size = static_cast<uint32_t>(provision_size);
        provision_response.type_name = content_type;

        std::vector<types::ObjectEvidenceProvisionRequirement> provision_values;
        ASSERT_TRUE(
            cons::decodeDataProvisionDependencyReadRequirementResponse(
                &provision_response, &provision_values))
            << content_type;
        ASSERT_EQ(provision_values.size(), 1u);
        expectProvisionRequirementEqual(
            provision_values.front(), makeProvisionRequirement());
        std::free(provision_buf);

        void* processing_buf = nullptr;
        size_t processing_size = 0;
        cons::dispatch(handler,
                       cons::ServiceChannel::DataProcessingDependencyReadRequirement,
                       payload.data(), payload.size(), content_type,
                       &processing_buf, &processing_size);

        pcl_msg_t processing_response{};
        processing_response.data = processing_buf;
        processing_response.size = static_cast<uint32_t>(processing_size);
        processing_response.type_name = content_type;

        std::vector<types::ObjectAquisitionRequirement> processing_values;
        ASSERT_TRUE(
            cons::decodeDataProcessingDependencyReadRequirementResponse(
                &processing_response, &processing_values))
            << content_type;
        ASSERT_EQ(processing_values.size(), 1u);
        expectProcessingRequirementEqual(
            processing_values.front(), makeProcessingRequirement());
        std::free(processing_buf);
    }

    EXPECT_EQ(handler.provision_reads, 2);
    EXPECT_EQ(handler.processing_reads, 2);
}
