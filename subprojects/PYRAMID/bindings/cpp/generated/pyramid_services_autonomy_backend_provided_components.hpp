// Auto-generated component facade for the service binding.
// Generated from: pyramid.components.autonomy_backend.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::autonomy_backend::services::provided
//
// Layers on top of the low-level invoke/dispatch primitives in
// pyramid_services_autonomy_backend_provided.hpp. Components written against this header do not need to
// see pcl_msg_t, stream contexts, or response buffer ownership.
#pragma once

#include "pyramid_services_autonomy_backend_provided.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <cstdlib>
#include <cstdint>
#include <functional>
#include <future>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pyramid::components::autonomy_backend::services::provided {

// ---------------------------------------------------------------------------
// Result<T> -- typed result + pcl_status_t for async API.
// ---------------------------------------------------------------------------

template <class T>
struct Result {
    pcl_status_t status = PCL_OK;
    T            value{};

    bool ok() const { return status == PCL_OK; }
    explicit operator bool() const { return ok(); }
};

// ---------------------------------------------------------------------------
// StreamWriter<T> -- typed write handle the server-streaming handler
// receives. The handler may keep it alive across ticks and call send()
// when new frames are available, then end(). Dropping the writer without
// calling end() aborts the stream with PCL_ERR_STATE.
// ---------------------------------------------------------------------------

template <class T>
class StreamWriter {
public:
    using SendFn = pcl_status_t (*)(pcl_stream_context_t*,
                                    const T&,
                                    const char*);

    /// \brief Live writer: send() goes on the wire via pcl_stream_send.
    StreamWriter(pcl_stream_context_t* ctx,
                 std::string content_type,
                 SendFn send_fn)
        : ctx_(ctx),
          content_type_(std::move(content_type)),
          send_fn_(send_fn) {}

    /// \brief Collecting writer: send() appends to \p sink. Used when the
    ///        binding bridges a streaming-RPC handler to a unary caller.
    static StreamWriter collecting(std::vector<T>* sink) {
        StreamWriter w;
        w.sink_ = sink;
        return w;
    }

    StreamWriter() = default;
    StreamWriter(const StreamWriter&) = delete;
    StreamWriter& operator=(const StreamWriter&) = delete;
    StreamWriter(StreamWriter&& o) noexcept { *this = std::move(o); }
    StreamWriter& operator=(StreamWriter&& o) noexcept {
        if (this != &o) {
            abortIfLive();
            ctx_          = o.ctx_;          o.ctx_      = nullptr;
            content_type_ = std::move(o.content_type_);
            send_fn_      = o.send_fn_;      o.send_fn_  = nullptr;
            sink_         = o.sink_;         o.sink_     = nullptr;
        }
        return *this;
    }
    ~StreamWriter() { abortIfLive(); }

    /// \brief Emit one typed frame. Returns PCL_OK on success.
    pcl_status_t send(const T& frame) {
        if (sink_)   { sink_->push_back(frame); return PCL_OK; }
        if (!ctx_ || !send_fn_) return PCL_ERR_STATE;
        return send_fn_(ctx_, frame, content_type_.c_str());
    }

    /// \brief End the stream. \p status == PCL_OK clean-ends, otherwise
    ///        aborts with the given status. Idempotent.
    pcl_status_t end(pcl_status_t status = PCL_OK) {
        if (sink_) { sink_ = nullptr; return PCL_OK; }
        if (!ctx_) return PCL_OK;
        pcl_stream_context_t* c = ctx_;
        ctx_ = nullptr;
        send_fn_ = nullptr;
        if (status == PCL_OK) return pcl_stream_end(c);
        return pcl_stream_abort(c, status);
    }

    /// \brief True if the client has asked to cancel the stream.
    ///        Servers emitting long streams should poll this and stop.
    bool cancelled() const {
        return ctx_ && pcl_stream_is_cancelled(ctx_);
    }

    /// \brief Live state: an open wire stream the handler must end().
    ///        False for collecting writers and after end() has run.
    bool live() const { return ctx_ != nullptr; }

private:
    void abortIfLive() {
        if (!ctx_) return;
        pcl_stream_context_t* c = ctx_;
        ctx_ = nullptr;
        send_fn_ = nullptr;
        (void)pcl_stream_abort(c, PCL_ERR_STATE);
    }

    pcl_stream_context_t* ctx_ = nullptr;
    std::string           content_type_;
    SendFn                send_fn_ = nullptr;
    std::vector<T>*       sink_ = nullptr;
};

// ---------------------------------------------------------------------------
// ProvidedHandler -- typed callbacks the component author overrides.
//
// One on<Name> method per RPC. Unary RPCs return a typed value. Server-
// streaming RPCs take a StreamWriter<Frame> the handler keeps and pumps
// frames to over time; it must call writer.end() when the stream is
// complete (the writer auto-aborts if dropped without end()).
// ---------------------------------------------------------------------------

class ProvidedHandler {
public:
    virtual ~ProvidedHandler() = default;

    // Capabilities_Service
    virtual void
    onCapabilitiesReadCapabilities(const Query& /*request*/,
        StreamWriter<Capabilities> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }

    // Planning_Requirement_Service
    virtual Identifier
    onPlanningRequirementCreatePlanningRequirement(const PlanningRequirement& /*request*/) {
        return {};
    }
    virtual void
    onPlanningRequirementReadPlanningRequirement(const Query& /*request*/,
        StreamWriter<PlanningRequirement> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }
    virtual Ack
    onPlanningRequirementUpdatePlanningRequirement(const PlanningRequirement& /*request*/) {
        return {};
    }
    virtual Ack
    onPlanningRequirementDeletePlanningRequirement(const Identifier& /*request*/) {
        return {};
    }

    // Execution_Requirement_Service
    virtual Identifier
    onExecutionRequirementCreateExecutionRequirement(const ExecutionRequirement& /*request*/) {
        return {};
    }
    virtual void
    onExecutionRequirementReadExecutionRequirement(const Query& /*request*/,
        StreamWriter<ExecutionRequirement> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }
    virtual Ack
    onExecutionRequirementUpdateExecutionRequirement(const ExecutionRequirement& /*request*/) {
        return {};
    }
    virtual Ack
    onExecutionRequirementDeleteExecutionRequirement(const Identifier& /*request*/) {
        return {};
    }

    // State_Service
    virtual Identifier
    onStateCreateState(const StateUpdate& /*request*/) {
        return {};
    }
    virtual Ack
    onStateUpdateState(const StateUpdate& /*request*/) {
        return {};
    }
    virtual Ack
    onStateDeleteState(const Identifier& /*request*/) {
        return {};
    }

    // Plan_Service
    virtual Identifier
    onPlanCreatePlan(const Plan& /*request*/) {
        return {};
    }
    virtual void
    onPlanReadPlan(const Query& /*request*/,
        StreamWriter<Plan> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }
    virtual Ack
    onPlanUpdatePlan(const Plan& /*request*/) {
        return {};
    }
    virtual Ack
    onPlanDeletePlan(const Identifier& /*request*/) {
        return {};
    }

    // Execution_Run_Service
    virtual void
    onExecutionRunReadRun(const Query& /*request*/,
        StreamWriter<ExecutionRun> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }

    // Requirement_Placement_Service
    virtual void
    onRequirementPlacementReadPlacement(const Query& /*request*/,
        StreamWriter<RequirementPlacement> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }
};

// ---------------------------------------------------------------------------
// ProvidedService -- attach to your pcl::Component to host this package's
// RPCs. Owns the ServiceHandler-bridge adapter and the per-channel binding
// storage (response buffer, content type). Lifecycle of any open streams is
// owned by the user via StreamWriter::end().
//
// Usage: construct as a member of your component, call bind() from
// on_configure(); optionally restrict callers with routeAllRemote().
// ---------------------------------------------------------------------------

class ProvidedService {
public:
    ProvidedService(pcl::Component& host,
                    pcl::Executor& executor,
                    ProvidedHandler& handler,
                    std::string content_type = kJsonContentType)
        : host_(&host),
          executor_(&executor),
          handler_(&handler),
          content_type_(std::move(content_type)),
          bridge_(*this) {}

    /// \brief Owning constructor. \p handler must not be null;
    ///        std::invalid_argument is thrown otherwise.
    ProvidedService(pcl::Component& host,
                    pcl::Executor& executor,
                    std::unique_ptr<ProvidedHandler> handler,
                    std::string content_type = kJsonContentType)
        : ProvidedService(host, executor,
                          requireHandler(handler.get()),
                          std::move(content_type)) {
        owned_handler_ = std::move(handler);
    }

    ProvidedService(const ProvidedService&) = delete;
    ProvidedService& operator=(const ProvidedService&) = delete;
    ProvidedService(ProvidedService&&) = delete;
    ProvidedService& operator=(ProvidedService&&) = delete;

    /// \brief Install the service ports on the host component. Call from
    ///        the host's on_configure().
    pcl_status_t bind() {
        if (!supportsContentType(content_type_.c_str())) {
            return PCL_ERR_INVALID;
        }
        if (!addStreamBinding(kSvcCapabilitiesReadCapabilities, ServiceChannel::CapabilitiesReadCapabilities)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcPlanningRequirementCreatePlanningRequirement, ServiceChannel::PlanningRequirementCreatePlanningRequirement)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcPlanningRequirementReadPlanningRequirement, ServiceChannel::PlanningRequirementReadPlanningRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcPlanningRequirementUpdatePlanningRequirement, ServiceChannel::PlanningRequirementUpdatePlanningRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcPlanningRequirementDeletePlanningRequirement, ServiceChannel::PlanningRequirementDeletePlanningRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcExecutionRequirementCreateExecutionRequirement, ServiceChannel::ExecutionRequirementCreateExecutionRequirement)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcExecutionRequirementReadExecutionRequirement, ServiceChannel::ExecutionRequirementReadExecutionRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcExecutionRequirementUpdateExecutionRequirement, ServiceChannel::ExecutionRequirementUpdateExecutionRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcExecutionRequirementDeleteExecutionRequirement, ServiceChannel::ExecutionRequirementDeleteExecutionRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcStateCreateState, ServiceChannel::StateCreateState)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcStateUpdateState, ServiceChannel::StateUpdateState)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcStateDeleteState, ServiceChannel::StateDeleteState)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcPlanCreatePlan, ServiceChannel::PlanCreatePlan)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcPlanReadPlan, ServiceChannel::PlanReadPlan)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcPlanUpdatePlan, ServiceChannel::PlanUpdatePlan)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcPlanDeletePlan, ServiceChannel::PlanDeletePlan)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcExecutionRunReadRun, ServiceChannel::ExecutionRunReadRun)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcRequirementPlacementReadPlacement, ServiceChannel::RequirementPlacementReadPlacement)) return PCL_ERR_NOMEM;
        return PCL_OK;
    }

    /// \brief Restrict every advertised service to a single peer.
    pcl_status_t routeAllRemote(std::string_view peer_id) {
        for (const auto& port : ports_) {
            const pcl_status_t rc = port.routeRemote(peer_id);
            if (rc != PCL_OK) return rc;
        }
        return PCL_OK;
    }

private:
    static ProvidedHandler& requireHandler(ProvidedHandler* h) {
        if (!h) throw std::invalid_argument("ProvidedHandler must not be null");
        return *h;
    }

    // Adapter that satisfies the existing ServiceHandler ABI by
    // forwarding typed requests to the user-supplied ProvidedHandler.
    class Bridge final : public ServiceHandler {
    public:
        explicit Bridge(ProvidedService& owner) : owner_(&owner) {}

        std::vector<Capabilities>
        handleCapabilitiesReadCapabilities(const Query& request) override {
            std::vector<Capabilities> collected;
            auto writer = StreamWriter<Capabilities>::collecting(&collected);
            owner_->handler_->onCapabilitiesReadCapabilities(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamCapabilitiesReadCapabilities(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<Capabilities> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendCapabilitiesReadCapabilitiesStreamFrame
            };
            owner_->handler_->onCapabilitiesReadCapabilities(request, std::move(writer));
            return PCL_STREAMING;
        }
        Identifier
        handlePlanningRequirementCreatePlanningRequirement(const PlanningRequirement& request) override {
            return owner_->handler_->onPlanningRequirementCreatePlanningRequirement(request);
        }
        std::vector<PlanningRequirement>
        handlePlanningRequirementReadPlanningRequirement(const Query& request) override {
            std::vector<PlanningRequirement> collected;
            auto writer = StreamWriter<PlanningRequirement>::collecting(&collected);
            owner_->handler_->onPlanningRequirementReadPlanningRequirement(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamPlanningRequirementReadPlanningRequirement(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<PlanningRequirement> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendPlanningRequirementReadPlanningRequirementStreamFrame
            };
            owner_->handler_->onPlanningRequirementReadPlanningRequirement(request, std::move(writer));
            return PCL_STREAMING;
        }
        Ack
        handlePlanningRequirementUpdatePlanningRequirement(const PlanningRequirement& request) override {
            return owner_->handler_->onPlanningRequirementUpdatePlanningRequirement(request);
        }
        Ack
        handlePlanningRequirementDeletePlanningRequirement(const Identifier& request) override {
            return owner_->handler_->onPlanningRequirementDeletePlanningRequirement(request);
        }
        Identifier
        handleExecutionRequirementCreateExecutionRequirement(const ExecutionRequirement& request) override {
            return owner_->handler_->onExecutionRequirementCreateExecutionRequirement(request);
        }
        std::vector<ExecutionRequirement>
        handleExecutionRequirementReadExecutionRequirement(const Query& request) override {
            std::vector<ExecutionRequirement> collected;
            auto writer = StreamWriter<ExecutionRequirement>::collecting(&collected);
            owner_->handler_->onExecutionRequirementReadExecutionRequirement(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamExecutionRequirementReadExecutionRequirement(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<ExecutionRequirement> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendExecutionRequirementReadExecutionRequirementStreamFrame
            };
            owner_->handler_->onExecutionRequirementReadExecutionRequirement(request, std::move(writer));
            return PCL_STREAMING;
        }
        Ack
        handleExecutionRequirementUpdateExecutionRequirement(const ExecutionRequirement& request) override {
            return owner_->handler_->onExecutionRequirementUpdateExecutionRequirement(request);
        }
        Ack
        handleExecutionRequirementDeleteExecutionRequirement(const Identifier& request) override {
            return owner_->handler_->onExecutionRequirementDeleteExecutionRequirement(request);
        }
        Identifier
        handleStateCreateState(const StateUpdate& request) override {
            return owner_->handler_->onStateCreateState(request);
        }
        Ack
        handleStateUpdateState(const StateUpdate& request) override {
            return owner_->handler_->onStateUpdateState(request);
        }
        Ack
        handleStateDeleteState(const Identifier& request) override {
            return owner_->handler_->onStateDeleteState(request);
        }
        Identifier
        handlePlanCreatePlan(const Plan& request) override {
            return owner_->handler_->onPlanCreatePlan(request);
        }
        std::vector<Plan>
        handlePlanReadPlan(const Query& request) override {
            std::vector<Plan> collected;
            auto writer = StreamWriter<Plan>::collecting(&collected);
            owner_->handler_->onPlanReadPlan(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamPlanReadPlan(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<Plan> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendPlanReadPlanStreamFrame
            };
            owner_->handler_->onPlanReadPlan(request, std::move(writer));
            return PCL_STREAMING;
        }
        Ack
        handlePlanUpdatePlan(const Plan& request) override {
            return owner_->handler_->onPlanUpdatePlan(request);
        }
        Ack
        handlePlanDeletePlan(const Identifier& request) override {
            return owner_->handler_->onPlanDeletePlan(request);
        }
        std::vector<ExecutionRun>
        handleExecutionRunReadRun(const Query& request) override {
            std::vector<ExecutionRun> collected;
            auto writer = StreamWriter<ExecutionRun>::collecting(&collected);
            owner_->handler_->onExecutionRunReadRun(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamExecutionRunReadRun(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<ExecutionRun> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendExecutionRunReadRunStreamFrame
            };
            owner_->handler_->onExecutionRunReadRun(request, std::move(writer));
            return PCL_STREAMING;
        }
        std::vector<RequirementPlacement>
        handleRequirementPlacementReadPlacement(const Query& request) override {
            std::vector<RequirementPlacement> collected;
            auto writer = StreamWriter<RequirementPlacement>::collecting(&collected);
            owner_->handler_->onRequirementPlacementReadPlacement(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamRequirementPlacementReadPlacement(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<RequirementPlacement> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendRequirementPlacementReadPlacementStreamFrame
            };
            owner_->handler_->onRequirementPlacementReadPlacement(request, std::move(writer));
            return PCL_STREAMING;
        }
    private:
        ProvidedService* owner_;
    };

    struct UnaryBinding {
        Bridge*         bridge = nullptr;
        ServiceChannel  channel = ServiceChannel{};
        std::string     content_type;
        std::string     response_storage;
    };

    struct StreamBinding {
        Bridge*         bridge = nullptr;
        ServiceChannel  channel = ServiceChannel{};
        std::string     content_type;
    };

    static pcl_status_t unaryDispatch(pcl_container_t* /*self*/,
                                       const pcl_msg_t*  request,
                                       pcl_msg_t*        response,
                                       pcl_svc_context_t* /*ctx*/,
                                       void*             user_data) {
        auto* b = static_cast<UnaryBinding*>(user_data);
        if (!b || !b->bridge || !response) return PCL_ERR_INVALID;
        void*  buf = nullptr;
        size_t sz = 0;
        const char* ct = (request && request->type_name)
                             ? request->type_name
                             : b->content_type.c_str();
        dispatch(*b->bridge, b->channel,
                 request ? request->data : nullptr,
                 request ? request->size : 0u,
                 ct, &buf, &sz);
        b->response_storage.clear();
        if (buf && sz > 0u) {
            b->response_storage.assign(static_cast<const char*>(buf), sz);
            std::free(buf);
        }
        response->data = b->response_storage.empty() ? nullptr
                                                     : b->response_storage.data();
        response->size = static_cast<uint32_t>(b->response_storage.size());
        response->type_name = ct;
        return PCL_OK;
    }

    static pcl_status_t streamDispatch(pcl_container_t* /*self*/,
                                        const pcl_msg_t*       request,
                                        pcl_stream_context_t*  stream_context,
                                        void*                  user_data) {
        auto* b = static_cast<StreamBinding*>(user_data);
        if (!b || !b->bridge) return PCL_ERR_INVALID;
        const char* ct = (request && request->type_name)
                             ? request->type_name
                             : b->content_type.c_str();
        return dispatchStream(*b->bridge, b->channel,
                              request ? request->data : nullptr,
                              request ? request->size : 0u,
                              ct, stream_context);
    }

    bool addUnaryBinding(const char* service_name, ServiceChannel channel) {
        unary_bindings_.push_back(UnaryBinding{&bridge_, channel, content_type_, {}});
        UnaryBinding& binding = unary_bindings_.back();
        pcl::Port port = host_->addService(service_name, content_type_.c_str(),
                                           &ProvidedService::unaryDispatch, &binding);
        if (!port) { unary_bindings_.pop_back(); return false; }
        ports_.push_back(port);
        return true;
    }

    bool addStreamBinding(const char* service_name, ServiceChannel channel) {
        stream_bindings_.push_back(StreamBinding{&bridge_, channel, content_type_});
        StreamBinding& binding = stream_bindings_.back();
        pcl::Port port = host_->addStreamService(service_name, content_type_.c_str(),
                                                 &ProvidedService::streamDispatch, &binding);
        if (!port) { stream_bindings_.pop_back(); return false; }
        ports_.push_back(port);
        return true;
    }

    pcl::Component*                   host_     = nullptr;
    pcl::Executor*                    executor_ = nullptr;
    ProvidedHandler*                  handler_  = nullptr;
    std::unique_ptr<ProvidedHandler>  owned_handler_;
    std::string                       content_type_;
    Bridge                            bridge_;
    std::list<UnaryBinding>           unary_bindings_;
    std::list<StreamBinding>          stream_bindings_;
    std::vector<pcl::Port>            ports_;
};

// ---------------------------------------------------------------------------
// ConsumedService -- attach to your pcl::Component to call this package's
// RPCs. Per-RPC entry points are async-shaped:
//   * Unary RPCs return std::future<Result<T>>.
//   * Streaming RPCs return a StreamHandle and deliver frames via the
//     supplied on_frame/on_end callbacks (both fire on the executor
//     thread; together they cover the stream lifetime).
//
// Usage: construct as a member of your component; call routeAllRemote()
// (or routeAllLocal()) once the executor transport is up.
// ---------------------------------------------------------------------------

/// \brief Move-only handle for an in-flight server-streaming RPC.
///
/// Returned by every <op>Streaming() entry point. Call cancel() to stop
/// receiving frames -- the on_frame callback is suppressed from then on,
/// the underlying executor stream context is cancelled (the transport may
/// notify the server depending on its cancel support), and on_end fires
/// with PCL_ERR_CANCELLED once the framework finalises the stream.
/// Destroying or overwriting the handle also cancels, but suppresses
/// later user callbacks because their captures may be going away.
class StreamHandle {
public:
    StreamHandle() = default;
    StreamHandle(StreamHandle&& o) noexcept
        : cancel_fn_(std::move(o.cancel_fn_)) {
        o.cancel_fn_ = {};
    }
    StreamHandle& operator=(StreamHandle&& o) noexcept {
        if (this != &o) {
            cancelImpl(false);
            cancel_fn_ = std::move(o.cancel_fn_);
            o.cancel_fn_ = {};
        }
        return *this;
    }
    StreamHandle(const StreamHandle&) = delete;
    StreamHandle& operator=(const StreamHandle&) = delete;

    ~StreamHandle() { cancelImpl(false); }

    bool valid() const { return static_cast<bool>(cancel_fn_); }
    explicit operator bool() const { return valid(); }

    /// \brief Cancel the stream. Idempotent; subsequent calls no-op.
    void cancel() {
        cancelImpl(true);
    }

private:
    void cancelImpl(bool notify_end) {
        if (cancel_fn_) {
            cancel_fn_(notify_end);
            cancel_fn_ = {};
        }
    }

    friend class ConsumedService;
    explicit StreamHandle(std::function<void(bool)> cancel_fn)
        : cancel_fn_(std::move(cancel_fn)) {}
    std::function<void(bool)> cancel_fn_;
};

class ConsumedService {
public:
    ConsumedService(pcl::Component& host,
                    pcl::Executor& executor,
                    std::string content_type = kJsonContentType)
        : host_(&host),
          executor_(&executor),
          content_type_(std::move(content_type)) {}

    ConsumedService(const ConsumedService&) = delete;
    ConsumedService& operator=(const ConsumedService&) = delete;
    ConsumedService(ConsumedService&&) = delete;
    ConsumedService& operator=(ConsumedService&&) = delete;

    /// \brief Route every consumed endpoint to the executor's
    ///        default transport. The transport itself picks the peer
    ///        (e.g. shared-memory bus discovers the unique provider).
    pcl_status_t routeAllRemote() {
        if (auto rc = executor_->setEndpointRoute(
                kSvcCapabilitiesReadCapabilities, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanningRequirementCreatePlanningRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanningRequirementReadPlanningRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanningRequirementUpdatePlanningRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanningRequirementDeletePlanningRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcExecutionRequirementCreateExecutionRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcExecutionRequirementReadExecutionRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcExecutionRequirementUpdateExecutionRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcExecutionRequirementDeleteExecutionRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcStateCreateState, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcStateUpdateState, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcStateDeleteState, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanCreatePlan, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanReadPlan, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanUpdatePlan, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcPlanDeletePlan, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcExecutionRunReadRun, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcRequirementPlacementReadPlacement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        return PCL_OK;
    }

    /// \brief Route every consumed endpoint to a named peer transport
    ///        previously registered via executor.registerTransport().
    pcl_status_t routeAllRemote(std::string_view peer_id) {
        if (auto rc = executor_->routeRemote(
                kSvcCapabilitiesReadCapabilities, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanningRequirementCreatePlanningRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanningRequirementReadPlanningRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanningRequirementUpdatePlanningRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanningRequirementDeletePlanningRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcExecutionRequirementCreateExecutionRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcExecutionRequirementReadExecutionRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcExecutionRequirementUpdateExecutionRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcExecutionRequirementDeleteExecutionRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcStateCreateState, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcStateUpdateState, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcStateDeleteState, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanCreatePlan, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanReadPlan, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanUpdatePlan, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcPlanDeletePlan, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcExecutionRunReadRun, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcRequirementPlacementReadPlacement, peer_id); rc != PCL_OK) return rc;
        return PCL_OK;
    }

    pcl_status_t routeAllLocal() {
        if (auto rc = executor_->routeLocal(
                kSvcCapabilitiesReadCapabilities); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanningRequirementCreatePlanningRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanningRequirementReadPlanningRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanningRequirementUpdatePlanningRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanningRequirementDeletePlanningRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcExecutionRequirementCreateExecutionRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcExecutionRequirementReadExecutionRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcExecutionRequirementUpdateExecutionRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcExecutionRequirementDeleteExecutionRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcStateCreateState); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcStateUpdateState); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcStateDeleteState); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanCreatePlan); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanReadPlan); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanUpdatePlan); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcPlanDeletePlan); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcExecutionRunReadRun); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcRequirementPlacementReadPlacement); rc != PCL_OK) return rc;
        return PCL_OK;
    }

    StreamHandle
    capabilitiesReadCapabilitiesStreaming(const Query& request,
                std::function<void(const Capabilities&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<Capabilities>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, Capabilities* out) {
            return decodeCapabilitiesReadCapabilitiesStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<Capabilities>>(
            StreamPushHolder<Capabilities>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeCapabilitiesReadCapabilitiesStream(
            executor_->handle(), request,
            &StreamPushState<Capabilities>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle](bool notify_end) {
            push->cancelled = true;
            if (!notify_end) {
                push->on_frame = {};
                push->on_end = {};
            }
            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    std::future<Result<Identifier>>
    planningRequirementCreatePlanningRequirementAsync(const PlanningRequirement& request) {
        auto state = std::make_shared<UnaryState<Identifier>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Identifier* out) {
            return decodePlanningRequirementCreatePlanningRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Identifier>>(UnaryHolder<Identifier>{state});
        const pcl_status_t rc = invokePlanningRequirementCreatePlanningRequirement(
            executor_->handle(), request,
            &UnaryState<Identifier>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    StreamHandle
    planningRequirementReadPlanningRequirementStreaming(const Query& request,
                std::function<void(const PlanningRequirement&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<PlanningRequirement>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, PlanningRequirement* out) {
            return decodePlanningRequirementReadPlanningRequirementStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<PlanningRequirement>>(
            StreamPushHolder<PlanningRequirement>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokePlanningRequirementReadPlanningRequirementStream(
            executor_->handle(), request,
            &StreamPushState<PlanningRequirement>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle](bool notify_end) {
            push->cancelled = true;
            if (!notify_end) {
                push->on_frame = {};
                push->on_end = {};
            }
            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    std::future<Result<Ack>>
    planningRequirementUpdatePlanningRequirementAsync(const PlanningRequirement& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodePlanningRequirementUpdatePlanningRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokePlanningRequirementUpdatePlanningRequirement(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Ack>>
    planningRequirementDeletePlanningRequirementAsync(const Identifier& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodePlanningRequirementDeletePlanningRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokePlanningRequirementDeletePlanningRequirement(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Identifier>>
    executionRequirementCreateExecutionRequirementAsync(const ExecutionRequirement& request) {
        auto state = std::make_shared<UnaryState<Identifier>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Identifier* out) {
            return decodeExecutionRequirementCreateExecutionRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Identifier>>(UnaryHolder<Identifier>{state});
        const pcl_status_t rc = invokeExecutionRequirementCreateExecutionRequirement(
            executor_->handle(), request,
            &UnaryState<Identifier>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    StreamHandle
    executionRequirementReadExecutionRequirementStreaming(const Query& request,
                std::function<void(const ExecutionRequirement&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<ExecutionRequirement>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, ExecutionRequirement* out) {
            return decodeExecutionRequirementReadExecutionRequirementStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<ExecutionRequirement>>(
            StreamPushHolder<ExecutionRequirement>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeExecutionRequirementReadExecutionRequirementStream(
            executor_->handle(), request,
            &StreamPushState<ExecutionRequirement>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle](bool notify_end) {
            push->cancelled = true;
            if (!notify_end) {
                push->on_frame = {};
                push->on_end = {};
            }
            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    std::future<Result<Ack>>
    executionRequirementUpdateExecutionRequirementAsync(const ExecutionRequirement& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodeExecutionRequirementUpdateExecutionRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokeExecutionRequirementUpdateExecutionRequirement(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Ack>>
    executionRequirementDeleteExecutionRequirementAsync(const Identifier& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodeExecutionRequirementDeleteExecutionRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokeExecutionRequirementDeleteExecutionRequirement(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Identifier>>
    stateCreateStateAsync(const StateUpdate& request) {
        auto state = std::make_shared<UnaryState<Identifier>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Identifier* out) {
            return decodeStateCreateStateResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Identifier>>(UnaryHolder<Identifier>{state});
        const pcl_status_t rc = invokeStateCreateState(
            executor_->handle(), request,
            &UnaryState<Identifier>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Ack>>
    stateUpdateStateAsync(const StateUpdate& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodeStateUpdateStateResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokeStateUpdateState(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Ack>>
    stateDeleteStateAsync(const Identifier& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodeStateDeleteStateResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokeStateDeleteState(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Identifier>>
    planCreatePlanAsync(const Plan& request) {
        auto state = std::make_shared<UnaryState<Identifier>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Identifier* out) {
            return decodePlanCreatePlanResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Identifier>>(UnaryHolder<Identifier>{state});
        const pcl_status_t rc = invokePlanCreatePlan(
            executor_->handle(), request,
            &UnaryState<Identifier>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    StreamHandle
    planReadPlanStreaming(const Query& request,
                std::function<void(const Plan&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<Plan>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, Plan* out) {
            return decodePlanReadPlanStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<Plan>>(
            StreamPushHolder<Plan>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokePlanReadPlanStream(
            executor_->handle(), request,
            &StreamPushState<Plan>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle](bool notify_end) {
            push->cancelled = true;
            if (!notify_end) {
                push->on_frame = {};
                push->on_end = {};
            }
            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    std::future<Result<Ack>>
    planUpdatePlanAsync(const Plan& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodePlanUpdatePlanResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokePlanUpdatePlan(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    std::future<Result<Ack>>
    planDeletePlanAsync(const Identifier& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodePlanDeletePlanResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokePlanDeletePlan(
            executor_->handle(), request,
            &UnaryState<Ack>::trampoline,
            holder.get(),
            nullptr, content_type_.c_str());
        if (rc == PCL_OK) {
            (void)holder.release();
        } else {
            state->promise.set_value({rc, {}});
        }
        return future;
    }

    StreamHandle
    executionRunReadRunStreaming(const Query& request,
                std::function<void(const ExecutionRun&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<ExecutionRun>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, ExecutionRun* out) {
            return decodeExecutionRunReadRunStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<ExecutionRun>>(
            StreamPushHolder<ExecutionRun>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeExecutionRunReadRunStream(
            executor_->handle(), request,
            &StreamPushState<ExecutionRun>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle](bool notify_end) {
            push->cancelled = true;
            if (!notify_end) {
                push->on_frame = {};
                push->on_end = {};
            }
            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    StreamHandle
    requirementPlacementReadPlacementStreaming(const Query& request,
                std::function<void(const RequirementPlacement&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<RequirementPlacement>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, RequirementPlacement* out) {
            return decodeRequirementPlacementReadPlacementStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<RequirementPlacement>>(
            StreamPushHolder<RequirementPlacement>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeRequirementPlacementReadPlacementStream(
            executor_->handle(), request,
            &StreamPushState<RequirementPlacement>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle](bool notify_end) {
            push->cancelled = true;
            if (!notify_end) {
                push->on_frame = {};
                push->on_end = {};
            }
            if (ctx_handle && !push->closed) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

private:
    template <class T>
    struct UnaryState {
        std::promise<Result<T>>                       promise;
        std::function<bool(const pcl_msg_t*, T*)>     decoder;

        static void trampoline(const pcl_msg_t* msg, void* user_data);
    };

    template <class T>
    struct UnaryHolderT { std::shared_ptr<UnaryState<T>> state; };
    template <class T> using UnaryHolder = UnaryHolderT<T>;

    template <class T>
    struct StreamPushState {
        std::function<void(const T&)>                 on_frame;
        std::function<void(pcl_status_t)>             on_end;
        std::function<bool(const pcl_msg_t*, T*)>     decoder;
        bool                                          cancelled = false;

        bool                                          closed = false;

        static void trampoline(const pcl_msg_t* msg,
                                bool             end,
                                pcl_status_t     status,
                                void*            user_data);
    };

    template <class T>
    struct StreamPushHolderT { std::shared_ptr<StreamPushState<T>> state; };
    template <class T> using StreamPushHolder = StreamPushHolderT<T>;

    pcl::Component* host_     = nullptr;
    pcl::Executor*  executor_ = nullptr;
    std::string     content_type_;
};

template <class T>
inline void ConsumedService::UnaryState<T>::trampoline(
        const pcl_msg_t* msg, void* user_data) {
    std::unique_ptr<ConsumedService::UnaryHolder<T>> holder(
        static_cast<ConsumedService::UnaryHolder<T>*>(user_data));
    if (!holder || !holder->state) return;
    auto& state = *holder->state;
    Result<T> result{};
    if (state.decoder && state.decoder(msg, &result.value)) {
        result.status = PCL_OK;
    } else {
        result.status = PCL_ERR_INVALID;
    }
    state.promise.set_value(std::move(result));
}

template <class T>
inline void ConsumedService::StreamPushState<T>::trampoline(
        const pcl_msg_t* msg, bool end, pcl_status_t status,
        void* user_data) {
    auto* holder = static_cast<ConsumedService::StreamPushHolder<T>*>(
        user_data);
    if (!holder || !holder->state) {
        if (end) delete holder;
        return;
    }
    auto& state = *holder->state;
    if (end) {
        state.closed = true;
        const pcl_status_t end_status = state.cancelled
            ? PCL_ERR_CANCELLED
            : status;
        if (state.on_end) state.on_end(end_status);
        delete holder;
        return;
    }
    if (state.cancelled || status != PCL_OK) return;
    T frame{};
    if (state.decoder && state.decoder(msg, &frame) && state.on_frame) {
        state.on_frame(frame);
    }
}

} // namespace pyramid::components::autonomy_backend::services::provided
