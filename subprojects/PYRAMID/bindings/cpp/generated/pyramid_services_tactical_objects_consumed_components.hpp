// Auto-generated component facade for the service binding.
// Generated from: pyramid.components.tactical_objects.services.provided.consumed.proto by generate_bindings.py
// Namespace: pyramid::components::tactical_objects::services::consumed
//
// Layers on top of the low-level invoke/dispatch primitives in
// pyramid_services_tactical_objects_consumed.hpp. Components written against this header do not need to
// see pcl_msg_t, stream contexts, or response buffer ownership.
#pragma once

#include "pyramid_services_tactical_objects_consumed.hpp"

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

namespace pyramid::components::tactical_objects::services::consumed {

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

    // Object_Evidence_Service
    virtual void
    onObjectEvidenceReadDetail(const Query& /*request*/,
        StreamWriter<ObjectDetail> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }

    // Object_Solution_Evidence_Service
    virtual Identifier
    onObjectSolutionEvidenceCreateRequirement(const ObjectEvidenceRequirement& /*request*/) {
        return {};
    }
    virtual void
    onObjectSolutionEvidenceReadRequirement(const Query& /*request*/,
        StreamWriter<ObjectEvidenceRequirement> writer) {
        // Default: empty stream. Override to emit frames + end.
        writer.end();
    }
    virtual Ack
    onObjectSolutionEvidenceUpdateRequirement(const ObjectEvidenceRequirement& /*request*/) {
        return {};
    }
    virtual Ack
    onObjectSolutionEvidenceDeleteRequirement(const Identifier& /*request*/) {
        return {};
    }

    // Object_Source_Capability_Service
    virtual void
    onObjectSourceCapabilityReadCapability(const Query& /*request*/,
        StreamWriter<Capability> writer) {
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
        if (!addStreamBinding(kSvcObjectEvidenceReadDetail, ServiceChannel::ObjectEvidenceReadDetail)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcObjectSolutionEvidenceCreateRequirement, ServiceChannel::ObjectSolutionEvidenceCreateRequirement)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcObjectSolutionEvidenceReadRequirement, ServiceChannel::ObjectSolutionEvidenceReadRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcObjectSolutionEvidenceUpdateRequirement, ServiceChannel::ObjectSolutionEvidenceUpdateRequirement)) return PCL_ERR_NOMEM;
        if (!addUnaryBinding(kSvcObjectSolutionEvidenceDeleteRequirement, ServiceChannel::ObjectSolutionEvidenceDeleteRequirement)) return PCL_ERR_NOMEM;
        if (!addStreamBinding(kSvcObjectSourceCapabilityReadCapability, ServiceChannel::ObjectSourceCapabilityReadCapability)) return PCL_ERR_NOMEM;
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

        std::vector<ObjectDetail>
        handleObjectEvidenceReadDetail(const Query& request) override {
            std::vector<ObjectDetail> collected;
            auto writer = StreamWriter<ObjectDetail>::collecting(&collected);
            owner_->handler_->onObjectEvidenceReadDetail(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamObjectEvidenceReadDetail(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<ObjectDetail> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendObjectEvidenceReadDetailStreamFrame
            };
            owner_->handler_->onObjectEvidenceReadDetail(request, std::move(writer));
            return PCL_STREAMING;
        }
        Identifier
        handleObjectSolutionEvidenceCreateRequirement(const ObjectEvidenceRequirement& request) override {
            return owner_->handler_->onObjectSolutionEvidenceCreateRequirement(request);
        }
        std::vector<ObjectEvidenceRequirement>
        handleObjectSolutionEvidenceReadRequirement(const Query& request) override {
            std::vector<ObjectEvidenceRequirement> collected;
            auto writer = StreamWriter<ObjectEvidenceRequirement>::collecting(&collected);
            owner_->handler_->onObjectSolutionEvidenceReadRequirement(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamObjectSolutionEvidenceReadRequirement(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<ObjectEvidenceRequirement> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendObjectSolutionEvidenceReadRequirementStreamFrame
            };
            owner_->handler_->onObjectSolutionEvidenceReadRequirement(request, std::move(writer));
            return PCL_STREAMING;
        }
        Ack
        handleObjectSolutionEvidenceUpdateRequirement(const ObjectEvidenceRequirement& request) override {
            return owner_->handler_->onObjectSolutionEvidenceUpdateRequirement(request);
        }
        Ack
        handleObjectSolutionEvidenceDeleteRequirement(const Identifier& request) override {
            return owner_->handler_->onObjectSolutionEvidenceDeleteRequirement(request);
        }
        std::vector<Capability>
        handleObjectSourceCapabilityReadCapability(const Query& request) override {
            std::vector<Capability> collected;
            auto writer = StreamWriter<Capability>::collecting(&collected);
            owner_->handler_->onObjectSourceCapabilityReadCapability(request, std::move(writer));
            return collected;
        }
        pcl_status_t
        streamObjectSourceCapabilityReadCapability(const Query& request,
                                    pcl_stream_context_t* stream_context,
                                    const char* content_type) override {
            StreamWriter<Capability> writer{
                stream_context,
                content_type ? content_type : kJsonContentType,
                &sendObjectSourceCapabilityReadCapabilityStreamFrame
            };
            owner_->handler_->onObjectSourceCapabilityReadCapability(request, std::move(writer));
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
class StreamHandle {
public:
    StreamHandle() = default;
    StreamHandle(StreamHandle&&) noexcept = default;
    StreamHandle& operator=(StreamHandle&&) noexcept = default;
    StreamHandle(const StreamHandle&) = delete;
    StreamHandle& operator=(const StreamHandle&) = delete;

    bool valid() const { return static_cast<bool>(cancel_fn_); }
    explicit operator bool() const { return valid(); }

    /// \brief Cancel the stream. Idempotent; subsequent calls no-op.
    void cancel() {
        if (cancel_fn_) {
            cancel_fn_();
            cancel_fn_ = {};
        }
    }

private:
    friend class ConsumedService;
    explicit StreamHandle(std::function<void()> cancel_fn)
        : cancel_fn_(std::move(cancel_fn)) {}
    std::function<void()> cancel_fn_;
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
                kSvcObjectEvidenceReadDetail, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcObjectSolutionEvidenceCreateRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcObjectSolutionEvidenceReadRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcObjectSolutionEvidenceUpdateRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcObjectSolutionEvidenceDeleteRequirement, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        if (auto rc = executor_->setEndpointRoute(
                kSvcObjectSourceCapabilityReadCapability, PCL_ENDPOINT_CONSUMED,
                PCL_ROUTE_REMOTE); rc != PCL_OK) return rc;
        return PCL_OK;
    }

    /// \brief Route every consumed endpoint to a named peer transport
    ///        previously registered via executor.registerTransport().
    pcl_status_t routeAllRemote(std::string_view peer_id) {
        if (auto rc = executor_->routeRemote(
                kSvcObjectEvidenceReadDetail, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcObjectSolutionEvidenceCreateRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcObjectSolutionEvidenceReadRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcObjectSolutionEvidenceUpdateRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcObjectSolutionEvidenceDeleteRequirement, peer_id); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeRemote(
                kSvcObjectSourceCapabilityReadCapability, peer_id); rc != PCL_OK) return rc;
        return PCL_OK;
    }

    pcl_status_t routeAllLocal() {
        if (auto rc = executor_->routeLocal(
                kSvcObjectEvidenceReadDetail); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcObjectSolutionEvidenceCreateRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcObjectSolutionEvidenceReadRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcObjectSolutionEvidenceUpdateRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcObjectSolutionEvidenceDeleteRequirement); rc != PCL_OK) return rc;
        if (auto rc = executor_->routeLocal(
                kSvcObjectSourceCapabilityReadCapability); rc != PCL_OK) return rc;
        return PCL_OK;
    }

    StreamHandle
    objectEvidenceReadDetailStreaming(const Query& request,
                std::function<void(const ObjectDetail&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<ObjectDetail>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, ObjectDetail* out) {
            return decodeObjectEvidenceReadDetailStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<ObjectDetail>>(
            StreamPushHolder<ObjectDetail>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeObjectEvidenceReadDetailStream(
            executor_->handle(), request,
            &StreamPushState<ObjectDetail>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle]() {
            push->cancelled = true;
            if (ctx_handle) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    std::future<Result<Identifier>>
    objectSolutionEvidenceCreateRequirementAsync(const ObjectEvidenceRequirement& request) {
        auto state = std::make_shared<UnaryState<Identifier>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Identifier* out) {
            return decodeObjectSolutionEvidenceCreateRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Identifier>>(UnaryHolder<Identifier>{state});
        const pcl_status_t rc = invokeObjectSolutionEvidenceCreateRequirement(
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
    objectSolutionEvidenceReadRequirementStreaming(const Query& request,
                std::function<void(const ObjectEvidenceRequirement&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<ObjectEvidenceRequirement>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, ObjectEvidenceRequirement* out) {
            return decodeObjectSolutionEvidenceReadRequirementStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<ObjectEvidenceRequirement>>(
            StreamPushHolder<ObjectEvidenceRequirement>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeObjectSolutionEvidenceReadRequirementStream(
            executor_->handle(), request,
            &StreamPushState<ObjectEvidenceRequirement>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle]() {
            push->cancelled = true;
            if (ctx_handle) (void)pcl_stream_cancel(ctx_handle);
        }};
    }

    std::future<Result<Ack>>
    objectSolutionEvidenceUpdateRequirementAsync(const ObjectEvidenceRequirement& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodeObjectSolutionEvidenceUpdateRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokeObjectSolutionEvidenceUpdateRequirement(
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
    objectSolutionEvidenceDeleteRequirementAsync(const Identifier& request) {
        auto state = std::make_shared<UnaryState<Ack>>();
        auto future = state->promise.get_future();
        state->decoder = [](const pcl_msg_t* msg, Ack* out) {
            return decodeObjectSolutionEvidenceDeleteRequirementResponse(msg, out);
        };
        auto holder = std::make_unique<UnaryHolder<Ack>>(UnaryHolder<Ack>{state});
        const pcl_status_t rc = invokeObjectSolutionEvidenceDeleteRequirement(
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
    objectSourceCapabilityReadCapabilityStreaming(const Query& request,
                std::function<void(const Capability&)> on_frame,
                std::function<void(pcl_status_t)> on_end = {}) {
        auto push = std::make_shared<StreamPushState<Capability>>();
        push->on_frame  = std::move(on_frame);
        push->on_end    = std::move(on_end);
        push->decoder   = [](const pcl_msg_t* msg, Capability* out) {
            return decodeObjectSourceCapabilityReadCapabilityStreamFrame(msg, out);
        };
        auto holder = std::make_unique<StreamPushHolder<Capability>>(
            StreamPushHolder<Capability>{push});
        pcl_stream_context_t* ctx_handle = nullptr;
        const pcl_status_t rc = invokeObjectSourceCapabilityReadCapabilityStream(
            executor_->handle(), request,
            &StreamPushState<Capability>::trampoline, holder.get(),
            &ctx_handle, nullptr, content_type_.c_str());
        if (rc != PCL_OK && rc != PCL_STREAMING) {
            return StreamHandle{};
        }
        (void)holder.release();
        return StreamHandle{[push, ctx_handle]() {
            push->cancelled = true;
            if (ctx_handle) (void)pcl_stream_cancel(ctx_handle);
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

} // namespace pyramid::components::tactical_objects::services::consumed
