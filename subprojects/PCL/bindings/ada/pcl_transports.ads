--  RAII wrappers around the reference PCL transport adapters.
--
--  Each transport here owns one ``pcl_*_transport_t`` handle and exposes
--  three opinionated operations alongside the type-specific factories:
--
--   * ``Use_As_Default`` - install the transport as the executor's default
--     remote path (``pcl_executor_set_transport``).
--   * ``Register``       - bind the transport under a logical peer ID
--     (``pcl_executor_register_transport``).  Use this for multi-peer
--     setups; the peer ID is the join key for routes and ingress filters.
--   * ``Start_Gateway``  - configure, activate, and add the transport's
--     "gateway container" to the executor.  Required on any side that
--     exposes a remote-callable service.
--
--  GATEWAY CONCEPT
--
--  PCL's transports keep their I/O (TCP recv/send threads, shared-memory
--  poll thread) off the executor thread so the deterministic tick loop
--  stays predictable.  When an inbound *service request* lands on a
--  transport thread it cannot directly invoke the handler -- handlers must
--  run on the executor.  The bridge between the two is a synthetic
--  container called the "gateway":
--
--   1. The transport thread receives the request and republishes it on an
--      internal topic that only the gateway subscribes to (with
--      ``PCL_ROUTE_REMOTE``).
--   2. On the next executor spin, the gateway's subscriber callback wakes
--      up, looks up the matching ``provided`` service handler in the
--      executor, and invokes it -- honouring per-peer routing and
--      allow-lists.
--   3. The handler's response is handed back to the transport for framing
--      and transmission to the originating peer.
--
--  Pure publishers and pure clients can omit ``Start_Gateway``; only sides
--  that *serve* requests need it.  The socket transport's gateway exists
--  only on server-mode handles, while the shared-memory transport is
--  symmetric and any participant may start a gateway.

with Ada.Finalization;
with Ada.Strings.Unbounded;
with Interfaces.C;
with Pcl_Bindings;
with Pcl_Component;
with System;

package Pcl_Transports is
  --  Optional knobs for ``Create_Client``.  Defaults reproduce the legacy
  --  single-shot connect; populate fields to enable retry / auto-reconnect.
  --  See ``doc/guides/peer_transport_configuration.md`` section 8.1.
  type Socket_Client_Options is record
    Connect_Timeout_Ms : Interfaces.C.unsigned := 0;
    Max_Retries        : Interfaces.C.unsigned := 0;
    Auto_Reconnect     : Boolean := False;
    State_Callback     : Pcl_Bindings.Pcl_Socket_State_Cb_Access := null;
    State_Callback_Data : System.Address := System.Null_Address;
  end record;

  --  TCP socket transport -- one connection per instance.  Use one server
  --  per inbound peer and one client per outbound peer for fan-out.
  type Socket_Transport is new Ada.Finalization.Limited_Controlled with private;

  --  Listen on ``Port`` (0 = ephemeral) and block until one client
  --  connects.  See ``Pcl_Bindings.Create_Socket_Server``.
  procedure Create_Server
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor;
     Port : Interfaces.C.unsigned_short := 0);

  --  Variant that returns the bound port (useful when ``Port`` is 0).
  procedure Create_Server
    (This       : in out Socket_Transport;
     Exec       : in out Pcl_Component.Executor;
     Port       : Interfaces.C.unsigned_short;
     Bound_Port : out Interfaces.C.unsigned_short);

  --  Connect to ``Host``:``Port``.  ``Options`` defaults to the legacy
  --  single-shot connect; populate it to enable retry / auto-reconnect.
  procedure Create_Client
    (This    : in out Socket_Transport;
     Exec    : in out Pcl_Component.Executor;
     Host    : String;
     Port    : Interfaces.C.unsigned_short;
     Options : Socket_Client_Options := (others => <>));

  --  Raw C handles.  Use these only for interop scenarios the wrapper does
  --  not yet cover; everyday code should not need them.
  function Handle(This : Socket_Transport) return Pcl_Bindings.Pcl_Socket_Transport_Access;
  function Transport(This : Socket_Transport) return Pcl_Bindings.Pcl_Transport_Const_Access;

  --  Gateway container for inbound service requests (server-mode only,
  --  ``null`` for client transports).  See the GATEWAY CONCEPT block at the
  --  top of the package; ``Start_Gateway`` is the recommended way to wire
  --  it up.
  function Gateway_Container(This : Socket_Transport) return Pcl_Bindings.Pcl_Container_Access;

  --  TCP port bound by this transport (assigned ephemeral port for
  --  ``Port = 0`` servers, configured value otherwise).
  function Port(This : Socket_Transport) return Interfaces.C.unsigned_short;

  --  Thread-safe view of the current connection state.
  function State(This : Socket_Transport) return Pcl_Bindings.Pcl_Socket_State;

  --  Bind the logical peer ID used for ingress filtering and route
  --  matching.  This is the same name passed to ``Register``.
  procedure Set_Peer_Id(This : in out Socket_Transport; Peer_Id : String);

  --  Install as the executor's default remote transport.  Equivalent to
  --  ``Pcl_Component.Set_Transport``; convenient for single-peer setups.
  procedure Use_As_Default
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor);

  --  Register under the peer ID configured on the transport.  Raises
  --  ``Pcl_Component.Pcl_Error`` if the peer ID has not been set.
  procedure Register
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor);

  --  Set the peer ID and register in one step.
  procedure Register
    (This    : in out Socket_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String);

  --  Configure, activate, and add ``Gateway_Container`` to ``Exec``.  Call
  --  this on the server side whenever the executor exposes a service that
  --  remote clients may invoke; pure publishers and pure clients must omit
  --  it.  Raises ``Pcl_Component.Pcl_Error`` for client-mode handles, which
  --  have no gateway container.
  procedure Start_Gateway
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor);

  --  UDP datagram transport (publish/subscribe only).  Service RPC and
  --  streaming are deliberately not supported; use the socket transport
  --  for those.  ``Start_Gateway`` is therefore not exposed: there is no
  --  inbound RPC to dispatch.
  type Udp_Transport is new Ada.Finalization.Limited_Controlled with private;

  --  Bind ``Local_Port`` and target ``Remote_Host``:``Remote_Port``.  One
  --  instance addresses one peer; instantiate one per peer for fan-out.
  procedure Create
    (This        : in out Udp_Transport;
     Exec        : in out Pcl_Component.Executor;
     Local_Port  : Interfaces.C.unsigned_short;
     Remote_Host : String;
     Remote_Port : Interfaces.C.unsigned_short);

  function Handle(This : Udp_Transport) return Pcl_Bindings.Pcl_Udp_Transport_Access;
  function Transport(This : Udp_Transport) return Pcl_Bindings.Pcl_Transport_Const_Access;
  function Local_Port(This : Udp_Transport) return Interfaces.C.unsigned_short;

  procedure Set_Peer_Id(This : in out Udp_Transport; Peer_Id : String);
  procedure Use_As_Default
    (This : in out Udp_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This : in out Udp_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This    : in out Udp_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String);

  --  Shared-memory bus transport.  Multiple processes on the same host join
  --  a logical bus identified by ``Bus_Name``; each holds a mailbox slot
  --  keyed by ``Participant_Id``.
  --
  --  Unlike the asymmetric socket transport (one server, many clients), the
  --  shared-memory bus is symmetric: every participant can publish, every
  --  participant can host services, and the gateway is needed on any side
  --  that exposes ``provided`` services to other participants on the bus.
  type Shared_Memory_Transport is new Ada.Finalization.Limited_Controlled with private;

  --  Join (or create) the bus named ``Bus_Name`` with ``Participant_Id``
  --  and bind it to ``Exec``.  Participant IDs must be unique within a bus;
  --  collisions raise ``Pcl_Component.Pcl_Error``.
  procedure Create
    (This           : in out Shared_Memory_Transport;
     Exec           : in out Pcl_Component.Executor;
     Bus_Name       : String;
     Participant_Id : String);

  function Handle
    (This : Shared_Memory_Transport)
      return Pcl_Bindings.Pcl_Shared_Memory_Transport_Access;
  function Transport
    (This : Shared_Memory_Transport)
      return Pcl_Bindings.Pcl_Transport_Const_Access;

  --  Gateway container for inbound shared-memory service requests.
  --
  --  In the shared-memory transport the gateway plays the same role as in
  --  the socket transport (see GATEWAY CONCEPT at the top of this package):
  --  it lets a transport-thread service request be dispatched on the
  --  executor thread.  The shared-memory implementation does this by
  --  republishing the request as remote ingress on a private internal
  --  topic that only the gateway subscribes to.  Activate the gateway via
  --  ``Start_Gateway`` whenever this participant offers remote-callable
  --  services; pure publishers and pure clients can omit it.
  function Gateway_Container
    (This : Shared_Memory_Transport) return Pcl_Bindings.Pcl_Container_Access;

  --  Install as the executor's default transport.  Most useful when the
  --  participant talks to the bus as a whole and does not address peers
  --  individually.
  procedure Use_As_Default
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor);

  --  Register the transport under the participant ID set at ``Create``
  --  time, or under an explicit ``Peer_Id``.
  procedure Register
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This    : in out Shared_Memory_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String);

  --  Configure, activate, and add ``Gateway_Container`` to ``Exec``.
  --  Required on any participant that exposes ``provided`` services to
  --  the bus; without this call the executor never sees inbound service
  --  requests originating from other participants.
  procedure Start_Gateway
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor);

private
  type Socket_Transport is new Ada.Finalization.Limited_Controlled with record
    Handle  : Pcl_Bindings.Pcl_Socket_Transport_Access := null;
    Peer_Id : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  overriding procedure Finalize(This : in out Socket_Transport);

  type Udp_Transport is new Ada.Finalization.Limited_Controlled with record
    Handle  : Pcl_Bindings.Pcl_Udp_Transport_Access := null;
    Peer_Id : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  overriding procedure Finalize(This : in out Udp_Transport);

  type Shared_Memory_Transport is new Ada.Finalization.Limited_Controlled with record
    Handle         : Pcl_Bindings.Pcl_Shared_Memory_Transport_Access := null;
    Participant_Id : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  overriding procedure Finalize(This : in out Shared_Memory_Transport);
end Pcl_Transports;
