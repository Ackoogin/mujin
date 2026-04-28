--  Named content-type constants for PCL message wire formats.
--
--  PCL itself is payload-agnostic: every message is a byte buffer plus a
--  ``type_name`` string.  Components and bindings agree on a content type via
--  the strings declared here so that publishers, subscribers, and generated
--  PYRAMID Ada bindings can interoperate without re-defining these values
--  locally.

package Pcl_Content_Types is
  --  RFC 8259 JSON.
  Json_Content_Type        : constant String := "application/json";

  --  Google FlatBuffers binary payload.
  Flatbuffers_Content_Type : constant String := "application/flatbuffers";

  --  Google Protocol Buffers binary payload.
  Protobuf_Content_Type    : constant String := "application/protobuf";

  --  gRPC framed payload (length-prefixed protobuf with compression byte).
  Grpc_Content_Type        : constant String := "application/grpc";

  --  ROS 2 transport payload (CDR-encoded message).
  Ros2_Content_Type        : constant String := "application/ros2";

  --  Strongly-typed enum mirror of the content-type strings above.
  --
  --  Prefer this when callers want compile-time exhaustiveness checks instead
  --  of repeating the constant strings.
  type Content_Type_Kind is
    (Json,
     Flatbuffers,
     Protobuf,
     Grpc,
     Ros2);

  --  Return the canonical wire string for ``Kind`` (always one of the
  --  ``*_Content_Type`` constants).
  function Image(Kind : Content_Type_Kind) return String;

  --  True when ``Content_Type`` matches one of the canonical wire strings.
  function Is_Known(Content_Type : String) return Boolean;

  --  Inverse of ``Image``.  Raises ``Constraint_Error`` when ``Content_Type``
  --  does not match a known constant; gate calls with ``Is_Known`` if the
  --  caller wants to handle unknown formats gracefully.
  function Kind_Of(Content_Type : String) return Content_Type_Kind;
end Pcl_Content_Types;
