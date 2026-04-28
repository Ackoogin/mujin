--  Strongly-typed publish/decode helper around an existing codec pair.
--
--  ``Pcl_Component.Port`` and ``Pcl_Component.Message_View`` both deal in raw
--  byte buffers plus a content-type string.  When an application already has
--  ``Encode`` / ``Decode_Payload`` subprograms for a given Ada record, this
--  generic removes the boilerplate of:
--
--   * picking a ``Content_Type_Kind`` and translating it to a wire string,
--   * constructing the ``Pcl_Msg`` and calling ``Port.Publish``,
--   * calling ``Decode_Payload`` against ``Message_View``'s raw pointer.
--
--  The package does not own the codec state; it just routes between an Ada
--  record type and the underlying byte-oriented PCL ports.
--
--  Typical use::
--
--    package Json_State_Port is new Pcl_Typed_Ports
--      (Payload_Type   => State_Update,
--       Encode         => Encode_State_Update,
--       Decode_Payload => Decode_State_Update);
--
--  Then ``Json_State_Port.Publish`` and ``Json_State_Port.Decode`` work
--  directly on ``State_Update`` values.

with Pcl_Bindings;
with Pcl_Component;
with Pcl_Content_Types;

generic
  --  Application-level Ada type carried over the wire.
  type Payload_Type is private;

  --  Caller-supplied serialiser.  ``Content_Type`` is the canonical wire
  --  string (e.g. ``Pcl_Content_Types.Json_Content_Type``) so the codec can
  --  branch when it supports several formats.
  with function Encode
    (Payload      : Payload_Type;
     Content_Type : String) return String;

  --  Caller-supplied deserialiser invoked against the raw ``Pcl_Msg``
  --  delivered by PCL.  Implementations may inspect ``Msg.type_name`` to
  --  pick a wire format.
  with function Decode_Payload
    (Msg : access constant Pcl_Bindings.Pcl_Msg) return Payload_Type;
package Pcl_Typed_Ports is
  --  Encode ``Payload`` with the chosen ``Format`` and publish on ``Port``.
  --  ``Format`` defaults to JSON to match the most common PYRAMID setup.
  procedure Publish
    (Port    : Pcl_Component.Port;
     Payload : Payload_Type;
     Format  : Pcl_Content_Types.Content_Type_Kind := Pcl_Content_Types.Json);

  --  Decode the raw bytes behind ``Message`` using the generic ``Decode_Payload``.
  function Decode(Message : Pcl_Component.Message_View) return Payload_Type;
end Pcl_Typed_Ports;
