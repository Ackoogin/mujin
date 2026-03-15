--  tactical_objects_service.ads
--
--  EntityActions service interface for the Tactical Objects component.
--  This is the Ada equivalent of the generated service stubs that
--  ada_service_generator.py would produce from the proto IDL.
--
--  Corresponds to proto services:
--    TacticalObjectService  (flow: inout)
--    ZoneService            (flow: inout)
--    ObservationIngressService (flow: in)
--
--  Semantic contract:
--    Handle_Read_Tactical_Object maps to C++ subscribe_interest (PCL service name).
--    The wire name mapping is held in Read_Service_Name below so the client
--    never has a bare string literal for the service.
--
--  JSON serialization:
--    Query_To_Json  — serialise a Tactical_Object_Query to the JSON format
--                     expected by the C++ subscribe_interest handler.
--
--  Frame mapping:
--    Frame_To_Tactical_Object — map a decoded Streaming_Codec.Entity_Update_Frame
--                                to a Tactical_Objects_Types.Tactical_Object.

with Tactical_Objects_Types;  use Tactical_Objects_Types;
with Streaming_Codec;

package Tactical_Objects_Service is

  -- ── EntityActions operation → PCL service name mapping ───────────────────
  --
  --  These constants bridge the semantic EntityActions names to the actual
  --  PCL service names registered by TacticalObjectsComponent.
  --  Update these when Phase 3 (C++ handler renaming) is complete.

  Read_Service_Name   : constant String := "subscribe_interest";
  Create_Service_Name : constant String := "create_object";
  Update_Service_Name : constant String := "update_object";
  Delete_Service_Name : constant String := "delete_object";

  -- ── EntityActions handler stubs ───────────────────────────────────────────
  --  These match the signatures produced by ada_service_generator.py.
  --  Implement them in tactical_objects_service.adb with real logic.

  --  ReadTacticalObject (flow: out — client only receives; server provides)
  --  Serialises the query to JSON for Invoke_Remote over the PCL socket.
  function Build_Read_Request_Json
    (Query : Tactical_Object_Query) return String;

  --  Map a decoded binary frame → typed TacticalObject.
  --  Called in the entity_updates subscriber callback after Decode_Batch.
  function Frame_To_Tactical_Object
    (Frame : Streaming_Codec.Entity_Update_Frame) return Tactical_Object;

  --  Pretty-print a TacticalObject to a human-readable line (for logging).
  function Tactical_Object_Image
    (Obj : Tactical_Object) return String;

  --  Convert Affiliation ordinal byte (from wire) to the Ada enum.
  function Ordinal_To_Affiliation
    (V : Streaming_Codec.Byte) return Affiliation;

  --  Convert ObjectType ordinal byte (from wire) to the Ada enum.
  function Ordinal_To_Object_Type
    (V : Streaming_Codec.Byte) return Object_Type;

  --  Convert LifecycleStatus ordinal byte (from wire) to the Ada enum.
  function Ordinal_To_Lifecycle_Status
    (V : Streaming_Codec.Byte) return Lifecycle_Status;

end Tactical_Objects_Service;
