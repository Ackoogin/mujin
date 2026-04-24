with Pcl_Bindings;
with Pcl_Component;
with Pcl_Content_Types;

generic
  type Payload_Type is private;
  with function Encode
    (Payload      : Payload_Type;
     Content_Type : String) return String;
  with function Decode_Payload
    (Msg : access constant Pcl_Bindings.Pcl_Msg) return Payload_Type;
package Pcl_Typed_Ports is
  procedure Publish
    (Port    : Pcl_Component.Port;
     Payload : Payload_Type;
     Format  : Pcl_Content_Types.Content_Type_Kind := Pcl_Content_Types.Json);

  function Decode(Message : Pcl_Component.Message_View) return Payload_Type;
end Pcl_Typed_Ports;
