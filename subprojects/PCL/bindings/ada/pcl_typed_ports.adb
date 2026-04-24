package body Pcl_Typed_Ports is
  procedure Publish
    (Port    : Pcl_Component.Port;
     Payload : Payload_Type;
     Format  : Pcl_Content_Types.Content_Type_Kind := Pcl_Content_Types.Json) is
  begin
    Pcl_Component.Publish
      (This      => Port,
       Payload   => Encode(Payload, Pcl_Content_Types.Image(Format)),
       Type_Name => Pcl_Content_Types.Image(Format));
  end Publish;

  function Decode(Message : Pcl_Component.Message_View) return Payload_Type is
    Raw_Message : aliased constant Pcl_Bindings.Pcl_Msg :=
      Pcl_Component.To_Raw_Message(Message);
  begin
    return Decode_Payload(Raw_Message'Access);
  end Decode;
end Pcl_Typed_Ports;
