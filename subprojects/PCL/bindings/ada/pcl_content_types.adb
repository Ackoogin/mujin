package body Pcl_Content_Types is
  function Image(Kind : Content_Type_Kind) return String is
  begin
    case Kind is
      when Json =>
        return Json_Content_Type;
      when Flatbuffers =>
        return Flatbuffers_Content_Type;
      when Protobuf =>
        return Protobuf_Content_Type;
      when Grpc =>
        return Grpc_Content_Type;
      when Ros2 =>
        return Ros2_Content_Type;
    end case;
  end Image;

  function Is_Known(Content_Type : String) return Boolean is
  begin
    return Content_Type = Json_Content_Type
      or else Content_Type = Flatbuffers_Content_Type
      or else Content_Type = Protobuf_Content_Type
      or else Content_Type = Grpc_Content_Type
      or else Content_Type = Ros2_Content_Type;
  end Is_Known;

  function Kind_Of(Content_Type : String) return Content_Type_Kind is
  begin
    if Content_Type = Json_Content_Type then
      return Json;
    elsif Content_Type = Flatbuffers_Content_Type then
      return Flatbuffers;
    elsif Content_Type = Protobuf_Content_Type then
      return Protobuf;
    elsif Content_Type = Grpc_Content_Type then
      return Grpc;
    elsif Content_Type = Ros2_Content_Type then
      return Ros2;
    end if;

    raise Constraint_Error with "unknown content type: " & Content_Type;
  end Kind_Of;
end Pcl_Content_Types;
