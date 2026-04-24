package Pcl_Content_Types is
  Json_Content_Type        : constant String := "application/json";
  Flatbuffers_Content_Type : constant String := "application/flatbuffers";
  Protobuf_Content_Type    : constant String := "application/protobuf";
  Grpc_Content_Type        : constant String := "application/grpc";
  Ros2_Content_Type        : constant String := "application/ros2";

  type Content_Type_Kind is
    (Json,
     Flatbuffers,
     Protobuf,
     Grpc,
     Ros2);

  function Image(Kind : Content_Type_Kind) return String;
  function Is_Known(Content_Type : String) return Boolean;
  function Kind_Of(Content_Type : String) return Content_Type_Kind;
end Pcl_Content_Types;
