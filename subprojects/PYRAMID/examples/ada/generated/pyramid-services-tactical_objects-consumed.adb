--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Consumed

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;
with Pyramid.Data_Model.Common.Types_Codec;  use Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Tactical.Types_Codec;  use Pyramid.Data_Model.Tactical.Types_Codec;
with Pyramid.Services.Tactical_Objects.Json_Codec;
with Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;

package body Pyramid.Services.Tactical_Objects.Consumed is
   use type System.Address;
   use type Interfaces.C.Strings.chars_ptr;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   type Service_Handlers_Access is access constant Service_Handlers;

   function To_Handlers is new
     Ada.Unchecked_Conversion (System.Address, Service_Handlers_Access);

   function Handler_Address
     (Handlers : access constant Service_Handlers) return System.Address is
   begin
      if Handlers = null then
         return System.Null_Address;
      end if;

      return Handlers.all'Address;
   end Handler_Address;

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String
   is
      use System.Storage_Elements;
      type Char_Array is array (1 .. Natural (Size)) of Character;
      pragma Pack (Char_Array);
      Chars : Char_Array;
      for Chars'Address use Data;
      pragma Import (Ada, Chars);
   begin
      return String (Chars);
   end Msg_To_String;

   package Flatbuffers_Codec renames Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;

   --  -- Object_Evidence_Service ------------------------------------
   function Default_Handle_Read_Detail
     (Request : Query) return Object_Detail_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Detail;

   --  -- Object_Solution_Evidence_Service ------------------------------------
   procedure Default_Handle_Create_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Create_Requirement;

   function Default_Handle_Read_Requirement
     (Request : Query) return Object_Evidence_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Evidence_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Requirement;

   procedure Default_Handle_Update_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Update_Requirement;

   procedure Default_Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Delete_Requirement;

   --  -- Object_Source_Capability_Service ------------------------------------
   function Default_Handle_Read_Capability
     (Request : Query) return Capability_Array
   is
      pragma Unreferenced (Request);
      Empty : Capability_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Capability;

   function Service_Read_Detail
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Detail);

   function Service_Create_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Create_Requirement);

   function Service_Read_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Requirement);

   function Service_Update_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Update_Requirement);

   function Service_Delete_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Delete_Requirement);

   function Service_Read_Capability
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Capability);

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json")
   is
      Handler_Ptr : constant System.Address := Handler_Address (Handlers);
   begin
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Detail);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Detail'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Create_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Create_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Update_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Update_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Delete_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Delete_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Capability);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Capability'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
   end Register_Services;

   function Service_Read_Detail
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Read_Detail,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Read_Detail;

   function Service_Create_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Create_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Create_Requirement;

   function Service_Read_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Read_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Read_Requirement;

   function Service_Update_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Update_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Update_Requirement;

   function Service_Delete_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Delete_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Delete_Requirement;

   function Service_Read_Capability
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Read_Capability,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Read_Capability;

   --  -- PCL binding implementations -------------------------------

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : Pyramid.Services.Tactical_Objects.Wire_Types.Object_Evidence;
      Content_Type : String := "application/json")
   is
      Json_Payload : constant String := Pyramid.Services.Tactical_Objects.Json_Codec.To_Json (Payload);
      Wire_Payload : constant String :=
        (if Content_Type = "" or else Content_Type = "application/json"
         then Json_Payload
         elsif Content_Type = "application/flatbuffers"
         then Flatbuffers_Codec.To_Binary_Object_Evidence (Payload)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
   begin
      Publish_Object_Evidence (Exec, Wire_Payload, Content_Type);
   end Publish_Object_Evidence;

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : String;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Topic_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Object_Evidence);
      Msg       : aliased Pcl_Bindings.Pcl_Msg;
      Status    : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Payload_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Payload_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Publish (Exec, Topic_C, Msg'Access);
      if Payload_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Payload_C);
      end if;
      Interfaces.C.Strings.Free (Topic_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Publish_Object_Evidence;

   procedure Copy_To_Buf
     (S    : in  String;
      Buf  : out System.Address;
      Size : out Natural)
   is
      C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (S);
   begin
      Buf  := To_Address (C);
      Size := S'Length;
   end Copy_To_Buf;

   procedure Dispatch
     (Handlers      : access constant Service_Handlers := null;
      Channel       : in  Service_Channel;
      Request_Buf   : in  System.Address;
      Request_Size  : in  Natural;
      Content_Type  : in  String := "application/json";
      Response_Buf  : out System.Address;
      Response_Size : out Natural)
   is
      Request_Payload : constant String :=
        Msg_To_String (Request_Buf, Interfaces.C.unsigned (Request_Size));
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Read_Detail =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Object_Detail_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Detail /= null
                  then Handlers.On_Read_Detail.all (Req)
                  else Default_Handle_Read_Detail (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Object_Detail_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Create_Requirement =>
            declare
               Req : constant Object_Evidence_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Object_Evidence_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Identifier;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Create_Requirement /= null then
                  Handlers.On_Create_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Create_Requirement (Req, Rsp);
               end if;
               Json_Response := To_String (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Read_Requirement =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Object_Evidence_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Requirement /= null
                  then Handlers.On_Read_Requirement.all (Req)
                  else Default_Handle_Read_Requirement (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Object_Evidence_Requirement_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Update_Requirement =>
            declare
               Req : constant Object_Evidence_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Object_Evidence_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Update_Requirement /= null then
                  Handlers.On_Update_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Update_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Delete_Requirement =>
            declare
               Req : constant Identifier :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then To_Unbounded_String (Request_Payload)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Delete_Requirement /= null then
                  Handlers.On_Delete_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Delete_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Read_Capability =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Capability_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Capability /= null
                  then Handlers.On_Read_Capability.all (Req)
                  else Default_Handle_Read_Capability (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Capability_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Consumed;
