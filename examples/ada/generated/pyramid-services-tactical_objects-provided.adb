--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Provided

with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;

package body Pyramid.Services.Tactical_Objects.Provided is

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

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

   --  -- Matching_Objects_Service ------------------------------------
   procedure Handle_Read_Match
     (Request  : in  Query;
      Response : out Object_Match_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Match_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Match;

   --  -- Object_Of_Interest_Service ------------------------------------
   procedure Handle_Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Identifier;
   end Handle_Create_Requirement;

   procedure Handle_Read_Requirement
     (Request  : in  Query;
      Response : out Object_Interest_Requirement_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Interest_Requirement_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Requirement;

   procedure Handle_Update_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := Ack_Ok;
   end Handle_Update_Requirement;

   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := Ack_Ok;
   end Handle_Delete_Requirement;

   --  -- Specific_Object_Detail_Service ------------------------------------
   procedure Handle_Read_Detail
     (Request  : in  Query;
      Response : out Object_Detail_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Detail;

   --  -- PCL binding implementations -------------------------------

   procedure Subscribe_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address)
   is
      Topic  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Entity_Matches);
      Type_N : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("application/json");
      Port   : Pcl_Bindings.Pcl_Port_Access;
      pragma Unreferenced (Port);
   begin
      Port := Pcl_Bindings.Add_Subscriber
        (Container => Container,
         Topic     => Topic,
         Type_Name => Type_N,
         Callback  => Callback,
         User_Data => User_Data);
      Interfaces.C.Strings.Free (Topic);
      Interfaces.C.Strings.Free (Type_N);
   end Subscribe_Entity_Matches;

   procedure Subscribe_Evidence_Requirements
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address)
   is
      Topic  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Evidence_Requirements);
      Type_N : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("application/json");
      Port   : Pcl_Bindings.Pcl_Port_Access;
      pragma Unreferenced (Port);
   begin
      Port := Pcl_Bindings.Add_Subscriber
        (Container => Container,
         Topic     => Topic,
         Type_Name => Type_N,
         Callback  => Callback,
         User_Data => User_Data);
      Interfaces.C.Strings.Free (Topic);
      Interfaces.C.Strings.Free (Type_N);
   end Subscribe_Evidence_Requirements;

   procedure Invoke_Read_Match
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : String;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Match);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Request'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Remote_Async
        (Transport, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Read_Match;

   procedure Invoke_Create_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : String;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Create_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Request'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Remote_Async
        (Transport, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Create_Requirement;

   procedure Invoke_Read_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : String;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Request'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Remote_Async
        (Transport, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Read_Requirement;

   procedure Invoke_Update_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : String;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Update_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Request'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Remote_Async
        (Transport, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Update_Requirement;

   procedure Invoke_Delete_Requirement
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : String;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Delete_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Request'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Remote_Async
        (Transport, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Delete_Requirement;

   procedure Invoke_Read_Detail
     (Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Request   : String;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Detail);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Request'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Remote_Async
        (Transport, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Read_Detail;

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural)
   is
      pragma Unreferenced (Request_Buf, Request_Size);
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Read_Match =>
            null;  --  TODO: deserialise, call Handle_Read_Match
         when Ch_Create_Requirement =>
            null;  --  TODO: deserialise, call Handle_Create_Requirement
         when Ch_Read_Requirement =>
            null;  --  TODO: deserialise, call Handle_Read_Requirement
         when Ch_Update_Requirement =>
            null;  --  TODO: deserialise, call Handle_Update_Requirement
         when Ch_Delete_Requirement =>
            null;  --  TODO: deserialise, call Handle_Delete_Requirement
         when Ch_Read_Detail =>
            null;  --  TODO: deserialise, call Handle_Read_Detail
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Provided;
