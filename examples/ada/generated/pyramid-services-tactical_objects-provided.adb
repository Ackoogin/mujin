--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Provided

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;
with Pyramid_Data_Model_Common_Types_Codec;  use Pyramid_Data_Model_Common_Types_Codec;
with Pyramid_Data_Model_Tactical_Types_Codec;  use Pyramid_Data_Model_Tactical_Types_Codec;
with Pyramid.Services.Tactical_Objects.Json_Codec;

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
   function Handle_Read_Match
     (Request : Query) return Object_Match_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Match_Array (1 .. 0);
   begin
      return Empty;
   end Handle_Read_Match;

   --  -- Object_Of_Interest_Service ------------------------------------
   procedure Handle_Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Handle_Create_Requirement;

   function Handle_Read_Requirement
     (Request : Query) return Object_Interest_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Interest_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Handle_Read_Requirement;

   procedure Handle_Update_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Handle_Update_Requirement;

   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Handle_Delete_Requirement;

   --  -- Specific_Object_Detail_Service ------------------------------------
   function Handle_Read_Detail
     (Request : Query) return Object_Detail_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      return Empty;
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
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload : constant String := To_Json (Request);
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Match);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Read_Match;

   procedure Invoke_Create_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Pyramid.Services.Tactical_Objects.Json_Codec.Create_Requirement_Request;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload : constant String := Pyramid.Services.Tactical_Objects.Json_Codec.To_Json (Request);
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Create_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Create_Requirement;

   procedure Invoke_Read_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload : constant String := To_Json (Request);
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Read_Requirement;

   procedure Invoke_Update_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Pyramid.Services.Tactical_Objects.Json_Codec.Create_Requirement_Request;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload : constant String := Pyramid.Services.Tactical_Objects.Json_Codec.To_Json (Request);
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Update_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Update_Requirement;

   procedure Invoke_Delete_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload : constant String := To_String (Request);
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Delete_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Delete_Requirement;

   procedure Invoke_Read_Detail
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload : constant String := To_Json (Request);
      Req_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Detail);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Req_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      Interfaces.C.Strings.Free (Req_C);
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Read_Detail;

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
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural)
   is
      Req_Str : constant String := Msg_To_String (Request_Buf,
        Interfaces.C.unsigned (Request_Size));
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Read_Match =>
            declare
               Req : constant Query :=
                 From_Json (Req_Str, null);
               Rsp : constant Object_Match_Array :=
                 Handle_Read_Match (Req);
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Copy_To_Buf (To_String (Acc),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Create_Requirement =>
            declare
               Req : constant Object_Interest_Requirement :=
                 From_Json (Req_Str, null);
               Rsp : Identifier;
            begin
               Handle_Create_Requirement (Req, Rsp);
               Copy_To_Buf (To_String (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Read_Requirement =>
            declare
               Req : constant Query :=
                 From_Json (Req_Str, null);
               Rsp : constant Object_Interest_Requirement_Array :=
                 Handle_Read_Requirement (Req);
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Copy_To_Buf (To_String (Acc),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Update_Requirement =>
            declare
               Req : constant Object_Interest_Requirement :=
                 From_Json (Req_Str, null);
               Rsp : Ack;
            begin
               Handle_Update_Requirement (Req, Rsp);
               Copy_To_Buf (To_Json (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Delete_Requirement =>
            declare
               Req : constant Identifier :=
                 To_Unbounded_String (Req_Str);
               Rsp : Ack;
            begin
               Handle_Delete_Requirement (Req, Rsp);
               Copy_To_Buf (To_Json (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Read_Detail =>
            declare
               Req : constant Query :=
                 From_Json (Req_Str, null);
               Rsp : constant Object_Detail_Array :=
                 Handle_Read_Detail (Req);
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Copy_To_Buf (To_String (Acc),
                    Response_Buf, Response_Size);
               end;
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Provided;
