--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Provided

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;
with Pyramid_Data_Model_Common_Types_Codec;  use Pyramid_Data_Model_Common_Types_Codec;
with Pyramid_Data_Model_Tactical_Types_Codec;  use Pyramid_Data_Model_Tactical_Types_Codec;

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

   --  -- TacticalObjectService ------------------------------------
   procedure Handle_Create_Tactical_Object
     (Request  : in  Tactical_Object;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Handle_Create_Tactical_Object;

   function Handle_Read_Tactical_Object
     (Request : Tactical_Object_Query) return Tactical_Object_Array
   is
      pragma Unreferenced (Request);
      Empty : Tactical_Object_Array (1 .. 0);
   begin
      return Empty;
   end Handle_Read_Tactical_Object;

   procedure Handle_Update_Tactical_Object
     (Request  : in  Tactical_Object;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Handle_Update_Tactical_Object;

   procedure Handle_Delete_Tactical_Object
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Handle_Delete_Tactical_Object;

   --  -- ZoneService ------------------------------------
   procedure Handle_Create_Zone
     (Request  : in  Zone;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Handle_Create_Zone;

   function Handle_Read_Zone
     (Request : Query) return Zone_Array
   is
      pragma Unreferenced (Request);
      Empty : Zone_Array (1 .. 0);
   begin
      return Empty;
   end Handle_Read_Zone;

   procedure Handle_Update_Zone
     (Request  : in  Zone;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Handle_Update_Zone;

   procedure Handle_Delete_Zone
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Handle_Delete_Zone;

   --  -- ObservationIngressService ------------------------------------
   procedure Handle_Create_Observation
     (Request  : in  Observation;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Handle_Create_Observation;

   --  -- PCL binding implementations -------------------------------

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : String)
   is
      use type Pcl_Bindings.Pcl_Status;
      Payload_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Payload);
      Topic_C   : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Object_Evidence);
      Msg       : aliased Pcl_Bindings.Pcl_Msg;
      Status    : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      Msg.Data      := To_Address (Payload_C);
      Msg.Size      := Interfaces.C.unsigned (Payload'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String ("application/json");
      Status := Pcl_Bindings.Publish (Exec, Topic_C, Msg'Access);
      Interfaces.C.Strings.Free (Payload_C);
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
         when Ch_Create_Tactical_Object =>
            declare
               Req : constant Tactical_Object :=
                 From_Json (Req_Str, null);
               Rsp : Identifier;
            begin
               Handle_Create_Tactical_Object (Req, Rsp);
               Copy_To_Buf (To_String (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Read_Tactical_Object =>
            declare
               Req : constant Tactical_Object_Query :=
                 From_Json (Req_Str, null);
               Rsp : constant Tactical_Object_Array :=
                 Handle_Read_Tactical_Object (Req);
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
         when Ch_Update_Tactical_Object =>
            declare
               Req : constant Tactical_Object :=
                 From_Json (Req_Str, null);
               Rsp : Ack;
            begin
               Handle_Update_Tactical_Object (Req, Rsp);
               Copy_To_Buf (To_Json (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Delete_Tactical_Object =>
            declare
               Req : constant Identifier :=
                 To_Unbounded_String (Req_Str);
               Rsp : Ack;
            begin
               Handle_Delete_Tactical_Object (Req, Rsp);
               Copy_To_Buf (To_Json (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Create_Zone =>
            declare
               Req : constant Zone :=
                 From_Json (Req_Str, null);
               Rsp : Identifier;
            begin
               Handle_Create_Zone (Req, Rsp);
               Copy_To_Buf (To_String (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Read_Zone =>
            declare
               Req : constant Query :=
                 From_Json (Req_Str, null);
               Rsp : constant Zone_Array :=
                 Handle_Read_Zone (Req);
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
         when Ch_Update_Zone =>
            declare
               Req : constant Zone :=
                 From_Json (Req_Str, null);
               Rsp : Ack;
            begin
               Handle_Update_Zone (Req, Rsp);
               Copy_To_Buf (To_Json (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Delete_Zone =>
            declare
               Req : constant Identifier :=
                 To_Unbounded_String (Req_Str);
               Rsp : Ack;
            begin
               Handle_Delete_Zone (Req, Rsp);
               Copy_To_Buf (To_Json (Rsp),
                 Response_Buf, Response_Size);
            end;
         when Ch_Create_Observation =>
            declare
               Req : constant Observation :=
                 From_Json (Req_Str, null);
               Rsp : Identifier;
            begin
               Handle_Create_Observation (Req, Rsp);
               Copy_To_Buf (To_String (Rsp),
                 Response_Buf, Response_Size);
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Provided;
