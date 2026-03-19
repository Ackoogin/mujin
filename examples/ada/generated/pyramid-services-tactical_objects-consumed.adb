--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Consumed

with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;

package body Pyramid.Services.Tactical_Objects.Consumed is

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

   --  -- Object_Evidence_Service ------------------------------------
   procedure Handle_Read_Detail
     (Request  : in  Query;
      Response : out Object_Detail_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Detail;

   --  -- Object_Solution_Evidence_Service ------------------------------------
   procedure Handle_Create_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Identifier;
   end Handle_Create_Requirement;

   procedure Handle_Read_Requirement
     (Request  : in  Query;
      Response : out Object_Evidence_Requirement_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Evidence_Requirement_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Requirement;

   procedure Handle_Update_Requirement
     (Request  : in  Object_Evidence_Requirement;
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

   --  -- Object_Source_Capability_Service ------------------------------------
   procedure Handle_Read_Capability
     (Request  : in  Query;
      Response : out Identifier_Array)
   is
      pragma Unreferenced (Request);
      Empty : Identifier_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Capability;

   --  -- PCL binding implementations -------------------------------

   procedure Subscribe_Object_Evidence
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address)
   is
      Topic  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Object_Evidence);
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
   end Subscribe_Object_Evidence;

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
         when Ch_Read_Detail =>
            null;  --  TODO: deserialise, call Handle_Read_Detail
         when Ch_Create_Requirement =>
            null;  --  TODO: deserialise, call Handle_Create_Requirement
         when Ch_Read_Requirement =>
            null;  --  TODO: deserialise, call Handle_Read_Requirement
         when Ch_Update_Requirement =>
            null;  --  TODO: deserialise, call Handle_Update_Requirement
         when Ch_Delete_Requirement =>
            null;  --  TODO: deserialise, call Handle_Delete_Requirement
         when Ch_Read_Capability =>
            null;  --  TODO: deserialise, call Handle_Read_Capability
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Consumed;
