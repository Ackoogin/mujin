--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Consumed

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Plugins;
with System;
with System.Address_To_Access_Conversions;
with System.Storage_Elements;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pyramid.Data_Model.Base.Cabi;  use Pyramid.Data_Model.Base.Cabi;
with Pyramid.Data_Model.Common.Cabi;  use Pyramid.Data_Model.Common.Cabi;
with Pyramid.Data_Model.Tactical.Cabi;  use Pyramid.Data_Model.Tactical.Cabi;

package body Pyramid.Services.Tactical_Objects.Consumed is
   use type System.Address;
   use type Interfaces.C.unsigned;
   use type Interfaces.C.Strings.chars_ptr;
   use type Pcl_Bindings.Pcl_Resp_Cb_Access;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Plugins.Pcl_Codec_Const_Access;
   use type Pcl_Plugins.Pcl_Codec_Decode_Access;
   use type Pcl_Plugins.Pcl_Codec_Encode_Access;
   use type Pcl_Plugins.Pcl_Codec_Free_Msg_Access;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   function Pcl_Codec_Registry_Get_At
     (Registry     : System.Address;
      Content_Type : Interfaces.C.Strings.chars_ptr;
      Index        : Interfaces.C.unsigned)
      return Pcl_Plugins.Pcl_Codec_Const_Access;
   pragma Import (C, Pcl_Codec_Registry_Get_At,
                  "pcl_codec_registry_get_at");

   procedure C_Free (Ptr : System.Address);
   pragma Import (C, C_Free, "free");

   type Service_Handlers_Access is access constant Service_Handlers;

   function To_Handlers is new
     Ada.Unchecked_Conversion (System.Address, Service_Handlers_Access);

   package Geodetic_Position_Pointers is new
     System.Address_To_Access_Conversions (Geodetic_Position);

   package Poly_Area_Pointers is new
     System.Address_To_Access_Conversions (Poly_Area);

   package Achievement_Pointers is new
     System.Address_To_Access_Conversions (Achievement);

   package Requirement_Pointers is new
     System.Address_To_Access_Conversions (Requirement);

   package Capability_Pointers is new
     System.Address_To_Access_Conversions (Capability);

   package Entity_Pointers is new
     System.Address_To_Access_Conversions (Entity);

   package Circle_Area_Pointers is new
     System.Address_To_Access_Conversions (Circle_Area);

   package Point_Pointers is new
     System.Address_To_Access_Conversions (Point);

   package Contraint_Pointers is new
     System.Address_To_Access_Conversions (Contraint);

   package Ack_Pointers is new
     System.Address_To_Access_Conversions (Ack);

   package Query_Pointers is new
     System.Address_To_Access_Conversions (Query);

   package Object_Detail_Pointers is new
     System.Address_To_Access_Conversions (Object_Detail);

   package Object_Evidence_Requirement_Pointers is new
     System.Address_To_Access_Conversions (Object_Evidence_Requirement);

   package Object_Interest_Requirement_Pointers is new
     System.Address_To_Access_Conversions (Object_Interest_Requirement);

   package Object_Match_Pointers is new
     System.Address_To_Access_Conversions (Object_Match);

   package Identifier_Pointers is new
     System.Address_To_Access_Conversions (Identifier);

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

   function Registry_Has_Codec (Content_Type : String) return Boolean is
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
   begin
      if Content_Type = "" then
         Interfaces.C.Strings.Free (Content_C);
         return False;
      end if;
      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get
        (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C);
      Interfaces.C.Strings.Free (Content_C);
      return Codec /= null;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         return False;
   end Registry_Has_Codec;

   procedure Require_Codec (Content_Type : String) is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
   begin
      if not Registry_Has_Codec (Effective) then
         raise Program_Error with
           "fail-closed: no codec plugin registered for content type "
           & Effective;
      end if;
   end Require_Codec;

   function Try_Cabi_Registry_Encode
     (Codec     : Pcl_Plugins.Pcl_Codec_Const_Access;
      Schema_C  : Interfaces.C.Strings.chars_ptr;
      Schema_Id : String;
      Value     : System.Address;
      Msg       : access Pcl_Bindings.Pcl_Msg)
      return Pcl_Bindings.Pcl_Status
   is
   begin
      if Codec = null or else Codec.all.Encode = null then
         return Pcl_Bindings.PCL_ERR_INVALID;
      end if;
      if Schema_Id = "GeodeticPosition" then
         declare
            Native : constant Geodetic_Position_Pointers.Object_Pointer :=
              Geodetic_Position_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Geodetic_Position_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Geodetic_Position (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PolyArea" then
         declare
            Native : constant Poly_Area_Pointers.Object_Pointer :=
              Poly_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Poly_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Poly_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Achievement" then
         declare
            Native : constant Achievement_Pointers.Object_Pointer :=
              Achievement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Achievement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Achievement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Requirement" then
         declare
            Native : constant Requirement_Pointers.Object_Pointer :=
              Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Capability" then
         declare
            Native : constant Capability_Pointers.Object_Pointer :=
              Capability_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Capability_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Capability (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Entity" then
         declare
            Native : constant Entity_Pointers.Object_Pointer :=
              Entity_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Entity_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Entity (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "CircleArea" then
         declare
            Native : constant Circle_Area_Pointers.Object_Pointer :=
              Circle_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Circle_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Circle_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Point" then
         declare
            Native : constant Point_Pointers.Object_Pointer :=
              Point_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Point_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Point (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Contraint" then
         declare
            Native : constant Contraint_Pointers.Object_Pointer :=
              Contraint_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Contraint_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Contraint (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Ack" then
         declare
            Native : constant Ack_Pointers.Object_Pointer :=
              Ack_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Ack_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Ack (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Query" then
         declare
            Native : constant Query_Pointers.Object_Pointer :=
              Query_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Query_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Query (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectDetail" then
         declare
            Native : constant Object_Detail_Pointers.Object_Pointer :=
              Object_Detail_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Detail_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Object_Detail (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectEvidenceRequirement" then
         declare
            Native : constant Object_Evidence_Requirement_Pointers.Object_Pointer :=
              Object_Evidence_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Evidence_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Object_Evidence_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectInterestRequirement" then
         declare
            Native : constant Object_Interest_Requirement_Pointers.Object_Pointer :=
              Object_Interest_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Interest_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Object_Interest_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectMatch" then
         declare
            Native : constant Object_Match_Pointers.Object_Pointer :=
              Object_Match_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Match_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            To_C (Native.all, C_Value);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Free_Object_Match (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Angle" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Length" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Timestamp" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Identifier" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         declare
            Native : constant Identifier_Pointers.Object_Pointer :=
              Identifier_Pointers.To_Pointer (Value);
            S : constant String := To_String (Native.all);
            C_Value : aliased Pyramid.Data_Model.Base.Cabi.Pyramid_Str_T;
            Status : Pcl_Bindings.Pcl_Status;
         begin
            C_Value.Ptr := Interfaces.C.Strings.New_String (S);
            C_Value.Len := Interfaces.C.unsigned (S'Length);
            Status := Codec.all.Encode.all
              (Codec.all.Codec_Ctx, Schema_C, C_Value'Address, Msg);
            Interfaces.C.Strings.Free (C_Value.Ptr);
            return Status;
         end;
      end if;
      if Schema_Id = "Speed" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      if Schema_Id = "Percentage" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Encode.all
           (Codec.all.Codec_Ctx, Schema_C, Value, Msg);
      end if;
      return Pcl_Bindings.PCL_ERR_NOT_FOUND;
   end Try_Cabi_Registry_Encode;

   function Try_Cabi_Registry_Decode
     (Codec     : Pcl_Plugins.Pcl_Codec_Const_Access;
      Schema_C  : Interfaces.C.Strings.chars_ptr;
      Schema_Id : String;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Value     : System.Address)
      return Pcl_Bindings.Pcl_Status
   is
   begin
      if Codec = null or else Codec.all.Decode = null then
         return Pcl_Bindings.PCL_ERR_INVALID;
      end if;
      if Schema_Id = "GeodeticPosition" then
         declare
            Native : constant Geodetic_Position_Pointers.Object_Pointer :=
              Geodetic_Position_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Geodetic_Position_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Geodetic_Position (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "PolyArea" then
         declare
            Native : constant Poly_Area_Pointers.Object_Pointer :=
              Poly_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Poly_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Poly_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Achievement" then
         declare
            Native : constant Achievement_Pointers.Object_Pointer :=
              Achievement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Achievement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Achievement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Requirement" then
         declare
            Native : constant Requirement_Pointers.Object_Pointer :=
              Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Capability" then
         declare
            Native : constant Capability_Pointers.Object_Pointer :=
              Capability_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Capability_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Capability (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Entity" then
         declare
            Native : constant Entity_Pointers.Object_Pointer :=
              Entity_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Entity_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Entity (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "CircleArea" then
         declare
            Native : constant Circle_Area_Pointers.Object_Pointer :=
              Circle_Area_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Circle_Area_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Circle_Area (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Point" then
         declare
            Native : constant Point_Pointers.Object_Pointer :=
              Point_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Point_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Point (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Contraint" then
         declare
            Native : constant Contraint_Pointers.Object_Pointer :=
              Contraint_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Contraint_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Contraint (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Ack" then
         declare
            Native : constant Ack_Pointers.Object_Pointer :=
              Ack_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Ack_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Ack (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Query" then
         declare
            Native : constant Query_Pointers.Object_Pointer :=
              Query_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Query_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Query (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectDetail" then
         declare
            Native : constant Object_Detail_Pointers.Object_Pointer :=
              Object_Detail_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Detail_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Object_Detail (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectEvidenceRequirement" then
         declare
            Native : constant Object_Evidence_Requirement_Pointers.Object_Pointer :=
              Object_Evidence_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Evidence_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Object_Evidence_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectInterestRequirement" then
         declare
            Native : constant Object_Interest_Requirement_Pointers.Object_Pointer :=
              Object_Interest_Requirement_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Interest_Requirement_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Object_Interest_Requirement (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "ObjectMatch" then
         declare
            Native : constant Object_Match_Pointers.Object_Pointer :=
              Object_Match_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid_Object_Match_C := (others => <>);
            Status : Pcl_Bindings.Pcl_Status :=
              Pcl_Bindings.PCL_ERR_INVALID;
         begin
            if Value = System.Null_Address then
               return Pcl_Bindings.PCL_ERR_INVALID;
            end if;
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               From_C (C_Value, Native.all);
            end if;
            Free_Object_Match (C_Value'Access);
            return Status;
         end;
      end if;
      if Schema_Id = "Angle" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Length" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Timestamp" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Identifier" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         declare
            Native : constant Identifier_Pointers.Object_Pointer :=
              Identifier_Pointers.To_Pointer (Value);
            C_Value : aliased Pyramid.Data_Model.Base.Cabi.Pyramid_Str_T;
            Status : Pcl_Bindings.Pcl_Status;
         begin
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, C_Value'Address);
            if Status = Pcl_Bindings.PCL_OK then
               if C_Value.Ptr = Interfaces.C.Strings.Null_Ptr then
                  Native.all := Null_Unbounded_String;
               else
                  Native.all := To_Unbounded_String
                    (Interfaces.C.Strings.Value
                       (C_Value.Ptr,
                        Interfaces.C.size_t (C_Value.Len)));
               end if;
            end if;
            if C_Value.Ptr /= Interfaces.C.Strings.Null_Ptr then
               Interfaces.C.Strings.Free (C_Value.Ptr);
            end if;
            return Status;
         end;
      end if;
      if Schema_Id = "Speed" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      if Schema_Id = "Percentage" then
         if Value = System.Null_Address then
            return Pcl_Bindings.PCL_ERR_INVALID;
         end if;
         return Codec.all.Decode.all
           (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      end if;
      return Pcl_Bindings.PCL_ERR_NOT_FOUND;
   end Try_Cabi_Registry_Decode;

   function Try_Registry_Encode
     (Content_Type : String;
      Schema_Id    : String;
      Value        : System.Address;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Schema_Id);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      if Effective = "" then
         Interfaces.C.Strings.Free (Content_C);
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            Status := Try_Cabi_Registry_Encode
              (Codec, Schema_C, Schema_Id, Value, Msg'Access);
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode;

   function Try_Registry_Decode
     (Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Schema_Id : String;
      Value     : System.Address) return Boolean
   is
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Schema_Id);
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      if Msg = null then
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Status := Try_Cabi_Registry_Decode
              (Codec, Schema_C, Schema_Id, Msg, Value);
            if Status = Pcl_Bindings.PCL_OK then
               Interfaces.C.Strings.Free (Schema_C);
               if Type_C /= Interfaces.C.Strings.Null_Ptr then
                  Interfaces.C.Strings.Free (Type_C);
               end if;
               return True;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Schema_C);
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
      end if;
      return False;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         return False;
   end Try_Registry_Decode;

   function Try_Registry_Decode_Raw
     (Content_Type : String;
      Data         : System.Address;
      Size         : Natural;
      Schema_Id    : String;
      Value        : System.Address) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Type_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Msg : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => Data,
         Size      => Interfaces.C.unsigned (Size),
         Type_Name => Type_C);
      Ok : Boolean := False;
   begin
      Ok := Try_Registry_Decode (Msg'Access, Schema_Id, Value);
      Interfaces.C.Strings.Free (Type_C);
      return Ok;
   exception
      when others =>
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         return False;
   end Try_Registry_Decode_Raw;

   function Try_Registry_Encode_Object_Detail_Array
     (Content_Type : String;
      Payload      : Object_Detail_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ObjectDetailArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Tactical.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Object_Detail_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Object_Detail_C;
                     pragma Convention (C, Object_Detail_Array_C_Array);
                     Values : Object_Detail_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Object_Detail (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Object_Detail_Array;

   function Registry_Decode_Object_Detail_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Detail_Array
   is
      Empty : Object_Detail_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ObjectDetailArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Tactical.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema ObjectDetailArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema ObjectDetailArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Object_Detail_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Object_Detail_C;
         pragma Convention (C, Object_Detail_Array_C_Array);
         Values : Object_Detail_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Object_Detail_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Object_Detail (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Object_Detail_Array;

   function Try_Registry_Encode_Object_Evidence_Requirement_Array
     (Content_Type : String;
      Payload      : Object_Evidence_Requirement_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ObjectEvidenceRequirementArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Tactical.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Object_Evidence_Requirement_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Object_Evidence_Requirement_C;
                     pragma Convention (C, Object_Evidence_Requirement_Array_C_Array);
                     Values : Object_Evidence_Requirement_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Object_Evidence_Requirement (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Object_Evidence_Requirement_Array;

   function Registry_Decode_Object_Evidence_Requirement_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Evidence_Requirement_Array
   is
      Empty : Object_Evidence_Requirement_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ObjectEvidenceRequirementArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Tactical.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema ObjectEvidenceRequirementArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema ObjectEvidenceRequirementArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Object_Evidence_Requirement_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Object_Evidence_Requirement_C;
         pragma Convention (C, Object_Evidence_Requirement_Array_C_Array);
         Values : Object_Evidence_Requirement_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Object_Evidence_Requirement_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Object_Evidence_Requirement (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Object_Evidence_Requirement_Array;

   function Try_Registry_Encode_Capability_Array
     (Content_Type : String;
      Payload      : Capability_Array;
      Wire         : out Unbounded_String) return Boolean
   is
      Effective : constant String :=
        (if Content_Type = "" then Json_Content_Type else Content_Type);
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Effective);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("CapabilityArray");
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
   begin
      Wire := Null_Unbounded_String;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C, Index);
         exit when Codec = null;
         if Codec.all.Encode /= null then
            Msg :=
              (Data      => System.Null_Address,
               Size      => 0,
               Type_Name => Interfaces.C.Strings.Null_Ptr);
            declare
               Slice : aliased Pyramid.Data_Model.Common.Cabi.Pyramid_Slice_T :=
                 (Ptr => System.Null_Address, Len => 0);
            begin
               if Payload'Length = 0 then
                  Status := Codec.all.Encode.all
                    (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
               else
                  declare
                     Count : constant Natural := Payload'Length;
                     type Capability_Array_C_Array is array (Positive range 1 .. Count)
                       of aliased Pyramid_Capability_C;
                     pragma Convention (C, Capability_Array_C_Array);
                     Values : Capability_Array_C_Array := (others => (others => <>));
                  begin
                     for Offset in 0 .. Count - 1 loop
                        To_C (Payload (Payload'First + Offset),
                              Values (Values'First + Offset));
                     end loop;
                     Slice.Ptr := Values (Values'First)'Address;
                     Slice.Len := Interfaces.C.unsigned (Count);
                     Status := Codec.all.Encode.all
                       (Codec.all.Codec_Ctx, Schema_C, Slice'Address, Msg'Access);
                     for I in Values'Range loop
                        Free_Capability (Values (I)'Access);
                     end loop;
                  end;
               end if;
            end;
            if Status = Pcl_Bindings.PCL_OK then
               if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
                  Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
               end if;
               if Codec.all.Free_Msg /= null then
                  Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
               end if;
               Interfaces.C.Strings.Free (Content_C);
               Interfaces.C.Strings.Free (Schema_C);
               return True;
            end if;
            if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
               Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode_Capability_Array;

   function Registry_Decode_Capability_Array
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capability_Array
   is
      Empty : Capability_Array (1 .. 0);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("CapabilityArray");
      Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
      Index : Interfaces.C.unsigned := 0;
      Decoded : Boolean := False;
      Slice : aliased Pyramid.Data_Model.Common.Cabi.Pyramid_Slice_T :=
        (Ptr => System.Null_Address, Len => 0);
   begin
      if Msg = null then
         raise Program_Error with "codec registry decode failed for schema CapabilityArray";
      end if;
      if Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Type_C := Interfaces.C.Strings.New_String (Json_Content_Type);
      end if;
      loop
         Codec := Pcl_Codec_Registry_Get_At
           (Pcl_Plugins.Pcl_Codec_Registry_Default,
            (if Type_C = Interfaces.C.Strings.Null_Ptr then Msg.Type_Name else Type_C),
            Index);
         exit when Codec = null;
         if Codec.all.Decode /= null then
            Slice := (Ptr => System.Null_Address, Len => 0);
            Status := Codec.all.Decode.all
              (Codec.all.Codec_Ctx, Schema_C, Msg, Slice'Address);
            if Status = Pcl_Bindings.PCL_OK then
               Decoded := True;
               exit;
            end if;
            if Slice.Ptr /= System.Null_Address then
               C_Free (Slice.Ptr);
               Slice.Ptr := System.Null_Address;
            end if;
         end if;
         exit when Index = Interfaces.C.unsigned'Last;
         Index := Index + 1;
      end loop;
      if not Decoded then
         raise Program_Error with "codec registry decode failed for schema CapabilityArray";
      end if;
      Interfaces.C.Strings.Free (Schema_C);
      Schema_C := Interfaces.C.Strings.Null_Ptr;
      if Type_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Type_C);
         Type_C := Interfaces.C.Strings.Null_Ptr;
      end if;
      if Slice.Ptr = System.Null_Address or else Slice.Len = 0 then
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
            Slice.Ptr := System.Null_Address;
         end if;
         return Empty;
      end if;
      declare
         Count : constant Natural := Natural (Slice.Len);
         type Capability_Array_C_Array is array (Positive range 1 .. Count)
           of aliased Pyramid_Capability_C;
         pragma Convention (C, Capability_Array_C_Array);
         Values : Capability_Array_C_Array;
         for Values'Address use Slice.Ptr;
         pragma Import (Ada, Values);
         Result : Capability_Array (1 .. Count);
      begin
         for I in 1 .. Count loop
            From_C (Values (I), Result (I));
            Free_Capability (Values (I)'Access);
         end loop;
         C_Free (Slice.Ptr);
         Slice.Ptr := System.Null_Address;
         return Result;
      end;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         if Slice.Ptr /= System.Null_Address then
            C_Free (Slice.Ptr);
         end if;
         raise;
   end Registry_Decode_Capability_Array;


   function Supports_Content_Type (Content_Type : String) return Boolean is
   begin
      return Registry_Has_Codec
        ((if Content_Type = "" then Json_Content_Type else Content_Type));
   end Supports_Content_Type;

   function Message_Content_Type
     (Msg : access constant Pcl_Bindings.Pcl_Msg) return String is
   begin
      if Msg = null or else Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         return Json_Content_Type;
      end if;
      return Interfaces.C.Strings.Value (Msg.Type_Name);
   end Message_Content_Type;

   function Decode_Object_Evidence
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Detail
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         declare
            Result : Object_Detail;
         begin
            return Result;
         end;
      end if;
      Require_Codec (Content_Type);  --  fail closed if no plugin

      declare
         Result : Object_Detail;
      begin
         if Try_Registry_Decode (Msg, "ObjectDetail", Result'Address) then
            return Result;
         end if;
      end;

      raise Program_Error with
        "codec registry decode failed for schema ObjectDetail";
   end Decode_Object_Evidence;

   --  -- Object_Evidence_Service ------------------------------------
   function Default_Handle_Object_Evidence_Read_Detail
     (Request : Query) return Object_Detail_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Object_Evidence_Read_Detail;

   --  -- Object_Solution_Evidence_Service ------------------------------------
   procedure Default_Handle_Object_Solution_Evidence_Create_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Object_Solution_Evidence_Create_Requirement;

   function Default_Handle_Object_Solution_Evidence_Read_Requirement
     (Request : Query) return Object_Evidence_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Evidence_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Object_Solution_Evidence_Read_Requirement;

   procedure Default_Handle_Object_Solution_Evidence_Update_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Object_Solution_Evidence_Update_Requirement;

   procedure Default_Handle_Object_Solution_Evidence_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Object_Solution_Evidence_Delete_Requirement;

   --  -- Object_Source_Capability_Service ------------------------------------
   function Default_Handle_Object_Source_Capability_Read_Capability
     (Request : Query) return Capability_Array
   is
      pragma Unreferenced (Request);
      Empty : Capability_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Object_Source_Capability_Read_Capability;

   function Service_Object_Evidence_Read_Detail
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Evidence_Read_Detail);

   function Service_Object_Solution_Evidence_Create_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Solution_Evidence_Create_Requirement);

   function Service_Object_Solution_Evidence_Read_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Solution_Evidence_Read_Requirement);

   function Service_Object_Solution_Evidence_Update_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Solution_Evidence_Update_Requirement);

   function Service_Object_Solution_Evidence_Delete_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Solution_Evidence_Delete_Requirement);

   function Service_Object_Source_Capability_Read_Capability
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Source_Capability_Read_Capability);

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json")
   is
      Handler_Ptr : constant System.Address := Handler_Address (Handlers);
   begin
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Evidence_Read_Detail);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Evidence_Read_Detail'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Solution_Evidence_Create_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Solution_Evidence_Create_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Solution_Evidence_Read_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Solution_Evidence_Read_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Solution_Evidence_Update_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Solution_Evidence_Update_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Solution_Evidence_Delete_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Solution_Evidence_Delete_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Source_Capability_Read_Capability);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Source_Capability_Read_Capability'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
   end Register_Services;

   function Service_Object_Evidence_Read_Detail
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
         Channel       => Ch_Object_Evidence_Read_Detail,
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
   end Service_Object_Evidence_Read_Detail;

   function Service_Object_Solution_Evidence_Create_Requirement
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
         Channel       => Ch_Object_Solution_Evidence_Create_Requirement,
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
   end Service_Object_Solution_Evidence_Create_Requirement;

   function Service_Object_Solution_Evidence_Read_Requirement
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
         Channel       => Ch_Object_Solution_Evidence_Read_Requirement,
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
   end Service_Object_Solution_Evidence_Read_Requirement;

   function Service_Object_Solution_Evidence_Update_Requirement
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
         Channel       => Ch_Object_Solution_Evidence_Update_Requirement,
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
   end Service_Object_Solution_Evidence_Update_Requirement;

   function Service_Object_Solution_Evidence_Delete_Requirement
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
         Channel       => Ch_Object_Solution_Evidence_Delete_Requirement,
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
   end Service_Object_Solution_Evidence_Delete_Requirement;

   function Service_Object_Source_Capability_Read_Capability
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
         Channel       => Ch_Object_Source_Capability_Read_Capability,
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
   end Service_Object_Source_Capability_Read_Capability;

   --  -- PCL binding implementations -------------------------------

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : Object_Detail;
      Content_Type : String := "application/json")
   is
      function Build_Wire_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         Require_Codec (Content_Type);  --  fail closed if no plugin
         if Try_Registry_Encode
           (Content_Type, "ObjectDetail", Payload'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;
         raise Program_Error with
           "codec registry encode failed for schema ObjectDetail";
      end Build_Wire_Payload;
      Wire_Payload : constant String := Build_Wire_Payload;
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
         when Ch_Object_Evidence_Read_Detail =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Object_Detail_Array :=
                 (if Handlers /= null and then Handlers.On_Object_Evidence_Read_Detail /= null
                  then Handlers.On_Object_Evidence_Read_Detail.all (Req)
                  else Default_Handle_Object_Evidence_Read_Detail (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Object_Detail_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema ObjectDetailArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Object_Solution_Evidence_Create_Requirement =>
            declare
               function Decode_Request return Object_Evidence_Requirement is
                  Result : Object_Evidence_Requirement;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "ObjectEvidenceRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema ObjectEvidenceRequirement";
               end Decode_Request;
               Req : constant Object_Evidence_Requirement := Decode_Request;
               Rsp : Identifier;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Object_Solution_Evidence_Create_Requirement /= null then
                  Handlers.On_Object_Solution_Evidence_Create_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Object_Solution_Evidence_Create_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Identifier", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Identifier";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Object_Solution_Evidence_Read_Requirement =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Object_Evidence_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Object_Solution_Evidence_Read_Requirement /= null
                  then Handlers.On_Object_Solution_Evidence_Read_Requirement.all (Req)
                  else Default_Handle_Object_Solution_Evidence_Read_Requirement (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Object_Evidence_Requirement_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema ObjectEvidenceRequirementArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Object_Solution_Evidence_Update_Requirement =>
            declare
               function Decode_Request return Object_Evidence_Requirement is
                  Result : Object_Evidence_Requirement;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "ObjectEvidenceRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema ObjectEvidenceRequirement";
               end Decode_Request;
               Req : constant Object_Evidence_Requirement := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Object_Solution_Evidence_Update_Requirement /= null then
                  Handlers.On_Object_Solution_Evidence_Update_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Object_Solution_Evidence_Update_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Object_Solution_Evidence_Delete_Requirement =>
            declare
               function Decode_Request return Identifier is
                  Result : Identifier;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Identifier", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Identifier";
               end Decode_Request;
               Req : constant Identifier := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Object_Solution_Evidence_Delete_Requirement /= null then
                  Handlers.On_Object_Solution_Evidence_Delete_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Object_Solution_Evidence_Delete_Requirement (Req, Rsp);
               end if;
               Require_Codec (Content_Type);  --  fail closed if no plugin
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  raise Program_Error with
                    "codec registry encode failed for schema Ack";
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Object_Source_Capability_Read_Capability =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  raise Program_Error with
                    "codec registry decode failed for schema Query";
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Capability_Array :=
                 (if Handlers /= null and then Handlers.On_Object_Source_Capability_Read_Capability /= null
                  then Handlers.On_Object_Source_Capability_Read_Capability.all (Req)
                  else Default_Handle_Object_Source_Capability_Read_Capability (Req));
            begin
               declare
                  Wire_Response : Unbounded_String := Null_Unbounded_String;
               begin
                  Require_Codec (Content_Type);  --  fail closed if no plugin
                  if not Try_Registry_Encode_Capability_Array
                    (Content_Type, Rsp, Wire_Response)
                  then
                     raise Program_Error with
                       "codec registry encode failed for schema CapabilityArray";
                  end if;
                  Copy_To_Buf
                    (To_String (Wire_Response),
                    Response_Buf, Response_Size);
               end;
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Consumed;
