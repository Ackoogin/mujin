--  Auto-generated Ada C-ABI marshalling body
--  Package body: Pyramid.Data_Model.Sensors.Cabi

with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;

package body Pyramid.Data_Model.Sensors.Cabi is
   use type Interfaces.C.unsigned;
   use type Interfaces.C.unsigned_char;
   use type Interfaces.C.Strings.chars_ptr;
   use type System.Address;

   function Malloc (Size : Interfaces.C.size_t)
     return System.Address;
   pragma Import (C, Malloc, "malloc");

   function To_Chars_Ptr is new Ada.Unchecked_Conversion
     (System.Address, Interfaces.C.Strings.chars_ptr);

   function To_Address is new Ada.Unchecked_Conversion
     (Interfaces.C.Strings.chars_ptr, System.Address);

   function Bytes_For
     (Count : Natural;
      Element_Bits : Natural) return Interfaces.C.size_t
   is
   begin
      return Interfaces.C.size_t
        (Count * (Element_Bits / System.Storage_Unit));
   end Bytes_For;

   function To_Ada_String (S : Pyramid_Str_T) return String is
   begin
      if S.Ptr = Interfaces.C.Strings.Null_Ptr or else S.Len = 0 then
         return "";
      end if;
      return Interfaces.C.Strings.Value
        (S.Ptr, Interfaces.C.size_t (S.Len));
   end To_Ada_String;

   procedure Dup_Str (Out_Value : out Pyramid_Str_T; S : String) is
   begin
      Out_Value := (Ptr => Interfaces.C.Strings.Null_Ptr, Len => 0);
      if S'Length = 0 then
         return;
      end if;
      Out_Value.Len := Interfaces.C.unsigned (S'Length);
      Out_Value.Ptr := To_Chars_Ptr
        (Malloc (Interfaces.C.size_t (Out_Value.Len)));
      declare
         type Char_Array is array (Positive range 1 .. S'Length)
           of Interfaces.C.char;
         pragma Convention (C, Char_Array);
         Chars : Char_Array;
         for Chars'Address use To_Address (Out_Value.Ptr);
         pragma Import (Ada, Chars);
      begin
         for I in S'Range loop
            Chars (I - S'First + 1) :=
              Interfaces.C.char'Val (Character'Pos (S (I)));
         end loop;
      end;
   end Dup_Str;

   procedure To_C
     (In_Value  : Interpretation_Requirement;
      Out_Value : out Pyramid_Interpretation_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      Out_Value.Policy := Interfaces.C.int
        (Interpretation_Policy'Pos (In_Value.Policy));
      Out_Value.Type_Field := Interfaces.C.int
        (Interpretation_Type'Pos (In_Value.Type_Field));
      Out_Value.Has_Poly_Area :=
        (if In_Value.Has_Val_Poly_Area then 1 else 0);
      To_C (In_Value.Val_Poly_Area, Out_Value.Poly_Area);
      Out_Value.Has_Circle_Area :=
        (if In_Value.Has_Val_Circle_Area then 1 else 0);
      To_C (In_Value.Val_Circle_Area, Out_Value.Circle_Area);
      Out_Value.Has_Point :=
        (if In_Value.Has_Val_Point then 1 else 0);
      To_C (In_Value.Val_Point, Out_Value.Point);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Interpretation_Requirement_C;
      Out_Value : out Interpretation_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      Out_Value.Policy := Interpretation_Policy'Val
        (Integer (In_Value.Policy));
      Out_Value.Type_Field := Interpretation_Type'Val
        (Integer (In_Value.Type_Field));
      Out_Value.Has_Val_Poly_Area := In_Value.Has_Poly_Area /= 0;
      if Out_Value.Has_Val_Poly_Area then
         From_C (In_Value.Poly_Area, Out_Value.Val_Poly_Area);
      end if;
      Out_Value.Has_Val_Circle_Area := In_Value.Has_Circle_Area /= 0;
      if Out_Value.Has_Val_Circle_Area then
         From_C (In_Value.Circle_Area, Out_Value.Val_Circle_Area);
      end if;
      Out_Value.Has_Val_Point := In_Value.Has_Point /= 0;
      if Out_Value.Has_Val_Point then
         From_C (In_Value.Point, Out_Value.Val_Point);
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Manual_Track_Requirement;
      Out_Value : out Pyramid_Manual_Track_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      To_C (In_Value.Position, Out_Value.Position);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Manual_Track_Requirement_C;
      Out_Value : out Manual_Track_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      From_C (In_Value.Position, Out_Value.Position);
   end From_C;

   procedure To_C
     (In_Value  : Ati_Requirement;
      Out_Value : out Pyramid_Ati_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      To_C (In_Value.Auto_Zone, Out_Value.Auto_Zone);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Ati_Requirement_C;
      Out_Value : out Ati_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      From_C (In_Value.Auto_Zone, Out_Value.Auto_Zone);
   end From_C;

   procedure To_C
     (In_Value  : Track_Provision_Requirement;
      Out_Value : out Pyramid_Track_Provision_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      Out_Value.Has_Poly_Area :=
        (if In_Value.Has_Val_Poly_Area then 1 else 0);
      To_C (In_Value.Val_Poly_Area, Out_Value.Poly_Area);
      Out_Value.Has_Circle_Area :=
        (if In_Value.Has_Val_Circle_Area then 1 else 0);
      To_C (In_Value.Val_Circle_Area, Out_Value.Circle_Area);
      Out_Value.Has_Point :=
        (if In_Value.Has_Val_Point then 1 else 0);
      To_C (In_Value.Val_Point, Out_Value.Point);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Track_Provision_Requirement_C;
      Out_Value : out Track_Provision_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      Out_Value.Has_Val_Poly_Area := In_Value.Has_Poly_Area /= 0;
      if Out_Value.Has_Val_Poly_Area then
         From_C (In_Value.Poly_Area, Out_Value.Val_Poly_Area);
      end if;
      Out_Value.Has_Val_Circle_Area := In_Value.Has_Circle_Area /= 0;
      if Out_Value.Has_Val_Circle_Area then
         From_C (In_Value.Circle_Area, Out_Value.Val_Circle_Area);
      end if;
      Out_Value.Has_Val_Point := In_Value.Has_Point /= 0;
      if Out_Value.Has_Val_Point then
         From_C (In_Value.Point, Out_Value.Val_Point);
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Object_Evidence_Provision_Requirement;
      Out_Value : out Pyramid_Object_Evidence_Provision_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      Out_Value.Has_Poly_Area :=
        (if In_Value.Has_Val_Poly_Area then 1 else 0);
      To_C (In_Value.Val_Poly_Area, Out_Value.Poly_Area);
      Out_Value.Has_Circle_Area :=
        (if In_Value.Has_Val_Circle_Area then 1 else 0);
      To_C (In_Value.Val_Circle_Area, Out_Value.Circle_Area);
      Out_Value.Has_Point :=
        (if In_Value.Has_Val_Point then 1 else 0);
      To_C (In_Value.Val_Point, Out_Value.Point);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Object_Evidence_Provision_Requirement_C;
      Out_Value : out Object_Evidence_Provision_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      Out_Value.Has_Val_Poly_Area := In_Value.Has_Poly_Area /= 0;
      if Out_Value.Has_Val_Poly_Area then
         From_C (In_Value.Poly_Area, Out_Value.Val_Poly_Area);
      end if;
      Out_Value.Has_Val_Circle_Area := In_Value.Has_Circle_Area /= 0;
      if Out_Value.Has_Val_Circle_Area then
         From_C (In_Value.Circle_Area, Out_Value.Val_Circle_Area);
      end if;
      Out_Value.Has_Val_Point := In_Value.Has_Point /= 0;
      if Out_Value.Has_Val_Point then
         From_C (In_Value.Point, Out_Value.Val_Point);
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Object_Aquisition_Requirement;
      Out_Value : out Pyramid_Object_Aquisition_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      Out_Value.Has_Poly_Area :=
        (if In_Value.Has_Val_Poly_Area then 1 else 0);
      To_C (In_Value.Val_Poly_Area, Out_Value.Poly_Area);
      Out_Value.Has_Circle_Area :=
        (if In_Value.Has_Val_Circle_Area then 1 else 0);
      To_C (In_Value.Val_Circle_Area, Out_Value.Circle_Area);
      Out_Value.Has_Point :=
        (if In_Value.Has_Val_Point then 1 else 0);
      To_C (In_Value.Val_Point, Out_Value.Point);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Object_Aquisition_Requirement_C;
      Out_Value : out Object_Aquisition_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      Out_Value.Has_Val_Poly_Area := In_Value.Has_Poly_Area /= 0;
      if Out_Value.Has_Val_Poly_Area then
         From_C (In_Value.Poly_Area, Out_Value.Val_Poly_Area);
      end if;
      Out_Value.Has_Val_Circle_Area := In_Value.Has_Circle_Area /= 0;
      if Out_Value.Has_Val_Circle_Area then
         From_C (In_Value.Circle_Area, Out_Value.Val_Circle_Area);
      end if;
      Out_Value.Has_Val_Point := In_Value.Has_Point /= 0;
      if Out_Value.Has_Val_Point then
         From_C (In_Value.Point, Out_Value.Val_Point);
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Sensor_Object;
      Out_Value : out Pyramid_Sensor_Object_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      To_C (In_Value.Position, Out_Value.Position);
      Out_Value.Creation_Time := Interfaces.C.double (In_Value.Creation_Time);
      Out_Value.Has_Quality := 1;
      Out_Value.Quality := Interfaces.C.double (In_Value.Quality);
      Out_Value.Has_Course := 1;
      Out_Value.Course := Interfaces.C.double (In_Value.Course);
      Out_Value.Has_Speed := 1;
      Out_Value.Speed := Interfaces.C.double (In_Value.Speed);
      Out_Value.Has_Length := 1;
      Out_Value.Length := Interfaces.C.double (In_Value.Length);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Sensor_Object_C;
      Out_Value : out Sensor_Object)
   is
   begin
      Out_Value := (others => <>);
      if In_Value.Has_Update_Time /= 0 then
         Out_Value.Update_Time := Long_Float (In_Value.Update_Time);
      end if;
      Out_Value.Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Id));
      Out_Value.Source :=
        To_Unbounded_String (To_Ada_String (In_Value.Source));
      From_C (In_Value.Position, Out_Value.Position);
      Out_Value.Creation_Time := Long_Float (In_Value.Creation_Time);
      if In_Value.Has_Quality /= 0 then
         Out_Value.Quality := Long_Float (In_Value.Quality);
      end if;
      if In_Value.Has_Course /= 0 then
         Out_Value.Course := Long_Float (In_Value.Course);
      end if;
      if In_Value.Has_Speed /= 0 then
         Out_Value.Speed := Long_Float (In_Value.Speed);
      end if;
      if In_Value.Has_Length /= 0 then
         Out_Value.Length := Long_Float (In_Value.Length);
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Radar_Mode_Change_Requirement;
      Out_Value : out Pyramid_Radar_Mode_Change_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      Out_Value.Mode := Interfaces.C.int
        (Radar_Operational_Mode'Pos (In_Value.Mode));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Radar_Mode_Change_Requirement_C;
      Out_Value : out Radar_Mode_Change_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      Out_Value.Mode := Radar_Operational_Mode'Val
        (Integer (In_Value.Mode));
   end From_C;

end Pyramid.Data_Model.Sensors.Cabi;
