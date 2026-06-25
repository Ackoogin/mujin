--  Auto-generated Ada C-ABI marshalling body
--  Package body: Pyramid.Data_Model.Common.Cabi

with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;

package body Pyramid.Data_Model.Common.Cabi is
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
     (In_Value  : Geodetic_Position;
      Out_Value : out Pyramid_Geodetic_Position_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Latitude := Interfaces.C.double (In_Value.Latitude);
      Out_Value.Longitude := Interfaces.C.double (In_Value.Longitude);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Geodetic_Position_C;
      Out_Value : out Geodetic_Position)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Latitude := Long_Float (In_Value.Latitude);
      Out_Value.Longitude := Long_Float (In_Value.Longitude);
   end From_C;

   procedure To_C
     (In_Value  : Poly_Area;
      Out_Value : out Pyramid_Poly_Area_C)
   is
   begin
      Out_Value := (others => <>);
      declare
         Count : constant Natural :=
           (if In_Value.Points = null then 0
            else In_Value.Points.all'Length);
      begin
         if Count > 0 then
            Out_Value.Points.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Geodetic_Position_C'Size));
            Out_Value.Points.Len := Interfaces.C.unsigned (Count);
            declare
               type Points_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Geodetic_Position_C;
               pragma Convention (C, Points_Array_C_Array);
               Arr : Points_Array_C_Array;
               for Arr'Address use Out_Value.Points.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Points (I), Arr (I));
               end loop;
            end;
         end if;
      end;
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Poly_Area_C;
      Out_Value : out Poly_Area)
   is
   begin
      Out_Value := (others => <>);
      if In_Value.Points.Ptr /= System.Null_Address
        and then In_Value.Points.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Points.Len);
            type Points_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Geodetic_Position_C;
            pragma Convention (C, Points_Array_C_Array);
            Arr : Points_Array_C_Array;
            for Arr'Address use In_Value.Points.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Points := new Points_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Points (I));
            end loop;
         end;
      else
         Out_Value.Points := null;
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Achievement;
      Out_Value : out Pyramid_Achievement_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Out_Value.Status := Interfaces.C.int
        (Progress'Pos (In_Value.Status));
      Out_Value.Has_Quality := 1;
      Out_Value.Quality := Interfaces.C.double (In_Value.Quality);
      Out_Value.Achieveability := Interfaces.C.int
        (Feasibility'Pos (In_Value.Achieveability));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Achievement_C;
      Out_Value : out Achievement)
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
      Out_Value.Status := Progress'Val
        (Integer (In_Value.Status));
      if In_Value.Has_Quality /= 0 then
         Out_Value.Quality := Long_Float (In_Value.Quality);
      end if;
      Out_Value.Achieveability := Feasibility'Val
        (Integer (In_Value.Achieveability));
   end From_C;

   procedure To_C
     (In_Value  : Requirement;
      Out_Value : out Pyramid_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      To_C (In_Value.Status, Out_Value.Status);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Requirement_C;
      Out_Value : out Requirement)
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
      From_C (In_Value.Status, Out_Value.Status);
   end From_C;

   procedure To_C
     (In_Value  : Capability;
      Out_Value : out Pyramid_Capability_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Out_Value.Availability :=
        (if In_Value.Availability then 1 else 0);
      Dup_Str (Out_Value.Name, To_String (In_Value.Name));
      declare
         Count : constant Natural :=
           (if In_Value.Contraint = null then 0
            else In_Value.Contraint.all'Length);
      begin
         if Count > 0 then
            Out_Value.Contraint.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Contraint_C'Size));
            Out_Value.Contraint.Len := Interfaces.C.unsigned (Count);
            declare
               type Contraint_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Contraint_C;
               pragma Convention (C, Contraint_Array_C_Array);
               Arr : Contraint_Array_C_Array;
               for Arr'Address use Out_Value.Contraint.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Contraint (I), Arr (I));
               end loop;
            end;
         end if;
      end;
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Capability_C;
      Out_Value : out Capability)
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
      Out_Value.Availability := In_Value.Availability /= 0;
      Out_Value.Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Name));
      if In_Value.Contraint.Ptr /= System.Null_Address
        and then In_Value.Contraint.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Contraint.Len);
            type Contraint_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Contraint_C;
            pragma Convention (C, Contraint_Array_C_Array);
            Arr : Contraint_Array_C_Array;
            for Arr'Address use In_Value.Contraint.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Contraint := new Contraint_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Contraint (I));
            end loop;
         end;
      else
         Out_Value.Contraint := null;
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Entity;
      Out_Value : out Pyramid_Entity_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Entity_C;
      Out_Value : out Entity)
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
   end From_C;

   procedure To_C
     (In_Value  : Circle_Area;
      Out_Value : out Pyramid_Circle_Area_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Position, Out_Value.Position);
      Out_Value.Radius := Interfaces.C.double (In_Value.Radius);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Circle_Area_C;
      Out_Value : out Circle_Area)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Position, Out_Value.Position);
      Out_Value.Radius := Long_Float (In_Value.Radius);
   end From_C;

   procedure To_C
     (In_Value  : Point;
      Out_Value : out Pyramid_Point_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Position, Out_Value.Position);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Point_C;
      Out_Value : out Point)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Position, Out_Value.Position);
   end From_C;

   procedure To_C
     (In_Value  : Contraint;
      Out_Value : out Pyramid_Contraint_C)
   is
   begin
      Out_Value := (others => <>);
      Dup_Str (Out_Value.Name, To_String (In_Value.Name));
      Out_Value.Value := Interfaces.C.int (In_Value.Value);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Contraint_C;
      Out_Value : out Contraint)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Name));
      Out_Value.Value := Integer (In_Value.Value);
   end From_C;

   procedure To_C
     (In_Value  : Ack;
      Out_Value : out Pyramid_Ack_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Success :=
        (if In_Value.Success then 1 else 0);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Ack_C;
      Out_Value : out Ack)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Success := In_Value.Success /= 0;
   end From_C;

   procedure To_C
     (In_Value  : Query;
      Out_Value : out Pyramid_Query_C)
   is
   begin
      Out_Value := (others => <>);
      declare
         Count : constant Natural :=
           (if In_Value.Id = null then 0
            else In_Value.Id.all'Length);
      begin
         if Count > 0 then
            Out_Value.Id.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Str_T'Size));
            Out_Value.Id.Len := Interfaces.C.unsigned (Count);
            declare
               type Id_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Str_T;
               pragma Convention (C, Id_Array_C_Array);
               Arr : Id_Array_C_Array;
               for Arr'Address use Out_Value.Id.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  Dup_Str (Arr (I), To_String (In_Value.Id (I)));
               end loop;
            end;
         end if;
      end;
      Out_Value.Has_One_Shot :=
        (if In_Value.Has_One_Shot then 1 else 0);
      Out_Value.One_Shot :=
        (if In_Value.One_Shot then 1 else 0);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Query_C;
      Out_Value : out Query)
   is
   begin
      Out_Value := (others => <>);
      if In_Value.Id.Ptr /= System.Null_Address
        and then In_Value.Id.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Id.Len);
            type Id_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Str_T;
            pragma Convention (C, Id_Array_C_Array);
            Arr : Id_Array_C_Array;
            for Arr'Address use In_Value.Id.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Id := new Id_Array (1 .. Count);
            for I in 1 .. Count loop
               Out_Value.Id (I) := To_Unbounded_String (To_Ada_String (Arr (I)));
            end loop;
         end;
      else
         Out_Value.Id := null;
      end if;
      Out_Value.Has_One_Shot := In_Value.Has_One_Shot /= 0;
      if Out_Value.Has_One_Shot then
         Out_Value.One_Shot := In_Value.One_Shot /= 0;
      end if;
   end From_C;

end Pyramid.Data_Model.Common.Cabi;
