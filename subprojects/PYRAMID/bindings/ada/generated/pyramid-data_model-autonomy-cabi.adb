--  Auto-generated Ada C-ABI marshalling body
--  Package body: Pyramid.Data_Model.Autonomy.Cabi

with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;

package body Pyramid.Data_Model.Autonomy.Cabi is
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
     (In_Value  : Requirement_Reference;
      Out_Value : out Pyramid_Requirement_Reference_C)
   is
   begin
      Out_Value := (others => <>);
      Dup_Str (Out_Value.Requirement_Id, To_String (In_Value.Requirement_Id));
      Dup_Str (Out_Value.Component_Name, To_String (In_Value.Component_Name));
      Dup_Str (Out_Value.Service_Name, To_String (In_Value.Service_Name));
      Dup_Str (Out_Value.Type_Name, To_String (In_Value.Type_Name));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Requirement_Reference_C;
      Out_Value : out Requirement_Reference)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Requirement_Id));
      Out_Value.Component_Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Component_Name));
      Out_Value.Service_Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Service_Name));
      Out_Value.Type_Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Type_Name));
   end From_C;

   procedure To_C
     (In_Value  : Agent_State;
      Out_Value : out Pyramid_Agent_State_C)
   is
   begin
      Out_Value := (others => <>);
      Dup_Str (Out_Value.Agent_Id, To_String (In_Value.Agent_Id));
      Dup_Str (Out_Value.Agent_Type, To_String (In_Value.Agent_Type));
      Out_Value.Available :=
        (if In_Value.Available then 1 else 0);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Agent_State_C;
      Out_Value : out Agent_State)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Agent_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Agent_Id));
      Out_Value.Agent_Type :=
        To_Unbounded_String (To_Ada_String (In_Value.Agent_Type));
      Out_Value.Available := In_Value.Available /= 0;
   end From_C;

   procedure To_C
     (In_Value  : Planning_Policy;
      Out_Value : out Pyramid_Planning_Policy_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Max_Replans := Interfaces.C.unsigned (In_Value.Max_Replans);
      Out_Value.Enable_Replanning :=
        (if In_Value.Enable_Replanning then 1 else 0);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Planning_Policy_C;
      Out_Value : out Planning_Policy)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Max_Replans := Natural (In_Value.Max_Replans);
      Out_Value.Enable_Replanning := In_Value.Enable_Replanning /= 0;
   end From_C;

   procedure To_C
     (In_Value  : Planning_Goal;
      Out_Value : out Pyramid_Planning_Goal_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Dup_Str (Out_Value.Name, To_String (In_Value.Name));
      Out_Value.Has_Requirement :=
        (if In_Value.Has_Requirement then 1 else 0);
      To_C (In_Value.Requirement, Out_Value.Requirement);
      Out_Value.Has_Expression :=
        (if In_Value.Has_Expression then 1 else 0);
      Dup_Str (Out_Value.Expression, To_String (In_Value.Expression));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Planning_Goal_C;
      Out_Value : out Planning_Goal)
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
      Out_Value.Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Name));
      Out_Value.Has_Requirement := In_Value.Has_Requirement /= 0;
      if Out_Value.Has_Requirement then
         From_C (In_Value.Requirement, Out_Value.Requirement);
      end if;
      Out_Value.Has_Expression := In_Value.Has_Expression /= 0;
      if Out_Value.Has_Expression then
         Out_Value.Expression :=
           To_Unbounded_String (To_Ada_String (In_Value.Expression));
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Execution_Policy;
      Out_Value : out Pyramid_Execution_Policy_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Max_Replans := Interfaces.C.unsigned (In_Value.Max_Replans);
      Out_Value.Enable_Replanning :=
        (if In_Value.Enable_Replanning then 1 else 0);
      Out_Value.Max_Concurrent_Placements := Interfaces.C.unsigned (In_Value.Max_Concurrent_Placements);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Execution_Policy_C;
      Out_Value : out Execution_Policy)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Max_Replans := Natural (In_Value.Max_Replans);
      Out_Value.Enable_Replanning := In_Value.Enable_Replanning /= 0;
      Out_Value.Max_Concurrent_Placements := Natural (In_Value.Max_Concurrent_Placements);
   end From_C;

   procedure To_C
     (In_Value  : Planning_Requirement;
      Out_Value : out Pyramid_Planning_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      declare
         Count : constant Natural :=
           (if In_Value.Upstream_Requirement = null then 0
            else In_Value.Upstream_Requirement.all'Length);
      begin
         if Count > 0 then
            Out_Value.Upstream_Requirement.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Requirement_Reference_C'Object_Size));
            Out_Value.Upstream_Requirement.Len := Interfaces.C.unsigned (Count);
            declare
               type Upstream_Requirement_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Requirement_Reference_C;
               pragma Convention (C, Upstream_Requirement_Array_C_Array);
               Arr : Upstream_Requirement_Array_C_Array;
               for Arr'Address use Out_Value.Upstream_Requirement.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Upstream_Requirement (I), Arr (I));
               end loop;
            end;
         end if;
      end;
      declare
         Count : constant Natural :=
           (if In_Value.Goal = null then 0
            else In_Value.Goal.all'Length);
      begin
         if Count > 0 then
            Out_Value.Goal.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Planning_Goal_C'Object_Size));
            Out_Value.Goal.Len := Interfaces.C.unsigned (Count);
            declare
               type Goal_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Planning_Goal_C;
               pragma Convention (C, Goal_Array_C_Array);
               Arr : Goal_Array_C_Array;
               for Arr'Address use Out_Value.Goal.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Goal (I), Arr (I));
               end loop;
            end;
         end if;
      end;
      To_C (In_Value.Policy, Out_Value.Policy);
      declare
         Count : constant Natural :=
           (if In_Value.Available_Agents = null then 0
            else In_Value.Available_Agents.all'Length);
      begin
         if Count > 0 then
            Out_Value.Available_Agents.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Agent_State_C'Object_Size));
            Out_Value.Available_Agents.Len := Interfaces.C.unsigned (Count);
            declare
               type Available_Agents_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Agent_State_C;
               pragma Convention (C, Available_Agents_Array_C_Array);
               Arr : Available_Agents_Array_C_Array;
               for Arr'Address use Out_Value.Available_Agents.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Available_Agents (I), Arr (I));
               end loop;
            end;
         end if;
      end;
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Planning_Requirement_C;
      Out_Value : out Planning_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      if In_Value.Upstream_Requirement.Ptr /= System.Null_Address
        and then In_Value.Upstream_Requirement.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Upstream_Requirement.Len);
            type Upstream_Requirement_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Requirement_Reference_C;
            pragma Convention (C, Upstream_Requirement_Array_C_Array);
            Arr : Upstream_Requirement_Array_C_Array;
            for Arr'Address use In_Value.Upstream_Requirement.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Upstream_Requirement := new Upstream_Requirement_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Upstream_Requirement (I));
            end loop;
         end;
      else
         Out_Value.Upstream_Requirement := null;
      end if;
      if In_Value.Goal.Ptr /= System.Null_Address
        and then In_Value.Goal.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Goal.Len);
            type Goal_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Planning_Goal_C;
            pragma Convention (C, Goal_Array_C_Array);
            Arr : Goal_Array_C_Array;
            for Arr'Address use In_Value.Goal.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Goal := new Goal_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Goal (I));
            end loop;
         end;
      else
         Out_Value.Goal := null;
      end if;
      From_C (In_Value.Policy, Out_Value.Policy);
      if In_Value.Available_Agents.Ptr /= System.Null_Address
        and then In_Value.Available_Agents.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Available_Agents.Len);
            type Available_Agents_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Agent_State_C;
            pragma Convention (C, Available_Agents_Array_C_Array);
            Arr : Available_Agents_Array_C_Array;
            for Arr'Address use In_Value.Available_Agents.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Available_Agents := new Available_Agents_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Available_Agents (I));
            end loop;
         end;
      else
         Out_Value.Available_Agents := null;
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Execution_Requirement;
      Out_Value : out Pyramid_Execution_Requirement_C)
   is
   begin
      Out_Value := (others => <>);
      To_C (In_Value.Base, Out_Value.Base);
      To_C (In_Value.Status, Out_Value.Status);
      declare
         Count : constant Natural :=
           (if In_Value.Upstream_Requirement = null then 0
            else In_Value.Upstream_Requirement.all'Length);
      begin
         if Count > 0 then
            Out_Value.Upstream_Requirement.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Requirement_Reference_C'Object_Size));
            Out_Value.Upstream_Requirement.Len := Interfaces.C.unsigned (Count);
            declare
               type Upstream_Requirement_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Requirement_Reference_C;
               pragma Convention (C, Upstream_Requirement_Array_C_Array);
               Arr : Upstream_Requirement_Array_C_Array;
               for Arr'Address use Out_Value.Upstream_Requirement.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Upstream_Requirement (I), Arr (I));
               end loop;
            end;
         end if;
      end;
      Dup_Str (Out_Value.Plan_Id, To_String (In_Value.Plan_Id));
      To_C (In_Value.Policy, Out_Value.Policy);
      declare
         Count : constant Natural :=
           (if In_Value.Available_Agents = null then 0
            else In_Value.Available_Agents.all'Length);
      begin
         if Count > 0 then
            Out_Value.Available_Agents.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Agent_State_C'Object_Size));
            Out_Value.Available_Agents.Len := Interfaces.C.unsigned (Count);
            declare
               type Available_Agents_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Agent_State_C;
               pragma Convention (C, Available_Agents_Array_C_Array);
               Arr : Available_Agents_Array_C_Array;
               for Arr'Address use Out_Value.Available_Agents.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Available_Agents (I), Arr (I));
               end loop;
            end;
         end if;
      end;
      Dup_Str (Out_Value.Planning_Requirement_Id, To_String (In_Value.Planning_Requirement_Id));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Execution_Requirement_C;
      Out_Value : out Execution_Requirement)
   is
   begin
      Out_Value := (others => <>);
      From_C (In_Value.Base, Out_Value.Base);
      From_C (In_Value.Status, Out_Value.Status);
      if In_Value.Upstream_Requirement.Ptr /= System.Null_Address
        and then In_Value.Upstream_Requirement.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Upstream_Requirement.Len);
            type Upstream_Requirement_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Requirement_Reference_C;
            pragma Convention (C, Upstream_Requirement_Array_C_Array);
            Arr : Upstream_Requirement_Array_C_Array;
            for Arr'Address use In_Value.Upstream_Requirement.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Upstream_Requirement := new Upstream_Requirement_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Upstream_Requirement (I));
            end loop;
         end;
      else
         Out_Value.Upstream_Requirement := null;
      end if;
      Out_Value.Plan_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Plan_Id));
      From_C (In_Value.Policy, Out_Value.Policy);
      if In_Value.Available_Agents.Ptr /= System.Null_Address
        and then In_Value.Available_Agents.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Available_Agents.Len);
            type Available_Agents_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Agent_State_C;
            pragma Convention (C, Available_Agents_Array_C_Array);
            Arr : Available_Agents_Array_C_Array;
            for Arr'Address use In_Value.Available_Agents.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Available_Agents := new Available_Agents_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Available_Agents (I));
            end loop;
         end;
      else
         Out_Value.Available_Agents := null;
      end if;
      Out_Value.Planning_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Planning_Requirement_Id));
   end From_C;

   procedure To_C
     (In_Value  : World_Fact_Update;
      Out_Value : out Pyramid_World_Fact_Update_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Entity_Source, To_String (In_Value.Entity_Source));
      Dup_Str (Out_Value.Key, To_String (In_Value.Key));
      Out_Value.Value :=
        (if In_Value.Value then 1 else 0);
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Out_Value.Authority := Interfaces.C.int
        (Fact_Authority_Level'Pos (In_Value.Authority));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_World_Fact_Update_C;
      Out_Value : out World_Fact_Update)
   is
   begin
      Out_Value := (others => <>);
      if In_Value.Has_Update_Time /= 0 then
         Out_Value.Update_Time := Long_Float (In_Value.Update_Time);
      end if;
      Out_Value.Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Id));
      Out_Value.Entity_Source :=
        To_Unbounded_String (To_Ada_String (In_Value.Entity_Source));
      Out_Value.Key :=
        To_Unbounded_String (To_Ada_String (In_Value.Key));
      Out_Value.Value := In_Value.Value /= 0;
      Out_Value.Source :=
        To_Unbounded_String (To_Ada_String (In_Value.Source));
      Out_Value.Authority := Fact_Authority_Level'Val
        (Integer (In_Value.Authority));
   end From_C;

   procedure To_C
     (In_Value  : State_Update;
      Out_Value : out Pyramid_State_Update_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      declare
         Count : constant Natural :=
           (if In_Value.Fact_Update = null then 0
            else In_Value.Fact_Update.all'Length);
      begin
         if Count > 0 then
            Out_Value.Fact_Update.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_World_Fact_Update_C'Object_Size));
            Out_Value.Fact_Update.Len := Interfaces.C.unsigned (Count);
            declare
               type Fact_Update_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_World_Fact_Update_C;
               pragma Convention (C, Fact_Update_Array_C_Array);
               Arr : Fact_Update_Array_C_Array;
               for Arr'Address use Out_Value.Fact_Update.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Fact_Update (I), Arr (I));
               end loop;
            end;
         end if;
      end;
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_State_Update_C;
      Out_Value : out State_Update)
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
      if In_Value.Fact_Update.Ptr /= System.Null_Address
        and then In_Value.Fact_Update.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Fact_Update.Len);
            type Fact_Update_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_World_Fact_Update_C;
            pragma Convention (C, Fact_Update_Array_C_Array);
            Arr : Fact_Update_Array_C_Array;
            for Arr'Address use In_Value.Fact_Update.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Fact_Update := new Fact_Update_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Fact_Update (I));
            end loop;
         end;
      else
         Out_Value.Fact_Update := null;
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Capabilities;
      Out_Value : out Pyramid_Capabilities_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Dup_Str (Out_Value.Backend_Id, To_String (In_Value.Backend_Id));
      Out_Value.Supports_Planning_Requirements :=
        (if In_Value.Supports_Planning_Requirements then 1 else 0);
      Out_Value.Supports_Execution_Requirements :=
        (if In_Value.Supports_Execution_Requirements then 1 else 0);
      Out_Value.Supports_Approved_Plan_Execution :=
        (if In_Value.Supports_Approved_Plan_Execution then 1 else 0);
      Out_Value.Supports_Replanning :=
        (if In_Value.Supports_Replanning then 1 else 0);
      Out_Value.Supports_Typed_Component_Requirement_Placement :=
        (if In_Value.Supports_Typed_Component_Requirement_Placement then 1 else 0);
      Out_Value.Supports_State_Update_Ingress :=
        (if In_Value.Supports_State_Update_Ingress then 1 else 0);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Capabilities_C;
      Out_Value : out Capabilities)
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
      Out_Value.Backend_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Backend_Id));
      Out_Value.Supports_Planning_Requirements := In_Value.Supports_Planning_Requirements /= 0;
      Out_Value.Supports_Execution_Requirements := In_Value.Supports_Execution_Requirements /= 0;
      Out_Value.Supports_Approved_Plan_Execution := In_Value.Supports_Approved_Plan_Execution /= 0;
      Out_Value.Supports_Replanning := In_Value.Supports_Replanning /= 0;
      Out_Value.Supports_Typed_Component_Requirement_Placement := In_Value.Supports_Typed_Component_Requirement_Placement /= 0;
      Out_Value.Supports_State_Update_Ingress := In_Value.Supports_State_Update_Ingress /= 0;
   end From_C;

   procedure To_C
     (In_Value  : Planned_Component_Interaction;
      Out_Value : out Pyramid_Planned_Component_Interaction_C)
   is
   begin
      Out_Value := (others => <>);
      Dup_Str (Out_Value.Target_Component, To_String (In_Value.Target_Component));
      Dup_Str (Out_Value.Target_Service, To_String (In_Value.Target_Service));
      Dup_Str (Out_Value.Target_Type, To_String (In_Value.Target_Type));
      Out_Value.Operation := Interfaces.C.int
        (Requirement_Placement_Operation'Pos (In_Value.Operation));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Planned_Component_Interaction_C;
      Out_Value : out Planned_Component_Interaction)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Target_Component :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Component));
      Out_Value.Target_Service :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Service));
      Out_Value.Target_Type :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Type));
      Out_Value.Operation := Requirement_Placement_Operation'Val
        (Integer (In_Value.Operation));
   end From_C;

   procedure To_C
     (In_Value  : Plan_Step;
      Out_Value : out Pyramid_Plan_Step_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Out_Value.Sequence_Number := Interfaces.C.unsigned (In_Value.Sequence_Number);
      Dup_Str (Out_Value.Action_Name, To_String (In_Value.Action_Name));
      Dup_Str (Out_Value.Signature, To_String (In_Value.Signature));
      declare
         Count : constant Natural :=
           (if In_Value.Interaction = null then 0
            else In_Value.Interaction.all'Length);
      begin
         if Count > 0 then
            Out_Value.Interaction.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Planned_Component_Interaction_C'Object_Size));
            Out_Value.Interaction.Len := Interfaces.C.unsigned (Count);
            declare
               type Interaction_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Planned_Component_Interaction_C;
               pragma Convention (C, Interaction_Array_C_Array);
               Arr : Interaction_Array_C_Array;
               for Arr'Address use Out_Value.Interaction.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Interaction (I), Arr (I));
               end loop;
            end;
         end if;
      end;
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Plan_Step_C;
      Out_Value : out Plan_Step)
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
      Out_Value.Sequence_Number := Natural (In_Value.Sequence_Number);
      Out_Value.Action_Name :=
        To_Unbounded_String (To_Ada_String (In_Value.Action_Name));
      Out_Value.Signature :=
        To_Unbounded_String (To_Ada_String (In_Value.Signature));
      if In_Value.Interaction.Ptr /= System.Null_Address
        and then In_Value.Interaction.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Interaction.Len);
            type Interaction_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Planned_Component_Interaction_C;
            pragma Convention (C, Interaction_Array_C_Array);
            Arr : Interaction_Array_C_Array;
            for Arr'Address use In_Value.Interaction.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Interaction := new Interaction_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Interaction (I));
            end loop;
         end;
      else
         Out_Value.Interaction := null;
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Plan;
      Out_Value : out Pyramid_Plan_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Dup_Str (Out_Value.Planning_Requirement_Id, To_String (In_Value.Planning_Requirement_Id));
      Dup_Str (Out_Value.Backend_Id, To_String (In_Value.Backend_Id));
      Out_Value.World_Version := Interfaces.C.unsigned_long (In_Value.World_Version);
      Out_Value.Replan_Count := Interfaces.C.unsigned (In_Value.Replan_Count);
      Out_Value.Plan_Success :=
        (if In_Value.Plan_Success then 1 else 0);
      Out_Value.Solve_Time_Ms := Interfaces.C.double (In_Value.Solve_Time_Ms);
      declare
         Count : constant Natural :=
           (if In_Value.Step = null then 0
            else In_Value.Step.all'Length);
      begin
         if Count > 0 then
            Out_Value.Step.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Plan_Step_C'Object_Size));
            Out_Value.Step.Len := Interfaces.C.unsigned (Count);
            declare
               type Step_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Plan_Step_C;
               pragma Convention (C, Step_Array_C_Array);
               Arr : Step_Array_C_Array;
               for Arr'Address use Out_Value.Step.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Step (I), Arr (I));
               end loop;
            end;
         end if;
      end;
      Dup_Str (Out_Value.Compiled_Bt_Xml, To_String (In_Value.Compiled_Bt_Xml));
      Out_Value.Has_Predicted_Quality := 1;
      Out_Value.Predicted_Quality := Interfaces.C.double (In_Value.Predicted_Quality);
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Plan_C;
      Out_Value : out Plan)
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
      Out_Value.Planning_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Planning_Requirement_Id));
      Out_Value.Backend_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Backend_Id));
      Out_Value.World_Version := Long_Integer (In_Value.World_Version);
      Out_Value.Replan_Count := Natural (In_Value.Replan_Count);
      Out_Value.Plan_Success := In_Value.Plan_Success /= 0;
      Out_Value.Solve_Time_Ms := Long_Float (In_Value.Solve_Time_Ms);
      if In_Value.Step.Ptr /= System.Null_Address
        and then In_Value.Step.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Step.Len);
            type Step_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Plan_Step_C;
            pragma Convention (C, Step_Array_C_Array);
            Arr : Step_Array_C_Array;
            for Arr'Address use In_Value.Step.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Step := new Step_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Step (I));
            end loop;
         end;
      else
         Out_Value.Step := null;
      end if;
      Out_Value.Compiled_Bt_Xml :=
        To_Unbounded_String (To_Ada_String (In_Value.Compiled_Bt_Xml));
      if In_Value.Has_Predicted_Quality /= 0 then
         Out_Value.Predicted_Quality := Long_Float (In_Value.Predicted_Quality);
      end if;
   end From_C;

   procedure To_C
     (In_Value  : Requirement_Placement;
      Out_Value : out Pyramid_Requirement_Placement_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Dup_Str (Out_Value.Execution_Requirement_Id, To_String (In_Value.Execution_Requirement_Id));
      Dup_Str (Out_Value.Planning_Requirement_Id, To_String (In_Value.Planning_Requirement_Id));
      Dup_Str (Out_Value.Plan_Id, To_String (In_Value.Plan_Id));
      Dup_Str (Out_Value.Plan_Step_Id, To_String (In_Value.Plan_Step_Id));
      Dup_Str (Out_Value.Target_Component, To_String (In_Value.Target_Component));
      Dup_Str (Out_Value.Target_Service, To_String (In_Value.Target_Service));
      Dup_Str (Out_Value.Target_Type, To_String (In_Value.Target_Type));
      Out_Value.Operation := Interfaces.C.int
        (Requirement_Placement_Operation'Pos (In_Value.Operation));
      Dup_Str (Out_Value.Target_Requirement_Id, To_String (In_Value.Target_Requirement_Id));
      declare
         Count : constant Natural :=
           (if In_Value.Related_Entity_Id = null then 0
            else In_Value.Related_Entity_Id.all'Length);
      begin
         if Count > 0 then
            Out_Value.Related_Entity_Id.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Str_T'Object_Size));
            Out_Value.Related_Entity_Id.Len := Interfaces.C.unsigned (Count);
            declare
               type Related_Entity_Id_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Str_T;
               pragma Convention (C, Related_Entity_Id_Array_C_Array);
               Arr : Related_Entity_Id_Array_C_Array;
               for Arr'Address use Out_Value.Related_Entity_Id.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  Dup_Str (Arr (I), To_String (In_Value.Related_Entity_Id (I)));
               end loop;
            end;
         end if;
      end;
      Out_Value.Progress := Interfaces.C.int
        (Progress'Pos (In_Value.Val_Progress));
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Requirement_Placement_C;
      Out_Value : out Requirement_Placement)
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
      Out_Value.Execution_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Execution_Requirement_Id));
      Out_Value.Planning_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Planning_Requirement_Id));
      Out_Value.Plan_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Plan_Id));
      Out_Value.Plan_Step_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Plan_Step_Id));
      Out_Value.Target_Component :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Component));
      Out_Value.Target_Service :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Service));
      Out_Value.Target_Type :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Type));
      Out_Value.Operation := Requirement_Placement_Operation'Val
        (Integer (In_Value.Operation));
      Out_Value.Target_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Target_Requirement_Id));
      if In_Value.Related_Entity_Id.Ptr /= System.Null_Address
        and then In_Value.Related_Entity_Id.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Related_Entity_Id.Len);
            type Related_Entity_Id_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Str_T;
            pragma Convention (C, Related_Entity_Id_Array_C_Array);
            Arr : Related_Entity_Id_Array_C_Array;
            for Arr'Address use In_Value.Related_Entity_Id.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Related_Entity_Id := new Related_Entity_Id_Array (1 .. Count);
            for I in 1 .. Count loop
               Out_Value.Related_Entity_Id (I) := To_Unbounded_String (To_Ada_String (Arr (I)));
            end loop;
         end;
      else
         Out_Value.Related_Entity_Id := null;
      end if;
      Out_Value.Val_Progress := Progress'Val
        (Integer (In_Value.Progress));
   end From_C;

   procedure To_C
     (In_Value  : Execution_Run;
      Out_Value : out Pyramid_Execution_Run_C)
   is
   begin
      Out_Value := (others => <>);
      Out_Value.Has_Update_Time := 1;
      Out_Value.Update_Time := Interfaces.C.double (In_Value.Update_Time);
      Dup_Str (Out_Value.Id, To_String (In_Value.Id));
      Dup_Str (Out_Value.Source, To_String (In_Value.Source));
      Dup_Str (Out_Value.Execution_Requirement_Id, To_String (In_Value.Execution_Requirement_Id));
      Dup_Str (Out_Value.Planning_Requirement_Id, To_String (In_Value.Planning_Requirement_Id));
      Dup_Str (Out_Value.Plan_Id, To_String (In_Value.Plan_Id));
      Out_Value.State := Interfaces.C.int
        (Execution_State'Pos (In_Value.State));
      To_C (In_Value.Val_Achievement, Out_Value.Achievement);
      Out_Value.Replan_Count := Interfaces.C.unsigned (In_Value.Replan_Count);
      declare
         Count : constant Natural :=
           (if In_Value.Outstanding_Placement = null then 0
            else In_Value.Outstanding_Placement.all'Length);
      begin
         if Count > 0 then
            Out_Value.Outstanding_Placement.Ptr :=
              Malloc (Bytes_For (Count, Pyramid_Requirement_Placement_C'Object_Size));
            Out_Value.Outstanding_Placement.Len := Interfaces.C.unsigned (Count);
            declare
               type Outstanding_Placement_Array_C_Array is array (Positive range 1 .. Count)
                 of Pyramid_Requirement_Placement_C;
               pragma Convention (C, Outstanding_Placement_Array_C_Array);
               Arr : Outstanding_Placement_Array_C_Array;
               for Arr'Address use Out_Value.Outstanding_Placement.Ptr;
               pragma Import (Ada, Arr);
            begin
               for I in 1 .. Count loop
                  To_C (In_Value.Outstanding_Placement (I), Arr (I));
               end loop;
            end;
         end if;
      end;
   end To_C;

   procedure From_C
     (In_Value  : Pyramid_Execution_Run_C;
      Out_Value : out Execution_Run)
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
      Out_Value.Execution_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Execution_Requirement_Id));
      Out_Value.Planning_Requirement_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Planning_Requirement_Id));
      Out_Value.Plan_Id :=
        To_Unbounded_String (To_Ada_String (In_Value.Plan_Id));
      Out_Value.State := Execution_State'Val
        (Integer (In_Value.State));
      From_C (In_Value.Achievement, Out_Value.Val_Achievement);
      Out_Value.Replan_Count := Natural (In_Value.Replan_Count);
      if In_Value.Outstanding_Placement.Ptr /= System.Null_Address
        and then In_Value.Outstanding_Placement.Len > 0
      then
         declare
            Count : constant Natural := Natural (In_Value.Outstanding_Placement.Len);
            type Outstanding_Placement_Array_C_Array is array (Positive range 1 .. Count)
              of Pyramid_Requirement_Placement_C;
            pragma Convention (C, Outstanding_Placement_Array_C_Array);
            Arr : Outstanding_Placement_Array_C_Array;
            for Arr'Address use In_Value.Outstanding_Placement.Ptr;
            pragma Import (Ada, Arr);
         begin
            Out_Value.Outstanding_Placement := new Outstanding_Placement_Array (1 .. Count);
            for I in 1 .. Count loop
               From_C (Arr (I), Out_Value.Outstanding_Placement (I));
            end loop;
         end;
      else
         Out_Value.Outstanding_Placement := null;
      end if;
   end From_C;

end Pyramid.Data_Model.Autonomy.Cabi;
