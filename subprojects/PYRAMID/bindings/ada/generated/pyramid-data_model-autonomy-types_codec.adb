--  Auto-generated data model JSON codec body
--  Package: Pyramid.Data_Model.Autonomy.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Pyramid.Data_Model.Common.Types_Codec;
pragma Warnings (Off);

package body Pyramid.Data_Model.Autonomy.Types_Codec is

   function To_String (V : Fact_Authority_Level) return String is
   begin
      case V is
         when Level_Unspecified => return "FACT_AUTHORITY_LEVEL_UNSPECIFIED";
         when Level_Believed => return "FACT_AUTHORITY_LEVEL_BELIEVED";
         when Level_Confirmed => return "FACT_AUTHORITY_LEVEL_CONFIRMED";
      end case;
   end To_String;

   function Fact_Authority_Level_From_String (S : String) return Fact_Authority_Level is
   begin
      if S = "FACT_AUTHORITY_LEVEL_UNSPECIFIED" then return Level_Unspecified; end if;
      if S = "FACT_AUTHORITY_LEVEL_BELIEVED" then return Level_Believed; end if;
      if S = "FACT_AUTHORITY_LEVEL_CONFIRMED" then return Level_Confirmed; end if;
      return Level_Unspecified;
   end Fact_Authority_Level_From_String;

   function To_String (V : Execution_State) return String is
   begin
      case V is
         when State_Unspecified => return "EXECUTION_STATE_UNSPECIFIED";
         when State_Accepted => return "EXECUTION_STATE_ACCEPTED";
         when State_Executing => return "EXECUTION_STATE_EXECUTING";
         when State_WaitingForComponents => return "EXECUTION_STATE_WAITING_FOR_COMPONENTS";
         when State_Achieved => return "EXECUTION_STATE_ACHIEVED";
         when State_Failed => return "EXECUTION_STATE_FAILED";
         when State_Cancelled => return "EXECUTION_STATE_CANCELLED";
      end case;
   end To_String;

   function Execution_State_From_String (S : String) return Execution_State is
   begin
      if S = "EXECUTION_STATE_UNSPECIFIED" then return State_Unspecified; end if;
      if S = "EXECUTION_STATE_ACCEPTED" then return State_Accepted; end if;
      if S = "EXECUTION_STATE_EXECUTING" then return State_Executing; end if;
      if S = "EXECUTION_STATE_WAITING_FOR_COMPONENTS" then return State_WaitingForComponents; end if;
      if S = "EXECUTION_STATE_ACHIEVED" then return State_Achieved; end if;
      if S = "EXECUTION_STATE_FAILED" then return State_Failed; end if;
      if S = "EXECUTION_STATE_CANCELLED" then return State_Cancelled; end if;
      return State_Unspecified;
   end Execution_State_From_String;

   function To_String (V : Requirement_Placement_Operation) return String is
   begin
      case V is
         when Operation_Unspecified => return "REQUIREMENT_PLACEMENT_OPERATION_UNSPECIFIED";
         when Operation_CreateRequirement => return "REQUIREMENT_PLACEMENT_OPERATION_CREATE_REQUIREMENT";
         when Operation_ReadRequirement => return "REQUIREMENT_PLACEMENT_OPERATION_READ_REQUIREMENT";
         when Operation_UpdateRequirement => return "REQUIREMENT_PLACEMENT_OPERATION_UPDATE_REQUIREMENT";
         when Operation_DeleteRequirement => return "REQUIREMENT_PLACEMENT_OPERATION_DELETE_REQUIREMENT";
         when Operation_ReadProduct => return "REQUIREMENT_PLACEMENT_OPERATION_READ_PRODUCT";
         when Operation_ReadCapability => return "REQUIREMENT_PLACEMENT_OPERATION_READ_CAPABILITY";
      end case;
   end To_String;

   function Requirement_Placement_Operation_From_String (S : String) return Requirement_Placement_Operation is
   begin
      if S = "REQUIREMENT_PLACEMENT_OPERATION_UNSPECIFIED" then return Operation_Unspecified; end if;
      if S = "REQUIREMENT_PLACEMENT_OPERATION_CREATE_REQUIREMENT" then return Operation_CreateRequirement; end if;
      if S = "REQUIREMENT_PLACEMENT_OPERATION_READ_REQUIREMENT" then return Operation_ReadRequirement; end if;
      if S = "REQUIREMENT_PLACEMENT_OPERATION_UPDATE_REQUIREMENT" then return Operation_UpdateRequirement; end if;
      if S = "REQUIREMENT_PLACEMENT_OPERATION_DELETE_REQUIREMENT" then return Operation_DeleteRequirement; end if;
      if S = "REQUIREMENT_PLACEMENT_OPERATION_READ_PRODUCT" then return Operation_ReadProduct; end if;
      if S = "REQUIREMENT_PLACEMENT_OPERATION_READ_CAPABILITY" then return Operation_ReadCapability; end if;
      return Operation_Unspecified;
   end Requirement_Placement_Operation_From_String;

   function To_Json (Msg : Requirement_Reference) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Requirement_Id) & """");
      Comma;
      Append (Result, """component_name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Component_Name) & """");
      Comma;
      Append (Result, """service_name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Service_Name) & """");
      Comma;
      Append (Result, """type_name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Type_Name) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Requirement_Reference) return Requirement_Reference is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Requirement_Reference;
   begin
      if Has_Field (J, "requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "component_name") then
         declare
            Val : constant JSON_Value := Get (J, "component_name");
            Str : constant String := Get (Val);
         begin
            Result.Component_Name := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "service_name") then
         declare
            Val : constant JSON_Value := Get (J, "service_name");
            Str : constant String := Get (Val);
         begin
            Result.Service_Name := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "type_name") then
         declare
            Val : constant JSON_Value := Get (J, "type_name");
            Str : constant String := Get (Val);
         begin
            Result.Type_Name := To_Unbounded_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Agent_State) return String is
   begin
      return "{" &
        """agent_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Agent_Id) & """" &
        "," &
        """agent_type"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Agent_Type) & """" &
        "," &
        """available"":" & (if Msg.Available then "true" else "false") &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Agent_State) return Agent_State is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Agent_State;
   begin
      if Has_Field (J, "agent_id") then
         declare
            Val : constant JSON_Value := Get (J, "agent_id");
            Str : constant String := Get (Val);
         begin
            Result.Agent_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "agent_type") then
         declare
            Val : constant JSON_Value := Get (J, "agent_type");
            Str : constant String := Get (Val);
         begin
            Result.Agent_Type := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "available") then
         declare
            Val : constant JSON_Value := Get (J, "available");
         begin
            Result.Available := Get (Val);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Planning_Policy) return String is
   begin
      return "{" &
        """max_replans"":" & Natural'Image (Msg.Max_Replans) &
        "," &
        """enable_replanning"":" & (if Msg.Enable_Replanning then "true" else "false") &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Planning_Policy) return Planning_Policy is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Planning_Policy;
   begin
      if Has_Field (J, "max_replans") then
         Result.Max_Replans := Natural (Get_Long_Float (Get (J, "max_replans")));
      end if;
      if Has_Field (J, "enable_replanning") then
         declare
            Val : constant JSON_Value := Get (J, "enable_replanning");
         begin
            Result.Enable_Replanning := Get (Val);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Planning_Goal) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Name) & """");
      if Msg.Has_Requirement then
         Comma;
         Append (Result, """requirement"":" & To_Json (Msg.Requirement));
      end if;
      if Msg.Has_Expression then
         Comma;
         Append (Result, """expression"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Expression) & """");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Planning_Goal) return Planning_Goal is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Planning_Goal;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "name") then
         declare
            Val : constant JSON_Value := Get (J, "name");
            Str : constant String := Get (Val);
         begin
            Result.Name := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "requirement") then
         Result.Has_Requirement := True;
         declare
            Sub : constant String := Write (Get (J, "requirement"));
         begin
            Result.Requirement := From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "expression") then
         Result.Has_Expression := True;
         if Has_Field (J, "expression") then
            declare
               Val : constant JSON_Value := Get (J, "expression");
               Str : constant String := Get (Val);
            begin
               Result.Expression := To_Unbounded_String (Str);
            end;
         end if;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Execution_Policy) return String is
   begin
      return "{" &
        """max_replans"":" & Natural'Image (Msg.Max_Replans) &
        "," &
        """enable_replanning"":" & (if Msg.Enable_Replanning then "true" else "false") &
        "," &
        """max_concurrent_placements"":" & Natural'Image (Msg.Max_Concurrent_Placements) &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Execution_Policy) return Execution_Policy is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Execution_Policy;
   begin
      if Has_Field (J, "max_replans") then
         Result.Max_Replans := Natural (Get_Long_Float (Get (J, "max_replans")));
      end if;
      if Has_Field (J, "enable_replanning") then
         declare
            Val : constant JSON_Value := Get (J, "enable_replanning");
         begin
            Result.Enable_Replanning := Get (Val);
         end;
      end if;
      if Has_Field (J, "max_concurrent_placements") then
         Result.Max_Concurrent_Placements := Natural (Get_Long_Float (Get (J, "max_concurrent_placements")));
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Planning_Requirement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Base));
      Comma;
      Append (Result, """status"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Status));
      if Msg.Upstream_Requirement /= null then
         Comma;
         Append (Result, """upstream_requirement"":[");
         for I in Msg.Upstream_Requirement'Range loop
            if I > Msg.Upstream_Requirement'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Upstream_Requirement (I)));
         end loop;
         Append (Result, "]");
      end if;
      if Msg.Goal /= null then
         Comma;
         Append (Result, """goal"":[");
         for I in Msg.Goal'Range loop
            if I > Msg.Goal'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Goal (I)));
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """policy"":" & To_Json (Msg.Policy));
      if Msg.Available_Agents /= null then
         Comma;
         Append (Result, """available_agents"":[");
         for I in Msg.Available_Agents'Range loop
            if I > Msg.Available_Agents'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Available_Agents (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Planning_Requirement) return Planning_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Planning_Requirement;
   begin
      if Has_Field (J, "base") then
         declare
            Sub : constant String := Write (Get (J, "base"));
         begin
            Result.Base := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "upstream_requirement") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "upstream_requirement");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Upstream_Requirement := new Upstream_Requirement_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Upstream_Requirement (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "goal") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "goal");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Goal := new Goal_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Goal (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "policy") then
         declare
            Sub : constant String := Write (Get (J, "policy"));
         begin
            Result.Policy := From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "available_agents") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "available_agents");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Available_Agents := new Available_Agents_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Available_Agents (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Execution_Requirement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """base"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Base));
      Comma;
      Append (Result, """status"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Status));
      if Msg.Upstream_Requirement /= null then
         Comma;
         Append (Result, """upstream_requirement"":[");
         for I in Msg.Upstream_Requirement'Range loop
            if I > Msg.Upstream_Requirement'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Upstream_Requirement (I)));
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """plan_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Plan_Id) & """");
      Comma;
      Append (Result, """policy"":" & To_Json (Msg.Policy));
      if Msg.Available_Agents /= null then
         Comma;
         Append (Result, """available_agents"":[");
         for I in Msg.Available_Agents'Range loop
            if I > Msg.Available_Agents'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Available_Agents (I)));
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """planning_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Planning_Requirement_Id) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Execution_Requirement) return Execution_Requirement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Execution_Requirement;
   begin
      if Has_Field (J, "base") then
         declare
            Sub : constant String := Write (Get (J, "base"));
         begin
            Result.Base := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Sub : constant String := Write (Get (J, "status"));
         begin
            Result.Status := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "upstream_requirement") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "upstream_requirement");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Upstream_Requirement := new Upstream_Requirement_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Upstream_Requirement (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "plan_id") then
         declare
            Val : constant JSON_Value := Get (J, "plan_id");
            Str : constant String := Get (Val);
         begin
            Result.Plan_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "policy") then
         declare
            Sub : constant String := Write (Get (J, "policy"));
         begin
            Result.Policy := From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "available_agents") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "available_agents");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Available_Agents := new Available_Agents_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Available_Agents (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "planning_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "planning_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Planning_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : World_Fact_Update) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """entity_source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Entity_Source) & """");
      Comma;
      Append (Result, """key"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Key) & """");
      Comma;
      Append (Result, """value"":" & (if Msg.Value then "true" else "false"));
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """authority"":" & """" & To_String (Msg.Authority) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access World_Fact_Update) return World_Fact_Update is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : World_Fact_Update;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "entity_source") then
         declare
            Val : constant JSON_Value := Get (J, "entity_source");
            Str : constant String := Get (Val);
         begin
            Result.Entity_Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "key") then
         declare
            Val : constant JSON_Value := Get (J, "key");
            Str : constant String := Get (Val);
         begin
            Result.Key := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "value") then
         declare
            Val : constant JSON_Value := Get (J, "value");
         begin
            Result.Value := Get (Val);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "authority") then
         declare
            Val : constant JSON_Value := Get (J, "authority");
            Str : constant String := Get (Val);
         begin
            Result.Authority := Fact_Authority_Level_From_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : State_Update) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      if Msg.Fact_Update /= null then
         Comma;
         Append (Result, """fact_update"":[");
         for I in Msg.Fact_Update'Range loop
            if I > Msg.Fact_Update'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Fact_Update (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access State_Update) return State_Update is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : State_Update;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "fact_update") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "fact_update");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Fact_Update := new Fact_Update_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Fact_Update (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Capabilities) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """backend_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Backend_Id) & """");
      Comma;
      Append (Result, """supports_planning_requirements"":" & (if Msg.Supports_Planning_Requirements then "true" else "false"));
      Comma;
      Append (Result, """supports_execution_requirements"":" & (if Msg.Supports_Execution_Requirements then "true" else "false"));
      Comma;
      Append (Result, """supports_approved_plan_execution"":" & (if Msg.Supports_Approved_Plan_Execution then "true" else "false"));
      Comma;
      Append (Result, """supports_replanning"":" & (if Msg.Supports_Replanning then "true" else "false"));
      Comma;
      Append (Result, """supports_typed_component_requirement_placement"":" & (if Msg.Supports_Typed_Component_Requirement_Placement then "true" else "false"));
      Comma;
      Append (Result, """supports_state_update_ingress"":" & (if Msg.Supports_State_Update_Ingress then "true" else "false"));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Capabilities) return Capabilities is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Capabilities;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "backend_id") then
         declare
            Val : constant JSON_Value := Get (J, "backend_id");
            Str : constant String := Get (Val);
         begin
            Result.Backend_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "supports_planning_requirements") then
         declare
            Val : constant JSON_Value := Get (J, "supports_planning_requirements");
         begin
            Result.Supports_Planning_Requirements := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_execution_requirements") then
         declare
            Val : constant JSON_Value := Get (J, "supports_execution_requirements");
         begin
            Result.Supports_Execution_Requirements := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_approved_plan_execution") then
         declare
            Val : constant JSON_Value := Get (J, "supports_approved_plan_execution");
         begin
            Result.Supports_Approved_Plan_Execution := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_replanning") then
         declare
            Val : constant JSON_Value := Get (J, "supports_replanning");
         begin
            Result.Supports_Replanning := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_typed_component_requirement_placement") then
         declare
            Val : constant JSON_Value := Get (J, "supports_typed_component_requirement_placement");
         begin
            Result.Supports_Typed_Component_Requirement_Placement := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_state_update_ingress") then
         declare
            Val : constant JSON_Value := Get (J, "supports_state_update_ingress");
         begin
            Result.Supports_State_Update_Ingress := Get (Val);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Planned_Component_Interaction) return String is
   begin
      return "{" &
        """target_component"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Component) & """" &
        "," &
        """target_service"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Service) & """" &
        "," &
        """target_type"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Type) & """" &
        "," &
        """operation"":" & """" & To_String (Msg.Operation) & """" &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Planned_Component_Interaction) return Planned_Component_Interaction is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Planned_Component_Interaction;
   begin
      if Has_Field (J, "target_component") then
         declare
            Val : constant JSON_Value := Get (J, "target_component");
            Str : constant String := Get (Val);
         begin
            Result.Target_Component := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "target_service") then
         declare
            Val : constant JSON_Value := Get (J, "target_service");
            Str : constant String := Get (Val);
         begin
            Result.Target_Service := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "target_type") then
         declare
            Val : constant JSON_Value := Get (J, "target_type");
            Str : constant String := Get (Val);
         begin
            Result.Target_Type := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "operation") then
         declare
            Val : constant JSON_Value := Get (J, "operation");
            Str : constant String := Get (Val);
         begin
            Result.Operation := Requirement_Placement_Operation_From_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Plan_Step) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """sequence_number"":" & Natural'Image (Msg.Sequence_Number));
      Comma;
      Append (Result, """action_name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Action_Name) & """");
      Comma;
      Append (Result, """signature"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Signature) & """");
      if Msg.Interaction /= null then
         Comma;
         Append (Result, """interaction"":[");
         for I in Msg.Interaction'Range loop
            if I > Msg.Interaction'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Interaction (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Plan_Step) return Plan_Step is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Plan_Step;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "sequence_number") then
         Result.Sequence_Number := Natural (Get_Long_Float (Get (J, "sequence_number")));
      end if;
      if Has_Field (J, "action_name") then
         declare
            Val : constant JSON_Value := Get (J, "action_name");
            Str : constant String := Get (Val);
         begin
            Result.Action_Name := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "signature") then
         declare
            Val : constant JSON_Value := Get (J, "signature");
            Str : constant String := Get (Val);
         begin
            Result.Signature := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "interaction") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "interaction");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Interaction := new Interaction_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Interaction (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Plan) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """planning_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Planning_Requirement_Id) & """");
      Comma;
      Append (Result, """backend_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Backend_Id) & """");
      Comma;
      Append (Result, """world_version"":" & Long_Integer'Image (Msg.World_Version));
      Comma;
      Append (Result, """replan_count"":" & Natural'Image (Msg.Replan_Count));
      Comma;
      Append (Result, """plan_success"":" & (if Msg.Plan_Success then "true" else "false"));
      Comma;
      Append (Result, """solve_time_ms"":" & Long_Float'Image (Msg.Solve_Time_Ms));
      if Msg.Step /= null then
         Comma;
         Append (Result, """step"":[");
         for I in Msg.Step'Range loop
            if I > Msg.Step'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Step (I)));
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """compiled_bt_xml"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Compiled_Bt_Xml) & """");
      Comma;
      Append (Result, """predicted_quality"":" & Long_Float'Image (Msg.Predicted_Quality));
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Plan) return Plan is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Plan;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "planning_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "planning_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Planning_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "backend_id") then
         declare
            Val : constant JSON_Value := Get (J, "backend_id");
            Str : constant String := Get (Val);
         begin
            Result.Backend_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "world_version") then
         Result.World_Version := Long_Integer (Get_Long_Float (Get (J, "world_version")));
      end if;
      if Has_Field (J, "replan_count") then
         Result.Replan_Count := Natural (Get_Long_Float (Get (J, "replan_count")));
      end if;
      if Has_Field (J, "plan_success") then
         declare
            Val : constant JSON_Value := Get (J, "plan_success");
         begin
            Result.Plan_Success := Get (Val);
         end;
      end if;
      if Has_Field (J, "solve_time_ms") then
         Result.Solve_Time_Ms := Get_Long_Float (Get (J, "solve_time_ms"));
      end if;
      if Has_Field (J, "step") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "step");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Step := new Step_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Step (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "compiled_bt_xml") then
         declare
            Val : constant JSON_Value := Get (J, "compiled_bt_xml");
            Str : constant String := Get (Val);
         begin
            Result.Compiled_Bt_Xml := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "predicted_quality") then
         Result.Predicted_Quality := Get_Long_Float (Get (J, "predicted_quality"));
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Requirement_Placement) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """execution_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Execution_Requirement_Id) & """");
      Comma;
      Append (Result, """planning_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Planning_Requirement_Id) & """");
      Comma;
      Append (Result, """plan_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Plan_Id) & """");
      Comma;
      Append (Result, """plan_step_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Plan_Step_Id) & """");
      Comma;
      Append (Result, """target_component"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Component) & """");
      Comma;
      Append (Result, """target_service"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Service) & """");
      Comma;
      Append (Result, """target_type"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Type) & """");
      Comma;
      Append (Result, """operation"":" & """" & To_String (Msg.Operation) & """");
      Comma;
      Append (Result, """target_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Target_Requirement_Id) & """");
      if Msg.Related_Entity_Id /= null then
         Comma;
         Append (Result, """related_entity_id"":[");
         for I in Msg.Related_Entity_Id'Range loop
            if I > Msg.Related_Entity_Id'First then
               Append (Result, ",");
            end if;
            Append (Result, """" & Ada.Strings.Unbounded.To_String (Msg.Related_Entity_Id (I)) & """");
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """progress"":" & """" & Pyramid.Data_Model.Common.Types_Codec.To_String (Msg.Val_Progress) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Requirement_Placement) return Requirement_Placement is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Requirement_Placement;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "execution_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "execution_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Execution_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "planning_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "planning_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Planning_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "plan_id") then
         declare
            Val : constant JSON_Value := Get (J, "plan_id");
            Str : constant String := Get (Val);
         begin
            Result.Plan_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "plan_step_id") then
         declare
            Val : constant JSON_Value := Get (J, "plan_step_id");
            Str : constant String := Get (Val);
         begin
            Result.Plan_Step_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "target_component") then
         declare
            Val : constant JSON_Value := Get (J, "target_component");
            Str : constant String := Get (Val);
         begin
            Result.Target_Component := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "target_service") then
         declare
            Val : constant JSON_Value := Get (J, "target_service");
            Str : constant String := Get (Val);
         begin
            Result.Target_Service := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "target_type") then
         declare
            Val : constant JSON_Value := Get (J, "target_type");
            Str : constant String := Get (Val);
         begin
            Result.Target_Type := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "operation") then
         declare
            Val : constant JSON_Value := Get (J, "operation");
            Str : constant String := Get (Val);
         begin
            Result.Operation := Requirement_Placement_Operation_From_String (Str);
         end;
      end if;
      if Has_Field (J, "target_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "target_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Target_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "related_entity_id") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "related_entity_id");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Related_Entity_Id := new Related_Entity_Id_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Str : constant String := Get (Elem);
                  begin
                     Result.Related_Entity_Id (I) := To_Unbounded_String (Str);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "progress") then
         declare
            Val : constant JSON_Value := Get (J, "progress");
            Str : constant String := Get (Val);
         begin
            Result.Val_Progress := Pyramid.Data_Model.Common.Types_Codec.Progress_From_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Execution_Run) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """update_time"":" & Long_Float'Image (Msg.Update_Time));
      Comma;
      Append (Result, """id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Id) & """");
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Comma;
      Append (Result, """execution_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Execution_Requirement_Id) & """");
      Comma;
      Append (Result, """planning_requirement_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Planning_Requirement_Id) & """");
      Comma;
      Append (Result, """plan_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Plan_Id) & """");
      Comma;
      Append (Result, """state"":" & """" & To_String (Msg.State) & """");
      Comma;
      Append (Result, """achievement"":" & Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg.Val_Achievement));
      Comma;
      Append (Result, """replan_count"":" & Natural'Image (Msg.Replan_Count));
      if Msg.Outstanding_Placement /= null then
         Comma;
         Append (Result, """outstanding_placement"":[");
         for I in Msg.Outstanding_Placement'Range loop
            if I > Msg.Outstanding_Placement'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Outstanding_Placement (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Execution_Run) return Execution_Run is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Execution_Run;
   begin
      if Has_Field (J, "update_time") then
         Result.Update_Time := Get_Long_Float (Get (J, "update_time"));
      end if;
      if Has_Field (J, "id") then
         declare
            Val : constant JSON_Value := Get (J, "id");
            Str : constant String := Get (Val);
         begin
            Result.Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "source") then
         declare
            Val : constant JSON_Value := Get (J, "source");
            Str : constant String := Get (Val);
         begin
            Result.Source := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "execution_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "execution_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Execution_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "planning_requirement_id") then
         declare
            Val : constant JSON_Value := Get (J, "planning_requirement_id");
            Str : constant String := Get (Val);
         begin
            Result.Planning_Requirement_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "plan_id") then
         declare
            Val : constant JSON_Value := Get (J, "plan_id");
            Str : constant String := Get (Val);
         begin
            Result.Plan_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "state") then
         declare
            Val : constant JSON_Value := Get (J, "state");
            Str : constant String := Get (Val);
         begin
            Result.State := Execution_State_From_String (Str);
         end;
      end if;
      if Has_Field (J, "achievement") then
         declare
            Sub : constant String := Write (Get (J, "achievement"));
         begin
            Result.Val_Achievement := Pyramid.Data_Model.Common.Types_Codec.From_Json (Sub, null);
         end;
      end if;
      if Has_Field (J, "replan_count") then
         Result.Replan_Count := Natural (Get_Long_Float (Get (J, "replan_count")));
      end if;
      if Has_Field (J, "outstanding_placement") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "outstanding_placement");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Outstanding_Placement := new Outstanding_Placement_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Outstanding_Placement (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

end Pyramid.Data_Model.Autonomy.Types_Codec;
