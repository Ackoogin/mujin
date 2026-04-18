--  Auto-generated data model JSON codec body
--  Package: Pyramid.Data_Model.Autonomy.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
pragma Warnings (Off);

package body Pyramid.Data_Model.Autonomy.Types_Codec is

   function To_String (V : Autonomy_Backend_State) return String is
   begin
      case V is
         when State_Unspecified => return "AUTONOMY_BACKEND_STATE_UNSPECIFIED";
         when State_Idle => return "AUTONOMY_BACKEND_STATE_IDLE";
         when State_Ready => return "AUTONOMY_BACKEND_STATE_READY";
         when State_WaitingForResults => return "AUTONOMY_BACKEND_STATE_WAITING_FOR_RESULTS";
         when State_Complete => return "AUTONOMY_BACKEND_STATE_COMPLETE";
         when State_Failed => return "AUTONOMY_BACKEND_STATE_FAILED";
         when State_Stopped => return "AUTONOMY_BACKEND_STATE_STOPPED";
      end case;
   end To_String;

   function Autonomy_Backend_State_From_String (S : String) return Autonomy_Backend_State is
   begin
      if S = "AUTONOMY_BACKEND_STATE_UNSPECIFIED" then return State_Unspecified; end if;
      if S = "AUTONOMY_BACKEND_STATE_IDLE" then return State_Idle; end if;
      if S = "AUTONOMY_BACKEND_STATE_READY" then return State_Ready; end if;
      if S = "AUTONOMY_BACKEND_STATE_WAITING_FOR_RESULTS" then return State_WaitingForResults; end if;
      if S = "AUTONOMY_BACKEND_STATE_COMPLETE" then return State_Complete; end if;
      if S = "AUTONOMY_BACKEND_STATE_FAILED" then return State_Failed; end if;
      if S = "AUTONOMY_BACKEND_STATE_STOPPED" then return State_Stopped; end if;
      return State_Unspecified;
   end Autonomy_Backend_State_From_String;

   function To_String (V : Command_Status) return String is
   begin
      case V is
         when Status_Unspecified => return "COMMAND_STATUS_UNSPECIFIED";
         when Status_Pending => return "COMMAND_STATUS_PENDING";
         when Status_Running => return "COMMAND_STATUS_RUNNING";
         when Status_Succeeded => return "COMMAND_STATUS_SUCCEEDED";
         when Status_FailedTransient => return "COMMAND_STATUS_FAILED_TRANSIENT";
         when Status_FailedPermanent => return "COMMAND_STATUS_FAILED_PERMANENT";
         when Status_Cancelled => return "COMMAND_STATUS_CANCELLED";
      end case;
   end To_String;

   function Command_Status_From_String (S : String) return Command_Status is
   begin
      if S = "COMMAND_STATUS_UNSPECIFIED" then return Status_Unspecified; end if;
      if S = "COMMAND_STATUS_PENDING" then return Status_Pending; end if;
      if S = "COMMAND_STATUS_RUNNING" then return Status_Running; end if;
      if S = "COMMAND_STATUS_SUCCEEDED" then return Status_Succeeded; end if;
      if S = "COMMAND_STATUS_FAILED_TRANSIENT" then return Status_FailedTransient; end if;
      if S = "COMMAND_STATUS_FAILED_PERMANENT" then return Status_FailedPermanent; end if;
      if S = "COMMAND_STATUS_CANCELLED" then return Status_Cancelled; end if;
      return Status_Unspecified;
   end Command_Status_From_String;

   function To_String (V : Stop_Mode) return String is
   begin
      case V is
         when Mode_Unspecified => return "STOP_MODE_UNSPECIFIED";
         when Mode_Drain => return "STOP_MODE_DRAIN";
         when Mode_Immediate => return "STOP_MODE_IMMEDIATE";
      end case;
   end To_String;

   function Stop_Mode_From_String (S : String) return Stop_Mode is
   begin
      if S = "STOP_MODE_UNSPECIFIED" then return Mode_Unspecified; end if;
      if S = "STOP_MODE_DRAIN" then return Mode_Drain; end if;
      if S = "STOP_MODE_IMMEDIATE" then return Mode_Immediate; end if;
      return Mode_Unspecified;
   end Stop_Mode_From_String;

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

   function To_Json (Msg : Fact_Update) return String is
   begin
      return "{" &
        """key"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Key) & """" &
        "," &
        """value"":" & (if Msg.Value then "true" else "false") &
        "," &
        """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """" &
        "," &
        """authority"":" & """" & To_String (Msg.Authority) & """" &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Fact_Update) return Fact_Update is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Fact_Update;
   begin
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
      if Msg.Fact_Updates /= null then
         Comma;
         Append (Result, """fact_updates"":[");
         for I in Msg.Fact_Updates'Range loop
            if I > Msg.Fact_Updates'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Fact_Updates (I)));
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
      if Has_Field (J, "fact_updates") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "fact_updates");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Fact_Updates := new Fact_Updates_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Fact_Updates (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Mission_Intent) return String is
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
      if Msg.Goal_Fluents /= null then
         Comma;
         Append (Result, """goal_fluents"":[");
         for I in Msg.Goal_Fluents'Range loop
            if I > Msg.Goal_Fluents'First then
               Append (Result, ",");
            end if;
            Append (Result, """" & Ada.Strings.Unbounded.To_String (Msg.Goal_Fluents (I)) & """");
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Mission_Intent) return Mission_Intent is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Mission_Intent;
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
      if Has_Field (J, "goal_fluents") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "goal_fluents");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Goal_Fluents := new Goal_Fluents_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Str : constant String := Get (Elem);
                  begin
                     Result.Goal_Fluents (I) := To_Unbounded_String (Str);
                  end;
               end loop;
            end if;
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

   function To_Json (Msg : Policy_Envelope) return String is
   begin
      return "{" &
        """max_replans"":" & Integer'Image (Msg.Max_Replans) &
        "," &
        """enable_goal_dispatch"":" & (if Msg.Enable_Goal_Dispatch then "true" else "false") &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access Policy_Envelope) return Policy_Envelope is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Policy_Envelope;
   begin
      if Has_Field (J, "max_replans") then
         Result.Max_Replans := Integer (Get_Long_Float (Get (J, "max_replans")));
      end if;
      if Has_Field (J, "enable_goal_dispatch") then
         declare
            Val : constant JSON_Value := Get (J, "enable_goal_dispatch");
         begin
            Result.Enable_Goal_Dispatch := Get (Val);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Session) return String is
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
      Append (Result, """intent"":" & To_Json (Msg.Intent));
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

   function From_Json (S : String; Tag : access Session) return Session is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Session;
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
      if Has_Field (J, "intent") then
         declare
            Sub : constant String := Write (Get (J, "intent"));
         begin
            Result.Intent := From_Json (Sub, null);
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
      Append (Result, """supports_batch_planning"":" & (if Msg.Supports_Batch_Planning then "true" else "false"));
      Comma;
      Append (Result, """supports_external_command_dispatch"":" & (if Msg.Supports_External_Command_Dispatch then "true" else "false"));
      Comma;
      Append (Result, """supports_replanning"":" & (if Msg.Supports_Replanning then "true" else "false"));
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
      if Has_Field (J, "supports_batch_planning") then
         declare
            Val : constant JSON_Value := Get (J, "supports_batch_planning");
         begin
            Result.Supports_Batch_Planning := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_external_command_dispatch") then
         declare
            Val : constant JSON_Value := Get (J, "supports_external_command_dispatch");
         begin
            Result.Supports_External_Command_Dispatch := Get (Val);
         end;
      end if;
      if Has_Field (J, "supports_replanning") then
         declare
            Val : constant JSON_Value := Get (J, "supports_replanning");
         begin
            Result.Supports_Replanning := Get (Val);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : String_Key_Value) return String is
   begin
      return "{" &
        """key"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Key) & """" &
        "," &
        """value"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Value) & """" &
        "}";
   end To_Json;

   function From_Json (S : String; Tag : access String_Key_Value) return String_Key_Value is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : String_Key_Value;
   begin
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
            Str : constant String := Get (Val);
         begin
            Result.Value := To_Unbounded_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Command) return String is
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
      Append (Result, """command_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Command_Id) & """");
      Comma;
      Append (Result, """action_name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Action_Name) & """");
      Comma;
      Append (Result, """signature"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Signature) & """");
      Comma;
      Append (Result, """service_name"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Service_Name) & """");
      Comma;
      Append (Result, """operation"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Operation) & """");
      if Msg.Request_Fields /= null then
         Comma;
         Append (Result, """request_fields"":[");
         for I in Msg.Request_Fields'Range loop
            if I > Msg.Request_Fields'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Request_Fields (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Command) return Command is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Command;
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
      if Has_Field (J, "command_id") then
         declare
            Val : constant JSON_Value := Get (J, "command_id");
            Str : constant String := Get (Val);
         begin
            Result.Command_Id := To_Unbounded_String (Str);
         end;
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
      if Has_Field (J, "service_name") then
         declare
            Val : constant JSON_Value := Get (J, "service_name");
            Str : constant String := Get (Val);
         begin
            Result.Service_Name := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "operation") then
         declare
            Val : constant JSON_Value := Get (J, "operation");
            Str : constant String := Get (Val);
         begin
            Result.Operation := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "request_fields") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "request_fields");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Request_Fields := new Request_Fields_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Request_Fields (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Goal_Dispatch) return String is
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
      Append (Result, """dispatch_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Dispatch_Id) & """");
      Comma;
      Append (Result, """agent_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Agent_Id) & """");
      if Msg.Goals /= null then
         Comma;
         Append (Result, """goals"":[");
         for I in Msg.Goals'Range loop
            if I > Msg.Goals'First then
               Append (Result, ",");
            end if;
            Append (Result, """" & Ada.Strings.Unbounded.To_String (Msg.Goals (I)) & """");
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Goal_Dispatch) return Goal_Dispatch is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Goal_Dispatch;
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
      if Has_Field (J, "dispatch_id") then
         declare
            Val : constant JSON_Value := Get (J, "dispatch_id");
            Str : constant String := Get (Val);
         begin
            Result.Dispatch_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "agent_id") then
         declare
            Val : constant JSON_Value := Get (J, "agent_id");
            Str : constant String := Get (Val);
         begin
            Result.Agent_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "goals") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "goals");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Goals := new Goals_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Str : constant String := Get (Elem);
                  begin
                     Result.Goals (I) := To_Unbounded_String (Str);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Decision_Record) return String is
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
      Append (Result, """session_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Session_Id) & """");
      Comma;
      Append (Result, """backend_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Backend_Id) & """");
      Comma;
      Append (Result, """world_version"":" & Integer'Image (Msg.World_Version));
      Comma;
      Append (Result, """replan_count"":" & Integer'Image (Msg.Replan_Count));
      Comma;
      Append (Result, """plan_success"":" & (if Msg.Plan_Success then "true" else "false"));
      Comma;
      Append (Result, """solve_time_ms"":" & Long_Float'Image (Msg.Solve_Time_Ms));
      if Msg.Planned_Action_Signatures /= null then
         Comma;
         Append (Result, """planned_action_signatures"":[");
         for I in Msg.Planned_Action_Signatures'Range loop
            if I > Msg.Planned_Action_Signatures'First then
               Append (Result, ",");
            end if;
            Append (Result, """" & Ada.Strings.Unbounded.To_String (Msg.Planned_Action_Signatures (I)) & """");
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """compiled_bt_xml"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Compiled_Bt_Xml) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Decision_Record) return Decision_Record is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Decision_Record;
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
      if Has_Field (J, "session_id") then
         declare
            Val : constant JSON_Value := Get (J, "session_id");
            Str : constant String := Get (Val);
         begin
            Result.Session_Id := To_Unbounded_String (Str);
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
         Result.World_Version := Integer (Get_Long_Float (Get (J, "world_version")));
      end if;
      if Has_Field (J, "replan_count") then
         Result.Replan_Count := Integer (Get_Long_Float (Get (J, "replan_count")));
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
      if Has_Field (J, "planned_action_signatures") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "planned_action_signatures");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Planned_Action_Signatures := new Planned_Action_Signatures_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Str : constant String := Get (Elem);
                  begin
                     Result.Planned_Action_Signatures (I) := To_Unbounded_String (Str);
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
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Command_Result) return String is
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
      Append (Result, """command_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Command_Id) & """");
      Comma;
      Append (Result, """status"":" & """" & To_String (Msg.Status) & """");
      if Msg.Observed_Updates /= null then
         Comma;
         Append (Result, """observed_updates"":[");
         for I in Msg.Observed_Updates'Range loop
            if I > Msg.Observed_Updates'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Observed_Updates (I)));
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Command_Result) return Command_Result is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Command_Result;
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
      if Has_Field (J, "command_id") then
         declare
            Val : constant JSON_Value := Get (J, "command_id");
            Str : constant String := Get (Val);
         begin
            Result.Command_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Val : constant JSON_Value := Get (J, "status");
            Str : constant String := Get (Val);
         begin
            Result.Status := Command_Status_From_String (Str);
         end;
      end if;
      if Has_Field (J, "observed_updates") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "observed_updates");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Observed_Updates := new Observed_Updates_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Observed_Updates (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
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
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Dispatch_Result) return String is
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
      Append (Result, """dispatch_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Dispatch_Id) & """");
      Comma;
      Append (Result, """status"":" & """" & To_String (Msg.Status) & """");
      if Msg.Observed_Updates /= null then
         Comma;
         Append (Result, """observed_updates"":[");
         for I in Msg.Observed_Updates'Range loop
            if I > Msg.Observed_Updates'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Observed_Updates (I)));
         end loop;
         Append (Result, "]");
      end if;
      Comma;
      Append (Result, """source"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Source) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Dispatch_Result) return Dispatch_Result is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Dispatch_Result;
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
      if Has_Field (J, "dispatch_id") then
         declare
            Val : constant JSON_Value := Get (J, "dispatch_id");
            Str : constant String := Get (Val);
         begin
            Result.Dispatch_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "status") then
         declare
            Val : constant JSON_Value := Get (J, "status");
            Str : constant String := Get (Val);
         begin
            Result.Status := Command_Status_From_String (Str);
         end;
      end if;
      if Has_Field (J, "observed_updates") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "observed_updates");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Observed_Updates := new Observed_Updates_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Observed_Updates (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
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
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Session_Snapshot) return String is
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
      Append (Result, """session_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Session_Id) & """");
      Comma;
      Append (Result, """state"":" & """" & To_String (Msg.State) & """");
      Comma;
      Append (Result, """world_version"":" & Integer'Image (Msg.World_Version));
      Comma;
      Append (Result, """replan_count"":" & Integer'Image (Msg.Replan_Count));
      if Msg.Agent_States /= null then
         Comma;
         Append (Result, """agent_states"":[");
         for I in Msg.Agent_States'Range loop
            if I > Msg.Agent_States'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Agent_States (I)));
         end loop;
         Append (Result, "]");
      end if;
      if Msg.Outstanding_Commands /= null then
         Comma;
         Append (Result, """outstanding_commands"":[");
         for I in Msg.Outstanding_Commands'Range loop
            if I > Msg.Outstanding_Commands'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Outstanding_Commands (I)));
         end loop;
         Append (Result, "]");
      end if;
      if Msg.Outstanding_Goal_Dispatches /= null then
         Comma;
         Append (Result, """outstanding_goal_dispatches"":[");
         for I in Msg.Outstanding_Goal_Dispatches'Range loop
            if I > Msg.Outstanding_Goal_Dispatches'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Outstanding_Goal_Dispatches (I)));
         end loop;
         Append (Result, "]");
      end if;
      if Msg.Decision_History /= null then
         Comma;
         Append (Result, """decision_history"":[");
         for I in Msg.Decision_History'Range loop
            if I > Msg.Decision_History'First then
               Append (Result, ",");
            end if;
            Append (Result, To_Json (Msg.Decision_History (I)));
         end loop;
         Append (Result, "]");
      end if;
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Session_Snapshot) return Session_Snapshot is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Session_Snapshot;
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
      if Has_Field (J, "session_id") then
         declare
            Val : constant JSON_Value := Get (J, "session_id");
            Str : constant String := Get (Val);
         begin
            Result.Session_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "state") then
         declare
            Val : constant JSON_Value := Get (J, "state");
            Str : constant String := Get (Val);
         begin
            Result.State := Autonomy_Backend_State_From_String (Str);
         end;
      end if;
      if Has_Field (J, "world_version") then
         Result.World_Version := Integer (Get_Long_Float (Get (J, "world_version")));
      end if;
      if Has_Field (J, "replan_count") then
         Result.Replan_Count := Integer (Get_Long_Float (Get (J, "replan_count")));
      end if;
      if Has_Field (J, "agent_states") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "agent_states");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Agent_States := new Agent_States_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Agent_States (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "outstanding_commands") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "outstanding_commands");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Outstanding_Commands := new Outstanding_Commands_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Outstanding_Commands (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "outstanding_goal_dispatches") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "outstanding_goal_dispatches");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Outstanding_Goal_Dispatches := new Outstanding_Goal_Dispatches_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Outstanding_Goal_Dispatches (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      if Has_Field (J, "decision_history") then
         declare
            Arr_Val : constant JSON_Value := Get (J, "decision_history");
            Arr : constant JSON_Array := Get (Arr_Val);
            Len : constant Natural := Length (Arr);
         begin
            if Len > 0 then
               Result.Decision_History := new Decision_History_Array (1 .. Len);
               for I in 1 .. Len loop
                  declare
                     Elem : constant JSON_Value := Get (Arr, I);
                     Sub : constant String := Write (Elem);
                  begin
                     Result.Decision_History (I) := From_Json (Sub, null);
                  end;
               end loop;
            end if;
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Session_Step_Request) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """session_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Session_Id) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Session_Step_Request) return Session_Step_Request is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Session_Step_Request;
   begin
      if Has_Field (J, "session_id") then
         declare
            Val : constant JSON_Value := Get (J, "session_id");
            Str : constant String := Get (Val);
         begin
            Result.Session_Id := To_Unbounded_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

   function To_Json (Msg : Session_Stop_Request) return String is
      Result : Unbounded_String := To_Unbounded_String ("{");
      First  : Boolean := True;
      procedure Comma is
      begin
         if First then First := False;
         else Append (Result, ","); end if;
      end Comma;
   begin
      Comma;
      Append (Result, """session_id"":" & """" & Ada.Strings.Unbounded.To_String (Msg.Session_Id) & """");
      Comma;
      Append (Result, """mode"":" & """" & To_String (Msg.Mode) & """");
      Append (Result, "}");
      return To_String (Result);
   end To_Json;

   function From_Json (S : String; Tag : access Session_Stop_Request) return Session_Stop_Request is
      pragma Unreferenced (Tag);
      J      : constant JSON_Value := Read (S);
      Result : Session_Stop_Request;
   begin
      if Has_Field (J, "session_id") then
         declare
            Val : constant JSON_Value := Get (J, "session_id");
            Str : constant String := Get (Val);
         begin
            Result.Session_Id := To_Unbounded_String (Str);
         end;
      end if;
      if Has_Field (J, "mode") then
         declare
            Val : constant JSON_Value := Get (J, "mode");
            Str : constant String := Get (Val);
         begin
            Result.Mode := Stop_Mode_From_String (Str);
         end;
      end if;
      return Result;
   exception
      when others => return Result;
   end From_Json;

end Pyramid.Data_Model.Autonomy.Types_Codec;
