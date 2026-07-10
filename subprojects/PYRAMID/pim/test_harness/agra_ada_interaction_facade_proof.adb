--  Ada interaction-facade runtime proof (WS-F/F1) -- single-process,
--  single-executor equivalent of the C++ facade's
--  test_pcl_generated_interaction_facade.cpp: proves the generated
--  MA_Action_* client (Submit_*/Transitions) and provider
--  (Interaction_Handlers/Provider_Bind/Send_Transition) dispatch real
--  requests end to end under both the RPC and pub/sub realizations,
--  selected purely by MA_Action_Configure_Interaction_Binding's Config_Json
--  argument -- no other code differs between the two runs.
--
--  Not (yet) a cross-process/real-transport proof the way
--  agra_seam_interchange_test.cpp is for C++ -- see F1's TODO.md entry for
--  what's covered and what's a follow-up (remote routing, the D4
--  late-command-republish nuance, CI/.gpr wiring).
--
--  Build + run: pim/test_harness/build_agra_ada_interaction_facade_proof.sh

with Ada.Text_IO;
with Ada.Command_Line;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pcl_Bindings;
with Pcl_Plugins;

with Pyramid.Data_Model.Base.Types; use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Agra.Types; use Pyramid.Data_Model.Agra.Types;
with Pyramid.Data_Model.Common.Types; use Pyramid.Data_Model.Common.Types;
with Pyramid.Components.Agra.Mission_Autonomy.Services.Provided.Types;
with Pyramid.Components.Agra.C2_Station.Services.Consumed.Types;
with Pyramid.Services.Agra.Mission_Autonomy.Provided;
with Pyramid.Services.Agra.C2_Station.Consumed;

procedure Agra_Ada_Interaction_Facade_Proof is
   package Prov renames Pyramid.Services.Agra.Mission_Autonomy.Provided;
   package Cons renames Pyramid.Services.Agra.C2_Station.Consumed;
   package Prov_T renames Pyramid.Components.Agra.Mission_Autonomy.Services.Provided.Types;
   package Cons_T renames Pyramid.Components.Agra.C2_Station.Services.Consumed.Types;

   use type Pcl_Bindings.Pcl_Status;

   procedure Assert (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with "FAILED: " & Message;
      end if;
   end Assert;

   procedure Log (Message : String) is
   begin
      Ada.Text_IO.Put_Line (Message);
   end Log;

   function Load_Codec (Path : String) return Boolean is
      Path_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Path);
      Handle : aliased System.Address := System.Null_Address;
      Status : Pcl_Bindings.Pcl_Status;
   begin
      Status := Pcl_Plugins.Pcl_Plugin_Load_Codec
        (Path_C, Interfaces.C.Strings.Null_Ptr,
         Pcl_Plugins.Pcl_Codec_Registry_Default, Handle'Access);
      Interfaces.C.Strings.Free (Path_C);
      return Status = Pcl_Bindings.PCL_OK;
   end Load_Codec;

   Exec               : Pcl_Bindings.Pcl_Executor_Access;
   Provider_Container : Pcl_Bindings.Pcl_Container_Access;
   Client_Container   : Pcl_Bindings.Pcl_Container_Access;

   Create_Count : Natural := 0;
   Last_Create_Id : Unbounded_String := Null_Unbounded_String;

   function On_Create (Request : Prov_T.Ma_Action_Service_Request) return Ack is
   begin
      Create_Count := Create_Count + 1;
      if Request.Has_Ma_Action then
         Last_Create_Id := Request.Ma_Action.Id;
      end if;
      return (Success => True, Identifier => Last_Create_Id);
   end On_Create;

   function On_Update (Request : Prov_T.Ma_Action_Service_Requirement) return Ack is
      pragma Unreferenced (Request);
   begin
      return (Success => True, Identifier => Null_Unbounded_String);
   end On_Update;

   function On_Cancel (Request : Identifier) return Ack is
   begin
      return (Success => True, Identifier => Request);
   end On_Cancel;

   function Provider_On_Configure
     (Self : Pcl_Bindings.Pcl_Container_Access; User_Data : System.Address)
      return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Provider_On_Configure);

   function Client_On_Configure
     (Self : Pcl_Bindings.Pcl_Container_Access; User_Data : System.Address)
      return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Client_On_Configure);

   function Provider_On_Configure
     (Self : Pcl_Bindings.Pcl_Container_Access; User_Data : System.Address)
      return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, User_Data);
      Handlers : constant Prov.Ma_Action_Interaction_Handlers :=
        (On_Update => On_Update'Unrestricted_Access,
         On_Cancel => On_Cancel'Unrestricted_Access,
         On_Create => On_Create'Unrestricted_Access);
   begin
      Prov.Ma_Action_Provider_Bind (Provider_Container, Exec, Handlers);
      return Pcl_Bindings.PCL_OK;
   end Provider_On_Configure;

   function Client_On_Configure
     (Self : Pcl_Bindings.Pcl_Container_Access; User_Data : System.Address)
      return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, User_Data);
   begin
      Cons.Ma_Action_Client_Bind (Client_Container, Exec);
      return Pcl_Bindings.PCL_OK;
   end Client_On_Configure;

   Transition_Received : Natural := 0;
   Last_Transition_Id  : Unbounded_String := Null_Unbounded_String;

   procedure On_Transition (Item : Cons_T.Ma_Action_Service_Requirement) is
   begin
      Transition_Received := Transition_Received + 1;
      if Item.Has_Ma_Action_Status then
         Last_Transition_Id := Item.Ma_Action_Status.Id;
      end if;
   end On_Transition;

   procedure Run_Realization (Binding_Name : String) is
      Provider_Callbacks : aliased Pcl_Bindings.Pcl_Callbacks :=
        (On_Configure  => Provider_On_Configure'Unrestricted_Access,
         On_Activate   => null,
         On_Deactivate => null,
         On_Cleanup    => null,
         On_Shutdown   => null,
         On_Tick       => null);
      Client_Callbacks : aliased Pcl_Bindings.Pcl_Callbacks :=
        (On_Configure  => Client_On_Configure'Unrestricted_Access,
         On_Activate   => null,
         On_Deactivate => null,
         On_Cleanup    => null,
         On_Shutdown   => null,
         On_Tick       => null);
      Name_C : Interfaces.C.Strings.chars_ptr;
      Status : Pcl_Bindings.Pcl_Status;
   begin
      Log ("=== realization: " & Binding_Name & " ===");
      Create_Count := 0;
      Transition_Received := 0;
      Last_Create_Id := Null_Unbounded_String;
      Last_Transition_Id := Null_Unbounded_String;

      Name_C := Interfaces.C.Strings.New_String ("agra_provider");
      Provider_Container := Pcl_Bindings.Create_Container
        (Name_C, Provider_Callbacks'Access, System.Null_Address);
      Interfaces.C.Strings.Free (Name_C);

      Name_C := Interfaces.C.Strings.New_String ("agra_client");
      Client_Container := Pcl_Bindings.Create_Container
        (Name_C, Client_Callbacks'Access, System.Null_Address);
      Interfaces.C.Strings.Free (Name_C);

      Status := Pcl_Bindings.Configure (Provider_Container);
      Assert (Status = Pcl_Bindings.PCL_OK, "provider configure");
      Status := Pcl_Bindings.Activate (Provider_Container);
      Assert (Status = Pcl_Bindings.PCL_OK, "provider activate");
      Status := Pcl_Bindings.Add_Container (Exec, Provider_Container);
      Assert (Status = Pcl_Bindings.PCL_OK, "provider add to executor");

      Status := Pcl_Bindings.Configure (Client_Container);
      Assert (Status = Pcl_Bindings.PCL_OK, "client configure");
      Status := Pcl_Bindings.Activate (Client_Container);
      Assert (Status = Pcl_Bindings.PCL_OK, "client activate");
      Status := Pcl_Bindings.Add_Container (Exec, Client_Container);
      Assert (Status = Pcl_Bindings.PCL_OK, "client add to executor");

      declare
         Config_Json : constant String := "{""binding"":""" & Binding_Name & """}";
      begin
         Prov.Ma_Action_Configure_Interaction_Binding (Config_Json);
         Cons.Ma_Action_Configure_Interaction_Binding (Config_Json);
      end;

      declare
         Query_Filter : Query := (Id => null, Has_One_Shot => True, One_Shot => True);
         Ids : aliased Identifier_Array := (1 => To_Unbounded_String ("action-1"));
      begin
         Query_Filter.Id := Ids'Unrestricted_Access;
         Cons.Ma_Action_Transitions (Query_Filter, On_Transition'Unrestricted_Access);
      end;

      declare
         Command : Cons_T.Ma_Action_Service_Request := (others => <>);
         Action  : Pyramid.Data_Model.Agra.Types.Ma_Action;
         Accepted : Boolean;
         Status_Code : Interfaces.C.int;
         Has_Ack : Boolean;
         Ack_Value : Ack;
      begin
         Action.Id := To_Unbounded_String ("action-1");
         Command.Has_Ma_Action := True;
         Command.Ma_Action := Action;

         Cons.Ma_Action_Submit_Create
           (Command, Accepted, Status_Code, Has_Ack, Ack_Value);

         Assert (Accepted, "submit create accepted (" & Binding_Name & ")");
         Assert (Create_Count = 1, "provider observed create exactly once");
         Assert (Last_Create_Id = "action-1", "provider saw correct id");

         if Binding_Name = "rpc" then
            Assert (Has_Ack, "rpc realization synthesizes a remote ack");
            Assert (Ack_Value.Success, "rpc remote ack reports success");
         else
            Assert (not Has_Ack, "D3: pub/sub realization never synthesizes an ack");
         end if;
      end;

      declare
         Frame : Prov_T.Ma_Action_Service_Requirement := (others => <>);
         Status_Field : Requirement;
      begin
         Status_Field.Id := To_Unbounded_String ("action-1");
         Frame.Has_Ma_Action_Status := True;
         Frame.Ma_Action_Status := Status_Field;
         Prov.Ma_Action_Send_Transition (Frame);
      end;

      for I in 1 .. 5 loop
         declare
            Spin_Status : Pcl_Bindings.Pcl_Status;
            pragma Unreferenced (Spin_Status);
         begin
            Spin_Status := Pcl_Bindings.Spin_Once (Exec, 1);
         end;
         exit when Transition_Received > 0;
      end loop;

      Assert (Transition_Received = 1,
              "client received exactly one transition (" & Binding_Name & ")");
      Assert (Last_Transition_Id = "action-1", "client saw correct transition id");

      Status := Pcl_Bindings.Remove_Container (Exec, Provider_Container);
      Status := Pcl_Bindings.Remove_Container (Exec, Client_Container);
      Pcl_Bindings.Destroy_Container (Provider_Container);
      Pcl_Bindings.Destroy_Container (Client_Container);

      Log ("PASS: " & Binding_Name);
   end Run_Realization;

begin
   if Ada.Command_Line.Argument_Count /= 2 then
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error,
         "usage: agra_ada_interaction_facade_proof " &
         "<mission_autonomy_json_codec.so> <c2_station_json_codec.so>");
      Ada.Command_Line.Set_Exit_Status (2);
      return;
   end if;

   Assert (Load_Codec (Ada.Command_Line.Argument (1)),
           "load mission_autonomy codec plugin");
   Assert (Load_Codec (Ada.Command_Line.Argument (2)),
           "load c2_station codec plugin");

   Exec := Pcl_Bindings.Create_Executor;

   Run_Realization ("rpc");
   Run_Realization ("pubsub");

   Pcl_Bindings.Destroy_Executor (Exec);
   Log ("ALL ADA INTERACTION FACADE RUNTIME TESTS PASSED");
end Agra_Ada_Interaction_Facade_Proof;
