--  Showcase: the Ada interaction facade (WS-F/F2c) -- the Ada analogue of
--  examples/cpp/agra_interaction_facade_example.cpp -- against the A-GRA
--  example contract (pim/agra_example/, grammar-conforming Request-shape
--  MAAction_Service).
--
--  Both provider and client are built entirely from the generated
--  interaction facade: the provider composes an Interaction_Handlers
--  record + Provider_Bind + Send_Transition (Pyramid.Services.Agra.
--  Mission_Autonomy.Provided); the client composes Client_Bind +
--  Configure_Interaction_Binding + Submit_*/Transitions (Pyramid.Services.
--  Agra.C2_Station.Consumed). Nothing in this file names an RPC or pub/sub
--  primitive directly -- that is exactly the point of the facade.
--
--  Demonstrated flow:
--    1. Build one provider container and one client container on a single
--       pcl::Executor (local dispatch only -- see the "Scope" note below).
--    2. Pick the entity leg's realization -- RPC or pub/sub -- from a
--       manifest-style JSON string, selectable on the command line
--       (--binding=rpc (default) or --binding=pubsub). Same component code
--       either way: only Configure_Interaction_Binding's argument changes.
--    3. Submit a Create command and print whether a remote ack came back
--       (only ever true under RPC -- D3: pub/sub never synthesizes one).
--    4. Observe the transition the provider publishes back through
--       Send_Transition, delivered to the client's Transitions callback.
--
--  Scope: this example's Client_Bind/Provider_Bind route everything
--  PCL_ROUTE_LOCAL (single-process dispatch) -- the facade does not yet
--  expose remote/cross-process routing (see doc/todo/PYRAMID/TODO.md
--  WS-F F1). For the equivalent proof used to verify the facade itself,
--  see pim/test_harness/agra_ada_interaction_facade_proof.adb.
--
--  Build + run: examples/ada/build_agra_interaction_facade_example.sh
--  [--binding=rpc|pubsub]

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

procedure Agra_Interaction_Facade_Example is
   package Prov renames Pyramid.Services.Agra.Mission_Autonomy.Provided;
   package Cons renames Pyramid.Services.Agra.C2_Station.Consumed;
   package Prov_T renames Pyramid.Components.Agra.Mission_Autonomy.Services.Provided.Types;
   package Cons_T renames Pyramid.Components.Agra.C2_Station.Services.Consumed.Types;

   use type Pcl_Bindings.Pcl_Status;

   procedure Die (Message : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error, "error: " & Message);
      Ada.Command_Line.Set_Exit_Status (1);
      raise Program_Error with Message;
   end Die;

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

   --  [provider] business logic -- a real component would track mission
   --  state here instead of just logging.
   function On_Create (Request : Prov_T.Ma_Action_Service_Request) return Ack is
      Id : Unbounded_String := Null_Unbounded_String;
   begin
      if Request.Has_Ma_Action then
         Id := Request.Ma_Action.Id;
      end if;
      Ada.Text_IO.Put_Line ("[provider] Create id=" & To_String (Id));
      return (Success => True, Identifier => Id);
   end On_Create;

   function On_Update (Request : Prov_T.Ma_Action_Service_Entity) return Ack is
      pragma Unreferenced (Request);
   begin
      return (Success => True, Identifier => Null_Unbounded_String);
   end On_Update;

   function On_Cancel (Request : Identifier) return Ack is
   begin
      Ada.Text_IO.Put_Line ("[provider] Cancel id=" & To_String (Request));
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

   --  [client] observes the transition the provider later publishes back.
   procedure On_Transition (Item : Cons_T.Ma_Action_Service_Entity) is
   begin
      if Item.Has_Ma_Action_Status then
         Ada.Text_IO.Put_Line
           ("[client] transition frame id=" & To_String (Item.Ma_Action_Status.Id));
      end if;
   end On_Transition;

   Binding : Unbounded_String := To_Unbounded_String ("rpc");
   Provider_Callbacks : aliased Pcl_Bindings.Pcl_Callbacks :=
     (On_Configure  => Provider_On_Configure'Unrestricted_Access,
      On_Activate   => null, On_Deactivate => null,
      On_Cleanup    => null, On_Shutdown => null, On_Tick => null);
   Client_Callbacks : aliased Pcl_Bindings.Pcl_Callbacks :=
     (On_Configure  => Client_On_Configure'Unrestricted_Access,
      On_Activate   => null, On_Deactivate => null,
      On_Cleanup    => null, On_Shutdown => null, On_Tick => null);
   Name_C : Interfaces.C.Strings.chars_ptr;
   Status : Pcl_Bindings.Pcl_Status;
begin
   --  -- Parse CLI: --binding=rpc|pubsub, plus the two codec plugin paths --
   if Ada.Command_Line.Argument_Count < 2 then
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error,
         "usage: agra_interaction_facade_example " &
         "<mission_autonomy_json_codec.so> <c2_station_json_codec.so> " &
         "[--binding=rpc|pubsub]");
      Ada.Command_Line.Set_Exit_Status (2);
      return;
   end if;
   for I in 3 .. Ada.Command_Line.Argument_Count loop
      declare
         Arg : constant String := Ada.Command_Line.Argument (I);
      begin
         if Arg'Length > 10 and then Arg (Arg'First .. Arg'First + 9) = "--binding=" then
            Binding := To_Unbounded_String (Arg (Arg'First + 10 .. Arg'Last));
         end if;
      end;
   end loop;
   if To_String (Binding) /= "rpc" and then To_String (Binding) /= "pubsub" then
      Die ("--binding must be rpc or pubsub");
   end if;

   if not Load_Codec (Ada.Command_Line.Argument (1)) then
      Die ("failed to load mission_autonomy codec plugin");
   end if;
   if not Load_Codec (Ada.Command_Line.Argument (2)) then
      Die ("failed to load c2_station codec plugin");
   end if;

   Ada.Text_IO.Put_Line ("[client] realization: " & To_String (Binding));

   --  -- Bring up one executor hosting a provider container and a client
   --     container -- Bind happens inside each container's on_configure(),
   --     matching PCL's port-lifecycle rules. --
   Exec := Pcl_Bindings.Create_Executor;

   Name_C := Interfaces.C.Strings.New_String ("agra_provider");
   Provider_Container := Pcl_Bindings.Create_Container
     (Name_C, Provider_Callbacks'Access, System.Null_Address);
   Interfaces.C.Strings.Free (Name_C);

   Name_C := Interfaces.C.Strings.New_String ("agra_client");
   Client_Container := Pcl_Bindings.Create_Container
     (Name_C, Client_Callbacks'Access, System.Null_Address);
   Interfaces.C.Strings.Free (Name_C);

   Status := Pcl_Bindings.Configure (Provider_Container);
   if Status /= Pcl_Bindings.PCL_OK then Die ("provider configure failed"); end if;
   Status := Pcl_Bindings.Activate (Provider_Container);
   if Status /= Pcl_Bindings.PCL_OK then Die ("provider activate failed"); end if;
   Status := Pcl_Bindings.Add_Container (Exec, Provider_Container);
   if Status /= Pcl_Bindings.PCL_OK then Die ("provider add-to-executor failed"); end if;

   Status := Pcl_Bindings.Configure (Client_Container);
   if Status /= Pcl_Bindings.PCL_OK then Die ("client configure failed"); end if;
   Status := Pcl_Bindings.Activate (Client_Container);
   if Status /= Pcl_Bindings.PCL_OK then Die ("client activate failed"); end if;
   Status := Pcl_Bindings.Add_Container (Exec, Client_Container);
   if Status /= Pcl_Bindings.PCL_OK then Die ("client add-to-executor failed"); end if;

   --  -- Realization switch by manifest: this is the only thing that
   --     changes between "RPC" and "pub/sub" -- everything above and
   --     below is identical component code either way. --
   declare
      Config_Json : constant String :=
        "{""binding"":""" & To_String (Binding) & """}";
   begin
      Prov.Ma_Action_Configure_Interaction_Binding (Config_Json);
      Cons.Ma_Action_Configure_Interaction_Binding (Config_Json);
   end;

   --  -- transitions() -- subscribe before submitting, so the frame
   --     published below (once the Create has been handled) is delivered
   --     rather than missed. --
   declare
      Query_Filter : Query := (Id => null, Has_One_Shot => True, One_Shot => True);
      Ids : aliased Identifier_Array := (1 => To_Unbounded_String ("example-action-1"));
   begin
      Query_Filter.Id := Ids'Unrestricted_Access;
      Cons.Ma_Action_Transitions (Query_Filter, On_Transition'Unrestricted_Access);
   end;

   --  -- submit() -- Create. --
   declare
      Command  : Cons_T.Ma_Action_Service_Request := (others => <>);
      Action   : Pyramid.Data_Model.Agra.Types.Ma_Action;
      Accepted : Boolean;
      Status_Code : Interfaces.C.int;
      Has_Ack  : Boolean;
      Ack_Value : Ack;
   begin
      Action.Id := To_Unbounded_String ("example-action-1");
      Command.Has_Ma_Action := True;
      Command.Ma_Action := Action;

      Cons.Ma_Action_Submit_Create
        (Command, Accepted, Status_Code, Has_Ack, Ack_Value);

      Ada.Text_IO.Put_Line
        ("[client] submit accepted=" & Accepted'Image &
         " status=" & Status_Code'Image &
         " remote_ack=" &
         (if Has_Ack then Ack_Value.Success'Image else "<none>"));
   end;

   --  -- Publish the transition: business logic (here, main line -- standing
   --     in for whatever drives the mission autonomy state machine) uses
   --     Send_Transition, owned internally by the provider facade. --
   declare
      Frame : Prov_T.Ma_Action_Service_Entity := (others => <>);
      Status_Field : Requirement;
   begin
      Status_Field.Id := To_Unbounded_String ("example-action-1");
      Frame.Has_Ma_Action_Status := True;
      Frame.Ma_Action_Status := Status_Field;
      Prov.Ma_Action_Send_Transition (Frame);
   end;

   Status := Pcl_Bindings.Remove_Container (Exec, Provider_Container);
   Status := Pcl_Bindings.Remove_Container (Exec, Client_Container);
   Pcl_Bindings.Destroy_Container (Provider_Container);
   Pcl_Bindings.Destroy_Container (Client_Container);
   Pcl_Bindings.Destroy_Executor (Exec);
end Agra_Interaction_Facade_Example;
