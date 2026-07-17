with Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed; use Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed;
with Pyramid.Services.Pim_Osprey.Sensor_Products.Provided; use Pyramid.Services.Pim_Osprey.Sensor_Products.Provided;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types;
with Pyramid.Data_Model.Base.Types; use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types; use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types; use Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types;
with System;

package body Pyramid.Skeletons.Pim_Osprey.Sensor_Products is
   Instance : access Component_Skeleton'Class := null;
   Instance_Executor : Pcl_Bindings.Pcl_Executor_Access := null;

   function Authorisation_Dependency_Cancel_Trampoline
     (Request : Identifier) return Ack is
   begin
      return Instance.On_Authorisation_Dependency_Cancel(Request);
   end Authorisation_Dependency_Cancel_Trampoline;

   function Authorisation_Dependency_Create_Trampoline
     (Request : Authorisation_Dependency_Service_Request) return Ack is
   begin
      return Instance.On_Authorisation_Dependency_Create(Request);
   end Authorisation_Dependency_Create_Trampoline;

   function Authorisation_Dependency_Update_Trampoline
     (Request : Authorisation_Dependency_Service_Requirement) return Ack is
   begin
      return Instance.On_Authorisation_Dependency_Update(Request);
   end Authorisation_Dependency_Update_Trampoline;

   procedure Capability_Evidence_Information_Trampoline
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention(C, Capability_Evidence_Information_Trampoline);

   procedure Capability_Evidence_Information_Trampoline
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address) is
      pragma Unreferenced(Self, User_Data);
      Item : constant Capabilities := Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed.Decode_Pim_Osprey_Capability_Evidence_Information(Msg);
   begin
      Instance.On_Capability_Evidence(Item);
   end Capability_Evidence_Information_Trampoline;

   function SPR_Requirement_Cancel_Trampoline
     (Request : Identifier) return Ack is
   begin
      return Instance.On_SPR_Requirement_Cancel(Request);
   end SPR_Requirement_Cancel_Trampoline;

   function SPR_Requirement_Create_Trampoline
     (Request : SPR_Requirement_Service_Request) return Ack is
   begin
      return Instance.On_SPR_Requirement_Create(Request);
   end SPR_Requirement_Create_Trampoline;

   function SPR_Requirement_Update_Trampoline
     (Request : SPR_Requirement_Service_Requirement) return Ack is
   begin
      return Instance.On_SPR_Requirement_Update(Request);
   end SPR_Requirement_Update_Trampoline;

   procedure Bind
     (Self      : access Component_Skeleton'Class;
      Container : Pcl_Bindings.Pcl_Container_Access;
      Exec      : Pcl_Bindings.Pcl_Executor_Access) is
   begin
      if Instance /= null then
         raise Program_Error with "component skeleton is already bound";
      end if;
      Instance := Self;
      Instance_Executor := Exec;
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.Authorisation_Dependency_Provider_Bind
        (Container, Exec,
         Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.Authorisation_Dependency_Interaction_Handlers'(
           On_Cancel => Authorisation_Dependency_Cancel_Trampoline'Access,
           On_Create => Authorisation_Dependency_Create_Trampoline'Access
         ));
      Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed.Subscribe_Pim_Osprey_Capability_Evidence_Information
        (Container, Capability_Evidence_Information_Trampoline'Access);
      --  capability_information publishes through Exec; no subscriber binding is required.
      --  spr_information_information publishes through Exec; no subscriber binding is required.
      --  spr_measured_information_information: the current Ada facade does not expose a local-wrapper topic decoder.
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.SPR_Requirement_Provider_Bind
        (Container, Exec,
         Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.SPR_Requirement_Interaction_Handlers'(
           On_Cancel => SPR_Requirement_Cancel_Trampoline'Access,
           On_Create => SPR_Requirement_Create_Trampoline'Access
         ));
   end Bind;

   procedure Send_Authorisation_Dependency_Transition
     (Self : in out Component_Skeleton; Item : Authorisation_Dependency_Service_Requirement) is
      pragma Unreferenced(Self);
   begin
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.Authorisation_Dependency_Send_Transition(Item);
   end Send_Authorisation_Dependency_Transition;

   procedure Publish_Capability
     (Self : in out Component_Skeleton; Item : Capabilities) is
      pragma Unreferenced(Self);
   begin
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.Publish_Pim_Osprey_Capability_Information(Instance_Executor, Item);
   end Publish_Capability;

   procedure Publish_SPR_Information
     (Self : in out Component_Skeleton; Item : SPR_Information_Service_Information) is
      pragma Unreferenced(Self);
   begin
      pragma Unreferenced(Item);
      raise Program_Error with "the current Ada facade does not expose this local-wrapper information topic";
   end Publish_SPR_Information;

   procedure Send_SPR_Requirement_Transition
     (Self : in out Component_Skeleton; Item : SPR_Requirement_Service_Requirement) is
      pragma Unreferenced(Self);
   begin
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.SPR_Requirement_Send_Transition(Item);
   end Send_SPR_Requirement_Transition;

   function Deployment_Ports return Pcl_Process_Runtime.Port_Array is
   begin
      return Pcl_Process_Runtime.Port_Array'(
        1 => Pcl_Process_Runtime.Make_Port
          ("authorisation_dependency_request",
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("authorisation_dependency.create", Pcl_Bindings.PCL_ENDPOINT_PROVIDED),
             2 => Pcl_Process_Runtime.Make_Endpoint("authorisation_dependency.update", Pcl_Bindings.PCL_ENDPOINT_PROVIDED),
             3 => Pcl_Process_Runtime.Make_Endpoint("authorisation_dependency.cancel", Pcl_Bindings.PCL_ENDPOINT_PROVIDED),
             4 => Pcl_Process_Runtime.Make_Endpoint("authorisation_dependency.read", Pcl_Bindings.PCL_ENDPOINT_STREAM_PROVIDED)
           ),
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.authorisation_dependency.request", Pcl_Bindings.PCL_ENDPOINT_SUBSCRIBER),
             2 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.authorisation_dependency.requirement", Pcl_Bindings.PCL_ENDPOINT_PUBLISHER)
           )),
        2 => Pcl_Process_Runtime.Make_Port
          ("capability_evidence_information",
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("capability_evidence.read", Pcl_Bindings.PCL_ENDPOINT_STREAM_CONSUMED)
           ),
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.capability_evidence.information", Pcl_Bindings.PCL_ENDPOINT_SUBSCRIBER)
           )),
        3 => Pcl_Process_Runtime.Make_Port
          ("capability_information",
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("capability.read", Pcl_Bindings.PCL_ENDPOINT_STREAM_PROVIDED)
           ),
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.capability.information", Pcl_Bindings.PCL_ENDPOINT_PUBLISHER)
           )),
        4 => Pcl_Process_Runtime.Make_Port
          ("spr_information_information",
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("spr_information.read", Pcl_Bindings.PCL_ENDPOINT_STREAM_PROVIDED)
           ),
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.spr_information.information", Pcl_Bindings.PCL_ENDPOINT_PUBLISHER)
           )),
        5 => Pcl_Process_Runtime.Make_Port
          ("spr_measured_information_information",
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("spr_measured_information.read", Pcl_Bindings.PCL_ENDPOINT_STREAM_CONSUMED)
           ),
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.spr_measured_information.information", Pcl_Bindings.PCL_ENDPOINT_SUBSCRIBER)
           )),
        6 => Pcl_Process_Runtime.Make_Port
          ("spr_requirement_request",
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("spr_requirement.create", Pcl_Bindings.PCL_ENDPOINT_PROVIDED),
             2 => Pcl_Process_Runtime.Make_Endpoint("spr_requirement.update", Pcl_Bindings.PCL_ENDPOINT_PROVIDED),
             3 => Pcl_Process_Runtime.Make_Endpoint("spr_requirement.cancel", Pcl_Bindings.PCL_ENDPOINT_PROVIDED),
             4 => Pcl_Process_Runtime.Make_Endpoint("spr_requirement.read", Pcl_Bindings.PCL_ENDPOINT_STREAM_PROVIDED)
           ),
           Pcl_Process_Runtime.Endpoint_Array'(
             1 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.spr_requirement.request", Pcl_Bindings.PCL_ENDPOINT_SUBSCRIBER),
             2 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.spr_requirement.requirement", Pcl_Bindings.PCL_ENDPOINT_PUBLISHER)
           ))
      );
   end Deployment_Ports;

end Pyramid.Skeletons.Pim_Osprey.Sensor_Products;
