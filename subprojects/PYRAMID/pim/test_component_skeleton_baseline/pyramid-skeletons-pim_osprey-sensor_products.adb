with Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed; use Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed;
with Pyramid.Services.Pim_Osprey.Sensor_Products.Provided; use Pyramid.Services.Pim_Osprey.Sensor_Products.Provided;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types;
with Pyramid.Data_Model.Base.Types; use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types; use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types; use Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types;
with System;

package body Pyramid.Skeletons.Pim_Osprey.Sensor_Products is
   Bound : Boolean := False;
   Binding_State : access Handlers := null;

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
      Binding_State.Capability_Evidence_Information.all(Item);
   end Capability_Evidence_Information_Trampoline;

   procedure Bind
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Exec      : Pcl_Bindings.Pcl_Executor_Access;
      Binding   : Handlers) is
   begin
      if Bound then
         raise Program_Error with "component skeleton is already bound";
      end if;
      if Binding.Capability_Evidence_Information = null then
         raise Program_Error with "required handler slot Capability_Evidence_Information is null";
      end if;
      if Binding.Spr_Measured_Information_Information = null then
         raise Program_Error with "required handler slot Spr_Measured_Information_Information is null";
      end if;
      Binding_State := new Handlers'(Binding);
      Bound := True;
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.Authorisation_Dependency_Provider_Bind
        (Container, Exec, Binding.Authorisation_Dependency_Request);
      Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed.Subscribe_Pim_Osprey_Capability_Evidence_Information
        (Container, Capability_Evidence_Information_Trampoline'Access);
      --  capability_information publishes through Exec; no subscriber binding is required.
      --  spr_information_information publishes through Exec; no subscriber binding is required.
      --  spr_measured_information_information: fail closed because the current Ada
      --  facade does not expose a local-wrapper topic decoder.
      raise Program_Error with "spr_measured_information_information has no local-wrapper topic decoder";
      Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.SPR_Requirement_Provider_Bind
        (Container, Exec, Binding.Spr_Requirement_Request);
   end Bind;

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
             2 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.authorisation_dependency.entity", Pcl_Bindings.PCL_ENDPOINT_PUBLISHER)
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
             2 => Pcl_Process_Runtime.Make_Endpoint("pim_osprey.spr_requirement.entity", Pcl_Bindings.PCL_ENDPOINT_PUBLISHER)
           ))
      );
   end Deployment_Ports;

end Pyramid.Skeletons.Pim_Osprey.Sensor_Products;
