--  Auto-generated component skeleton. Regenerated on every run.
with Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed;
with Pyramid.Services.Pim_Osprey.Sensor_Products.Provided;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types;
with Pyramid.Data_Model.Base.Types; use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types; use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types; use Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types;
with Interfaces.C;
with Pcl_Bindings;
with Pcl_Process_Runtime;

package Pyramid.Skeletons.Pim_Osprey.Sensor_Products is
   type Component_Skeleton is abstract tagged limited private;

   procedure Bind
     (Self      : access Component_Skeleton'Class;
      Container : Pcl_Bindings.Pcl_Container_Access;
      Exec      : Pcl_Bindings.Pcl_Executor_Access);

   function On_Authorisation_Dependency_Cancel
     (Self    : in out Component_Skeleton;
      Request : Identifier) return Ack is abstract;

   function On_Authorisation_Dependency_Create
     (Self    : in out Component_Skeleton;
      Request : Authorisation_Dependency_Service_Request) return Ack is abstract;

   function On_Authorisation_Dependency_Update
     (Self    : in out Component_Skeleton;
      Request : Authorisation_Dependency_Service_Requirement) return Ack is abstract;

   procedure Send_Authorisation_Dependency_Transition
     (Self : in out Component_Skeleton; Item : Authorisation_Dependency_Service_Requirement);

   procedure On_Capability_Evidence
     (Self : in out Component_Skeleton;
      Item : Capabilities) is abstract;

   procedure Publish_Capability
     (Self : in out Component_Skeleton;
      Item : Capabilities);

   procedure Publish_SPR_Information
     (Self : in out Component_Skeleton;
      Item : SPR_Information_Service_Information);

   procedure On_SPR_Measured_Information
     (Self : in out Component_Skeleton;
      Item : SPR_Measured_Information_Service_Information) is abstract;

   function On_SPR_Requirement_Cancel
     (Self    : in out Component_Skeleton;
      Request : Identifier) return Ack is abstract;

   function On_SPR_Requirement_Create
     (Self    : in out Component_Skeleton;
      Request : SPR_Requirement_Service_Request) return Ack is abstract;

   function On_SPR_Requirement_Update
     (Self    : in out Component_Skeleton;
      Request : SPR_Requirement_Service_Requirement) return Ack is abstract;

   procedure Send_SPR_Requirement_Transition
     (Self : in out Component_Skeleton; Item : SPR_Requirement_Service_Requirement);

   procedure On_Tick
     (Self       : in out Component_Skeleton;
      Dt_Seconds : Interfaces.C.double) is null;

   function Deployment_Ports return Pcl_Process_Runtime.Port_Array;

private
   type Component_Skeleton is abstract tagged limited null record;
end Pyramid.Skeletons.Pim_Osprey.Sensor_Products;
