--  Auto-generated component skeleton. Regenerated on every run.
with Pyramid.Services.Pim_Osprey.Sensor_Products.Consumed;
with Pyramid.Services.Pim_Osprey.Sensor_Products.Provided;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Consumed.Types;
with Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types; use Pyramid.Components.Pim_Osprey.Sensor_Products.Services.Provided.Types;
with Pyramid.Data_Model.Base.Types; use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types; use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types; use Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types;
with Pcl_Bindings;
with Pcl_Process_Runtime;

package Pyramid.Skeletons.Pim_Osprey.Sensor_Products is
   type Handlers is record
      Authorisation_Dependency_Request :
        Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.Authorisation_Dependency_Interaction_Handlers;
      Capability_Evidence_Information :
        access procedure
          (Item : Capabilities);
      Spr_Measured_Information_Information :
        access procedure
          (Item : SPR_Measured_Information_Service_Information);
      Spr_Requirement_Request :
        Pyramid.Services.Pim_Osprey.Sensor_Products.Provided.SPR_Requirement_Interaction_Handlers;
   end record;

   procedure Bind
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Exec      : Pcl_Bindings.Pcl_Executor_Access;
      Binding   : Handlers);

   function Deployment_Ports return Pcl_Process_Runtime.Port_Array;

end Pyramid.Skeletons.Pim_Osprey.Sensor_Products;
