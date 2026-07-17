--  Auto-generated component skeleton. Regenerated on every run.
with Pyramid.Services.Pim_Osprey.Sensors.Consumed;
with Pyramid.Services.Pim_Osprey.Sensors.Provided;
with Pyramid.Components.Pim_Osprey.Sensors.Services.Consumed.Types; use Pyramid.Components.Pim_Osprey.Sensors.Services.Consumed.Types;
with Pyramid.Components.Pim_Osprey.Sensors.Services.Provided.Types; use Pyramid.Components.Pim_Osprey.Sensors.Services.Provided.Types;
with Pyramid.Data_Model.Base.Types; use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types; use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types; use Pyramid.Data_Model.Generic_Pim.Generic_Pkg.Types;
with Pcl_Bindings;
with Pcl_Process_Runtime;

package Pyramid.Skeletons.Pim_Osprey.Sensors is
   type Handlers is record
      Authorisation_Dependency_Request :
        Pyramid.Services.Pim_Osprey.Sensors.Provided.Authorisation_Dependency_Interaction_Handlers;
      Capability_Evidence_Information :
        access procedure
          (Item : Capabilities);
      Sen_Requirement_Request :
        Pyramid.Services.Pim_Osprey.Sensors.Provided.SEN_Requirement_Interaction_Handlers;
   end record;

   procedure Bind
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Exec      : Pcl_Bindings.Pcl_Executor_Access;
      Binding   : Handlers);

   function Deployment_Ports return Pcl_Process_Runtime.Port_Array;

end Pyramid.Skeletons.Pim_Osprey.Sensors;
