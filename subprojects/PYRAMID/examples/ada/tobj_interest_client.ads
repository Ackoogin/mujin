--  tobj_interest_client.ads
--
--  Reusable component: subscribes to standard.entity_matches, invokes
--  object_of_interest.create_requirement via the generated service bindings.
--
--  Architecture:
--    component logic (this)
--    -> proto-native tactical codecs
--    -> service binding
--    -> PCL

with Pcl_Bindings;
with Ada.Strings.Unbounded;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with System;

package Tobj_Interest_Client is

   Content_Type : Ada.Strings.Unbounded.Unbounded_String :=
     Ada.Strings.Unbounded.To_Unbounded_String
       (Pyramid.Services.Tactical_Objects.Provided.Json_Content_Type);

   --  Component state (written by callbacks on the executor thread)
   Matches_Received     : Natural := 0;
   Found_Hostile_Entity : Boolean := False;
   Svc_Response_Ready   : Boolean := False;
   Interest_Id_Received : Boolean := False;

   --  PCL lifecycle callback — subscribes to standard.entity_matches
   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, On_Configure);

   --  Subscriber callback for standard.entity_matches (canonical Object_Match array)
   procedure On_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Entity_Matches);

   --  Service response callback for create_requirement
   procedure On_Create_Requirement_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Create_Requirement_Response);

   --  Send a create_requirement request.
   --  Takes typed enum values. Serialisation is handled by the generated codecs.
   procedure Send_Create_Requirement
     (Exec        : Pcl_Bindings.Pcl_Executor_Access;
      Policy      : Pyramid.Data_Model.Common.Types.Data_Policy;
      Identity    : Pyramid.Data_Model.Common.Types.Standard_Identity;
      Dimension   : Pyramid.Data_Model.Common.Types.Battle_Dimension :=
                     Pyramid.Data_Model.Common.Types.Dimension_Unspecified;
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0);

   --  Reset state between runs
   procedure Reset;

end Tobj_Interest_Client;
