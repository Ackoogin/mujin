--  Auto-generated ROS2 transport constants — do not edit
--  Backend: ros2 | Package: Pyramid.Components.Tactical_objects.ROS2_Transport

package Pyramid.Components.Tactical_objects.ROS2_Transport is

   Content_Type : constant String := "application/ros2";

   Tactical_Object_Create_Tactical_Object_Service : constant String := "/pyramid/service/tactical_object/create_tactical_object";

   Tactical_Object_Read_Tactical_Object_Open_Service : constant String := "/pyramid/stream/tactical_object/read_tactical_object/open";
   Tactical_Object_Read_Tactical_Object_Frame_Topic : constant String := "/pyramid/stream/tactical_object/read_tactical_object/frames";
   Tactical_Object_Read_Tactical_Object_Cancel_Topic : constant String := "/pyramid/stream/tactical_object/read_tactical_object/cancel";

   Tactical_Object_Update_Tactical_Object_Service : constant String := "/pyramid/service/tactical_object/update_tactical_object";

   Tactical_Object_Delete_Tactical_Object_Service : constant String := "/pyramid/service/tactical_object/delete_tactical_object";

   Zone_Create_Zone_Service : constant String := "/pyramid/service/zone/create_zone";

   Zone_Read_Zone_Open_Service : constant String := "/pyramid/stream/zone/read_zone/open";
   Zone_Read_Zone_Frame_Topic : constant String := "/pyramid/stream/zone/read_zone/frames";
   Zone_Read_Zone_Cancel_Topic : constant String := "/pyramid/stream/zone/read_zone/cancel";

   Zone_Update_Zone_Service : constant String := "/pyramid/service/zone/update_zone";

   Zone_Delete_Zone_Service : constant String := "/pyramid/service/zone/delete_zone";

   Observation_Ingress_Create_Observation_Service : constant String := "/pyramid/service/observation_ingress/create_observation";

end Pyramid.Components.Tactical_objects.ROS2_Transport;
