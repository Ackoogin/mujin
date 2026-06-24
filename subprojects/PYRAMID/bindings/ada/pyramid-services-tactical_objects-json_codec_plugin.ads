with Pcl_Plugins;

package Pyramid.Services.Tactical_Objects.Json_Codec_Plugin is
   function Plugin_Entry return Pcl_Plugins.Pcl_Codec_Const_Access;
   pragma Export (C, Plugin_Entry, "pcl_codec_plugin_entry");
end Pyramid.Services.Tactical_Objects.Json_Codec_Plugin;
