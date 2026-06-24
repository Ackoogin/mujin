with Ada.Command_Line;
with Ada.Exceptions;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Text_IO;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Plugins;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with System;

procedure Test_Ada_JSON_Codec_Roundtrip is
   use type Interfaces.C.unsigned;
   use type Interfaces.C.Strings.chars_ptr;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Plugins.Pcl_Codec_Const_Access;
   use type Pcl_Plugins.Pcl_Codec_Decode_Access;
   use type Pcl_Plugins.Pcl_Codec_Encode_Access;
   use type Pcl_Plugins.Pcl_Codec_Free_Msg_Access;
   use type Source_Array_Acc;
   use type System.Address;

   procedure Log_Line (Message : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error, Message);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log_Line;

   procedure Assert (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Assert;

   function Same (Left, Right : Long_Float) return Boolean is
   begin
      return abs (Left - Right) < 0.000_000_001;
   end Same;

   function Source_Equals
     (Left  : Source_Array_Acc;
      Right : Source_Array_Acc) return Boolean
   is
   begin
      if Left = null or else Right = null then
         return Left = null and then Right = null;
      end if;
      if Left.all'Length /= Right.all'Length then
         return False;
      end if;
      for I in Left.all'Range loop
         if Left.all (I) /= Right.all (I) then
            return False;
         end if;
      end loop;
      return True;
   end Source_Equals;

   procedure Assert_Equal (Expected, Actual : Object_Detail) is
   begin
      Assert (Same (Expected.Update_Time, Actual.Update_Time),
              "update_time mismatch");
      Assert (Expected.Id = Actual.Id, "id mismatch");
      Assert (Expected.Entity_Source = Actual.Entity_Source,
              "entity_source mismatch");
      Assert (Source_Equals (Expected.Source, Actual.Source),
              "source mismatch");
      Assert (Same (Expected.Position.Latitude, Actual.Position.Latitude),
              "latitude mismatch");
      Assert (Same (Expected.Position.Longitude, Actual.Position.Longitude),
              "longitude mismatch");
      Assert (Same (Expected.Creation_Time, Actual.Creation_Time),
              "creation_time mismatch");
      Assert (Same (Expected.Quality, Actual.Quality), "quality mismatch");
      Assert (Same (Expected.Course, Actual.Course), "course mismatch");
      Assert (Same (Expected.Speed, Actual.Speed), "speed mismatch");
      Assert (Same (Expected.Length, Actual.Length), "length mismatch");
      Assert (Expected.Identity = Actual.Identity, "identity mismatch");
      Assert (Expected.Dimension = Actual.Dimension, "dimension mismatch");
   end Assert_Equal;

   procedure Run (Plugin_Path : String) is
      Registry : System.Address := System.Null_Address;
      Handle   : aliased System.Address := System.Null_Address;
      Codec    : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Out_Msg  : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Original : aliased Object_Detail;
      Decoded  : aliased Object_Detail;

      Path_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Plugin_Path);
      Json_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("application/json");
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("ObjectDetail");

      procedure Cleanup is
      begin
         if Out_Msg.Data /= System.Null_Address and then Codec /= null
           and then Codec.all.Free_Msg /= null
         then
            Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Out_Msg'Access);
         end if;
         if Handle /= System.Null_Address then
            Pcl_Plugins.Pcl_Plugin_Unload (Handle);
            Handle := System.Null_Address;
         end if;
         if Registry /= System.Null_Address then
            Pcl_Plugins.Pcl_Codec_Registry_Destroy (Registry);
            Registry := System.Null_Address;
         end if;
         Interfaces.C.Strings.Free (Path_C);
         Interfaces.C.Strings.Free (Json_C);
         Interfaces.C.Strings.Free (Schema_C);
      end Cleanup;
   begin
      Original.Update_Time := 42.5;
      Original.Id := To_Unbounded_String ("obj-ada-001");
      Original.Entity_Source := To_Unbounded_String ("ada-roundtrip");
      Original.Source := new Source_Array'(1 => Source_Radar,
                                           2 => Source_Local);
      Original.Position.Latitude := 0.8901179185171081;
      Original.Position.Longitude := -0.017453292519943295;
      Original.Creation_Time := 40.0;
      Original.Quality := 0.875;
      Original.Course := 1.2;
      Original.Speed := 13.4;
      Original.Length := 55.0;
      Original.Identity := Identity_Hostile;
      Original.Dimension := Dimension_SeaSurface;

      Registry := Pcl_Plugins.Pcl_Codec_Registry_Create;
      Assert (Registry /= System.Null_Address,
              "registry create returned null");

      Assert (Pcl_Plugins.Pcl_Plugin_Load_Codec
                (Path_C, Registry, Handle'Access) = Pcl_Bindings.PCL_OK,
              "pcl_plugin_load_codec failed");
      Assert (Handle /= System.Null_Address, "plugin handle is null");

      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get (Registry, Json_C);
      Assert (Codec /= null, "application/json codec not registered");
      Assert (Codec.all.Encode /= null, "encode pointer is null");
      Assert (Codec.all.Decode /= null, "decode pointer is null");
      Assert (Codec.all.Free_Msg /= null, "free_msg pointer is null");

      Assert (Codec.all.Encode.all
                (Codec.all.Codec_Ctx, Schema_C, Original'Address,
                 Out_Msg'Access) = Pcl_Bindings.PCL_OK,
              "codec encode failed");
      Assert (Out_Msg.Data /= System.Null_Address,
              "encoded message data is null");
      Assert (Out_Msg.Size > 0, "encoded message is empty");

      Assert (Codec.all.Decode.all
                (Codec.all.Codec_Ctx, Schema_C, Out_Msg'Access,
                 Decoded'Address) = Pcl_Bindings.PCL_OK,
              "codec decode failed");

      Assert_Equal (Original, Decoded);
      Cleanup;
   exception
      when others =>
         Cleanup;
         raise;
   end Run;
begin
   if Ada.Command_Line.Argument_Count /= 1 then
      Log_Line ("FAIL: expected one plugin path argument");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;

   Run (Ada.Command_Line.Argument (1));
   Ada.Text_IO.Put_Line ("ALL TESTS PASSED");
   Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Success);
exception
   when Error : others =>
      Log_Line ("FAIL: " & Ada.Exceptions.Exception_Message (Error));
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
end Test_Ada_JSON_Codec_Roundtrip;
