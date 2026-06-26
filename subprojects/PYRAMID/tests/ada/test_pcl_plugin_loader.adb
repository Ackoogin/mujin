with Ada.Command_Line;
with Ada.Exceptions;
with Ada.Text_IO;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Plugins;
with System;
with System.Address_To_Access_Conversions;

procedure Test_Pcl_Plugin_Loader is
  use type Interfaces.C.char;
  use type Interfaces.C.unsigned;
  use type Interfaces.C.Strings.chars_ptr;
  use type Pcl_Plugins.Pcl_Codec_Const_Access;
  use type Pcl_Plugins.Pcl_Codec_Encode_Access;
  use type Pcl_Plugins.Pcl_Codec_Free_Msg_Access;
  use type Pcl_Bindings.Pcl_Status;
  use type System.Address;

  type Encoded_Buffer is array (Interfaces.C.size_t range 0 .. 12)
    of Interfaces.C.char;
  pragma Convention(C, Encoded_Buffer);

  package Encoded_Pointers is
    new System.Address_To_Access_Conversions(Encoded_Buffer);
  use type Encoded_Pointers.Object_Pointer;

  procedure Log_Line(Message : String) is
  begin
    Ada.Text_IO.Put_Line(Ada.Text_IO.Standard_Error, Message);
    Ada.Text_IO.Flush(Ada.Text_IO.Standard_Error);
  end Log_Line;

  procedure Assert(Condition : Boolean; Message : String) is
  begin
    if not Condition then
      raise Program_Error with Message;
    end if;
  end Assert;

  procedure Run(Plugin_Path : String) is
    Registry : System.Address := System.Null_Address;
    Handle   : aliased System.Address := System.Null_Address;
    Handle2  : aliased System.Address := System.Null_Address;
    Codec    : Pcl_Plugins.Pcl_Codec_Const_Access := null;
    Out_Msg  : aliased Pcl_Bindings.Pcl_Msg :=
      (Data      => System.Null_Address,
       Size      => 0,
       Type_Name => Interfaces.C.Strings.Null_Ptr);

    Path_C    : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String(Plugin_Path);
    Stub_C    : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("application/stub");
    Schema_C  : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("X");
    Missing_C : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String("/no/such/plugin.so");

    procedure Cleanup is
    begin
      if Handle /= System.Null_Address then
        Pcl_Plugins.Pcl_Plugin_Unload(Handle);
        Handle := System.Null_Address;
      end if;

      if Registry /= System.Null_Address then
        Pcl_Plugins.Pcl_Codec_Registry_Destroy(Registry);
        Registry := System.Null_Address;
      end if;

      Interfaces.C.Strings.Free(Path_C);
      Interfaces.C.Strings.Free(Stub_C);
      Interfaces.C.Strings.Free(Schema_C);
      Interfaces.C.Strings.Free(Missing_C);
    end Cleanup;

    procedure Verify_Encoded_Payload is
      Expected : constant Interfaces.C.char_array :=
        Interfaces.C.To_C("STUB_ENCODED", Append_Nul => True);
      Payload : constant Encoded_Pointers.Object_Pointer :=
        Encoded_Pointers.To_Pointer(Out_Msg.Data);
    begin
      Assert(Payload /= null, "encoded payload pointer is null");
      Assert(Expected'Length = 13, "unexpected expected marker length");

      for I in Expected'Range loop
        Assert(Payload.all(I) = Expected(I), "encoded marker byte mismatch");
      end loop;
    end Verify_Encoded_Payload;
  begin
    Registry := Pcl_Plugins.Pcl_Codec_Registry_Create;
    Assert(Registry /= System.Null_Address, "registry create returned null");

    Assert(Pcl_Plugins.Pcl_Plugin_Load_Codec
             (Path_C, Interfaces.C.Strings.Null_Ptr, Registry, Handle'Access) =
           Pcl_Bindings.PCL_OK,
           "pcl_plugin_load_codec failed for stub plugin");
    Assert(Handle /= System.Null_Address, "plugin load returned null handle");

    Codec := Pcl_Plugins.Pcl_Codec_Registry_Get(Registry, Stub_C);
    Assert(Codec /= null, "registry did not return application/stub codec");
    Assert(Codec.all.Abi_Version = Pcl_Plugins.Pcl_Codec_Abi_Version,
           "codec ABI version mismatch");
    Assert(Codec.all.Encode /= null, "codec encode pointer is null");
    Assert(Codec.all.Free_Msg /= null, "codec free_msg pointer is null");

    Assert(Codec.all.Encode.all(Codec.all.Codec_Ctx,
                                Schema_C,
                                System.Null_Address,
                                Out_Msg'Access) = Pcl_Bindings.PCL_OK,
           "codec encode failed");
    Assert(Out_Msg.Data /= System.Null_Address, "encode returned null data");
    Assert(Out_Msg.Size = Interfaces.C.unsigned(13),
           "encoded message size mismatch");
    Verify_Encoded_Payload;

    Codec.all.Free_Msg.all(Codec.all.Codec_Ctx, Out_Msg'Access);
    Assert(Out_Msg.Data = System.Null_Address, "free_msg did not clear data");
    Assert(Out_Msg.Size = 0, "free_msg did not clear size");
    Assert(Out_Msg.Type_Name = Interfaces.C.Strings.Null_Ptr,
           "free_msg did not clear type name");

    Assert(Pcl_Plugins.Pcl_Plugin_Load_Codec(Missing_C,
                                             Interfaces.C.Strings.Null_Ptr,
                                             Registry,
                                             Handle2'Access) =
           Pcl_Bindings.PCL_ERR_NOT_FOUND,
           "missing plugin path did not return PCL_ERR_NOT_FOUND");

    Cleanup;
  exception
    when others =>
      Cleanup;
      raise;
  end Run;
begin
  if Ada.Command_Line.Argument_Count /= 1 then
    Log_Line("FAIL: expected one plugin path argument");
    Ada.Command_Line.Set_Exit_Status(Ada.Command_Line.Failure);
    return;
  end if;

  Run(Ada.Command_Line.Argument(1));
  Ada.Text_IO.Put_Line("ALL TESTS PASSED");
  Ada.Command_Line.Set_Exit_Status(Ada.Command_Line.Success);
exception
  when Error : others =>
    Log_Line("FAIL: " & Ada.Exceptions.Exception_Message(Error));
    Ada.Command_Line.Set_Exit_Status(Ada.Command_Line.Failure);
end Test_Pcl_Plugin_Loader;
