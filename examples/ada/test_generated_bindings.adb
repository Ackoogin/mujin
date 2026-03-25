--  Test harness for generated Ada service bindings and codecs.
--  Validates that all generated packages compile and codec round-trips work.
--
--  Build:  gprbuild -p -P test_generated_bindings.gpr
--  Run:    bin/test_generated_bindings
--
--  Exit code 0 = all checks passed.

with Ada.Text_IO;                use Ada.Text_IO;
with Ada.Strings.Unbounded;      use Ada.Strings.Unbounded;

--  Generated data model types
with Pyramid_Data_Model_Base_Types;     use Pyramid_Data_Model_Base_Types;
with Pyramid_Data_Model_Common_Types;   use Pyramid_Data_Model_Common_Types;
with Pyramid_Data_Model_Tactical_Types; use Pyramid_Data_Model_Tactical_Types;

--  Generated data model codecs
with Pyramid_Data_Model_Common_Types_Codec;    use Pyramid_Data_Model_Common_Types_Codec;
with Pyramid_Data_Model_Tactical_Types_Codec;  use Pyramid_Data_Model_Tactical_Types_Codec;

--  Generated service bindings (validates with clauses and typed interfaces)
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Consumed;

procedure Test_Generated_Bindings is
   package Prov renames Pyramid.Services.Tactical_Objects.Provided;
   package Cons renames Pyramid.Services.Tactical_Objects.Consumed;

   Pass_Count : Natural := 0;
   Fail_Count : Natural := 0;

   procedure Check (Name : String; Ok : Boolean) is
   begin
      if Ok then
         Pass_Count := Pass_Count + 1;
      else
         Fail_Count := Fail_Count + 1;
         Put_Line ("FAIL: " & Name);
      end if;
   end Check;

begin
   Put_Line ("=== Generated Ada Binding Validation ===");

   --  1. Enum codec round-trip
   declare
      S : constant String := To_String (Policy_Obtain);
      V : constant Data_Policy := Data_Policy_From_String (S);
   begin
      Check ("DataPolicy enum round-trip",
             V = Policy_Obtain and S = "DATA_POLICY_OBTAIN");
   end;

   --  2. Ack codec round-trip
   declare
      A : constant Ack := (Success => True);
      J : constant String := To_Json (A);
      B : constant Ack := From_Json (J, null);
   begin
      Check ("Ack codec round-trip", B.Success = True);
   end;

   --  3. Query codec round-trip
   declare
      Q : Query;
      J : constant String := To_Json (Q);
      R : constant Query := From_Json (J, null);
   begin
      Check ("Query codec round-trip (empty)", R.Has_One_Shot = False);
   end;

   --  4. ObjectInterestRequirement codec round-trip
   declare
      Req : Object_Interest_Requirement;
      J   : constant String := To_Json (Req);
      R   : constant Object_Interest_Requirement := From_Json (J, null);
   begin
      Check ("ObjectInterestRequirement codec round-trip",
             R.Policy = Policy_Unspecified);
   end;

   --  5. Service wire-name constants are non-empty
   Check ("Provided wire-name constant",
          Prov.Svc_Create_Requirement'Length > 0);
   Check ("Consumed wire-name constant",
          Cons.Svc_Create_Requirement'Length > 0);

   --  6. Topic constants
   Check ("Provided topic constant",
          Prov.Topic_Entity_Matches = "standard.entity_matches");

   --  7. Service channel enumeration compiles and is usable
   declare
      Ch : constant Prov.Service_Channel := Prov.Ch_Create_Requirement;
      pragma Unreferenced (Ch);
   begin
      Check ("ServiceChannel enum usable", True);
   end;

   --  8. Dispatch round-trip — CreateRequirement
   --  We pass valid JSON, dispatch calls the stub handler, and we get
   --  a response buffer back (the stub returns Null_Identifier = "").
   declare
      use type System.Address;
      Req_Json  : constant String := "{}";
      Resp_Buf  : System.Address;
      Resp_Size : Natural;
   begin
      Prov.Dispatch
        (Channel      => Prov.Ch_Create_Requirement,
         Request_Buf  => Req_Json (Req_Json'First)'Address,
         Request_Size => Req_Json'Length,
         Response_Buf => Resp_Buf,
         Response_Size => Resp_Size);
      Check ("Dispatch CreateRequirement returns buffer",
             Resp_Buf /= System.Null_Address or Resp_Size = 0);
   end;

   --  Summary
   New_Line;
   Put_Line ("Passed:" & Natural'Image (Pass_Count));
   Put_Line ("Failed:" & Natural'Image (Fail_Count));

   if Fail_Count > 0 then
      Put_Line ("FAILED");
      raise Program_Error with "Test failures detected";
   else
      Put_Line ("ALL PASSED");
   end if;
end Test_Generated_Bindings;
