with Ada.Text_IO;                use Ada.Text_IO;
with Ada.Strings.Unbounded;      use Ada.Strings.Unbounded;
with Interfaces;
with Pyramid.UUIDs;              use Pyramid.UUIDs;

procedure Test_Pyramid_UUIDs is
   use type Interfaces.Unsigned_8;

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
   Put_Line ("=== Pyramid.UUIDs Validation ===");

   declare
      Parsed : constant UUID := Parse ("00112233-4455-6677-8899-aabbccddeeff");
   begin
      Check ("Parse canonical string",
             To_String (Parsed) = "00112233-4455-6677-8899-aabbccddeeff");
      Check ("Canonical identifier conversion",
             To_String (To_Identifier (Parsed)) =
               "00112233-4455-6677-8899-aabbccddeeff");
   end;

   declare
      Parsed : constant UUID := Parse (To_Unbounded_String ("00112233445566778899AABBCCDDEEFF"));
   begin
      Check ("Parse compact uppercase identifier",
             To_String (Parsed) = "00112233-4455-6677-8899-aabbccddeeff");
   end;

   declare
      Parsed : constant UUID := Parse ("00112233-4455-6677-8899-aabbccddeeff");
      Bytes  : constant Byte_Array := To_Bytes (Parsed);
      Round  : constant UUID := From_Bytes (Bytes);
   begin
      Check ("Byte round-trip", Round = Parsed);
      Check ("Byte values preserved",
             Bytes (0) = 16#00# and then Bytes (15) = 16#FF#);
   end;

   Check ("Reject malformed UUID",
          not Is_Valid ("not-a-uuid"));

   declare
      Random_Value : constant UUID := Generate_V4;
      Random_Text  : constant String := To_String (Random_Value);
      Random_Bytes : constant Byte_Array := To_Bytes (Random_Value);
   begin
      Check ("Generate_V4 format", Is_Valid (Random_Text));
      Check ("Generate_V4 version nibble",
             (Random_Bytes (6) and 16#F0#) = 16#40#);
      Check ("Generate_V4 variant bits",
             (Random_Bytes (8) and 16#C0#) = 16#80#);
   end;

   declare
      Namespace_UUID : constant UUID :=
        Parse ("6ba7b810-9dad-11d1-80b4-00c04fd430c8");
      Generated : constant UUID := Generate_V5 (Namespace_UUID, "python.org");
   begin
      Check ("Generate_V5 RFC example",
             To_String (Generated) =
               "886313e1-3b8a-5372-9b90-0c9aee199e5d");
   end;

   New_Line;
   Put_Line ("Passed:" & Natural'Image (Pass_Count));
   Put_Line ("Failed:" & Natural'Image (Fail_Count));

   if Fail_Count > 0 then
      Put_Line ("FAILED");
      raise Program_Error with "Test failures detected";
   else
      Put_Line ("ALL PASSED");
   end if;
end Test_Pyramid_UUIDs;
