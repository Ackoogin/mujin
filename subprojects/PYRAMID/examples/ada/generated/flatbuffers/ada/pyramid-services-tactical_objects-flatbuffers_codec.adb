--  Auto-generated tactical service FlatBuffers codec

package body Pyramid.Services.Tactical_Objects.Flatbuffers_Codec is
   Magic : constant String := "PWFB";

   function Encode_Payload (Payload : String) return String is
   begin
      return Magic & Payload;
   end Encode_Payload;

   function Decode_Payload (Payload : String) return String is
   begin
      if Payload'Length < Magic'Length
        or else Payload (Payload'First .. Payload'First + Magic'Length - 1) /= Magic
      then
         raise Constraint_Error with "Invalid tactical flatbuffers payload";
      end if;

      return Payload (Payload'First + Magic'Length .. Payload'Last);
   end Decode_Payload;
end Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
