--  Auto-generated FlatBuffers codec spec — do not edit
--  Backend: flatbuffers | Package: Pyramid.Data_model.Autonomy.Flatbuffers_Codec
--
--  This package provides thin bindings to the C++ FlatBuffers codec.
--  Actual ser/de is performed via C interop (Import pragma).

with Interfaces.C; use Interfaces.C;
with System;

package Pyramid.Data_model.Autonomy.Flatbuffers_Codec is

   Content_Type : constant String := "application/flatbuffers";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "FactUpdate_to_flatbuffer";

   function From_Binary_Fact_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "FactUpdate_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "StateUpdate_to_flatbuffer";

   function From_Binary_State_Update
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "StateUpdate_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "MissionIntent_to_flatbuffer";

   function From_Binary_Mission_Intent
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "MissionIntent_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "AgentState_to_flatbuffer";

   function From_Binary_Agent_State
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "AgentState_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PolicyEnvelope_to_flatbuffer";

   function From_Binary_Policy_Envelope
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PolicyEnvelope_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Session_to_flatbuffer";

   function From_Binary_Session
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Session_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Capabilities_to_flatbuffer";

   function From_Binary_Capabilities
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Capabilities_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "StringKeyValue_to_flatbuffer";

   function From_Binary_String_Key_Value
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "StringKeyValue_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Command_to_flatbuffer";

   function From_Binary_Command
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Command_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "GoalDispatch_to_flatbuffer";

   function From_Binary_Goal_Dispatch
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "GoalDispatch_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "DecisionRecord_to_flatbuffer";

   function From_Binary_Decision_Record
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "DecisionRecord_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "CommandResult_to_flatbuffer";

   function From_Binary_Command_Result
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "CommandResult_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "DispatchResult_to_flatbuffer";

   function From_Binary_Dispatch_Result
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "DispatchResult_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SessionSnapshot_to_flatbuffer";

   function From_Binary_Session_Snapshot
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SessionSnapshot_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SessionStepRequest_to_flatbuffer";

   function From_Binary_Session_Step_Request
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SessionStepRequest_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SessionStopRequest_to_flatbuffer";

   function From_Binary_Session_Stop_Request
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SessionStopRequest_from_flatbuffer";

end Pyramid.Data_model.Autonomy.Flatbuffers_Codec;
