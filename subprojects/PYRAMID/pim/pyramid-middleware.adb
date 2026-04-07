--  Pyramid Middleware Binding Body
--  TODO: Implement these procedures to bind to your actual middleware

with Ada.Text_IO;

package body Pyramid.Middleware is

   procedure Send
     (Channel : in Integer;
      Data    : in System.Address;
      Size    : in Integer)
   is
   begin
      --  TODO: Implement Send to your middleware
      --  Example for DDS:
      --    DDS_Writer.Write (Channel, Data, Size);
      --  
      --  Example for ZeroMQ:
      --    ZMQ.Send (Socket (Channel), Data, Size);
      --
      --  Example for shared memory:
      --    Shared_Memory.Write (Channel, Data, Size);
      
      Ada.Text_IO.Put_Line 
        ("Send: Channel=" & Channel'Image & 
         ", Size=" & Size'Image);
      
      --  Stub: Just log for now
      null;
   end Send;

   procedure Receive
     (Channel : in Integer;
      Data    : in System.Address;
      Size    : in Integer)
   is
   begin
      --  TODO: Implement Receive from your middleware
      --  Example for DDS:
      --    DDS_Reader.Read (Channel, Data, Size);
      --
      --  Example for ZeroMQ:
      --    ZMQ.Receive (Socket (Channel), Data, Size);
      --
      --  Example for shared memory:
      --    Shared_Memory.Read (Channel, Data, Size);
      
      Ada.Text_IO.Put_Line 
        ("Receive: Channel=" & Channel'Image & 
         ", Size=" & Size'Image);
      
      --  Stub: Block indefinitely (replace with actual receive)
      delay until Ada.Calendar.Clock + Duration'Last;
   end Receive;

   procedure Receive_Any
     (Channel : out Integer;
      Data    : out System.Address;
      Size    : out Integer)
   is
   begin
      --  TODO: Implement Receive_Any from your middleware
      --  This should block until data arrives on ANY channel
      --  and return which channel received data
      --
      --  Example for DDS:
      --    DDS_Waitset.Wait (Channel, Data, Size);
      --
      --  Example for multiplexed socket:
      --    Select_Loop.Wait (Channel, Data, Size);
      
      Ada.Text_IO.Put_Line ("Receive_Any: waiting...");
      
      --  Stub: Block indefinitely (replace with actual receive)
      delay until Ada.Calendar.Clock + Duration'Last;
      
      Channel := 0;
      Data := System.Null_Address;
      Size := 0;
   end Receive_Any;

end Pyramid.Middleware;
