--  Pyramid Middleware Binding
--  This is a template - implement the body to bind to your actual middleware
--  (DDS, CORBA, ZeroMQ, etc.)

with System;

package Pyramid.Middleware is

   --  Send data on a specific channel
   procedure Send
     (Channel : in Integer;
      Data    : in System.Address;
      Size    : in Integer);
   --  Sends Size bytes from Data address on the specified Channel
   --  Blocks until data is sent (or timeout if middleware supports it)

   --  Receive data from a specific channel
   procedure Receive
     (Channel : in Integer;
      Data    : in System.Address;
      Size    : in Integer);
   --  Receives up to Size bytes into Data address from specified Channel
   --  Blocks until data arrives

   --  Receive from any channel (for service event loops)
   procedure Receive_Any
     (Channel : out Integer;
      Data    : out System.Address;
      Size    : out Integer);
   --  Blocks until data arrives on any channel
   --  Returns the Channel that received data, Data address, and Size

end Pyramid.Middleware;
