open Base
open Stdio

type t =
  { in_channel : In_channel.t
  ; out_channel : Out_channel.t
  ; fd : Unix.file_descr
  }

let create ?(baudrate = 115200) device_name =
  let fd = Unix.openfile device_name [ O_RDWR ] 0 in
  let in_channel = Unix.in_channel_of_descr fd in
  let out_channel = Unix.out_channel_of_descr fd in
  let tc = Unix.tcgetattr fd in
  let tc =
    { tc with c_ibaud = baudrate; c_obaud = baudrate; c_parenb = false; c_cstopb = 1 }
  in
  Unix.tcsetattr fd TCSANOW tc;
  Unix.tcflush fd TCIFLUSH;
  { in_channel; out_channel; fd }

module Descriptor = struct
  type t =
    { size : int
    ; is_single : bool
    ; type_ : [ `info | `health | `scan ]
    }

  let read fd =
    let descriptor = Stdlib.really_input_string fd 1 in
    if String.length descriptor <> 7 then failwith "unexpected descriptor size";
    if Char.( <> ) descriptor.[0] '\xA5' || Char.( <> ) descriptor.[1] '\x5A'
    then failwith "incorrect descriptor sync bytes";
    let type_ =
      match Char.to_int descriptor.[6] with
      | 4 -> `info
      | 6 -> `health
      | 129 -> `scan
      | code -> Printf.failwithf "unexpected type code %d" code ()
    in
    { size = Char.to_int descriptor.[2]
    ; is_single = Char.to_int descriptor.[5] = 0
    ; type_
    }
end

let send_command t command =
  let command =
    match command with
    | `get_info -> "\xA5\x50"
  in
  Out_channel.output_string t.out_channel command;
  Out_channel.flush t.out_channel

module Info = struct
  type lidar = t

  type t =
    { model : int
    ; firmware : int * int
    ; hardware : int
    }

  let get t =
    send_command t `get_info;
    let descriptor = Descriptor.read t.in_channel in
    if descriptor.size <> 20
    then Printf.failwithf "unexpected info size %d" descriptor.size ();
    let info = Stdlib.really_input_string t.in_channel 20 in
    { model = Char.to_int info.[0]
    ; firmware = Char.to_int info.[2], Char.to_int info.[1]
    ; hardware = Char.to_int info.[3]
    }
end

let close t = Unix.close t.fd
