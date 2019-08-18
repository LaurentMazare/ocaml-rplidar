open Base

type t = { fd : Unix.file_descr }

let create ?(baudrate = 115200) device_name =
  let fd = Unix.openfile device_name [ O_RDWR; O_NOCTTY; O_NONBLOCK; O_CLOEXEC ] 0 in
  let tc = Unix.tcgetattr fd in
  let tc =
    { tc with
      c_ixon = false
    ; c_icrnl = false
    ; c_ibaud = baudrate
    ; c_opost = false
    ; c_obaud = baudrate
    ; c_parenb = false
    ; c_cstopb = 1
    ; c_isig = false
    ; c_icanon = false
    ; c_noflsh = false
    ; c_echo = false
    ; c_echoe = false
    ; c_echok = false
    ; c_echonl = false
    }
  in
  Unix.tcsetattr fd TCSANOW tc;
  Ioctl_bindings.ioctl fd ~cmd:`tiocmbis ~arg:`tiocm_dtr;
  Ioctl_bindings.ioctl fd ~cmd:`tiocmbis ~arg:`tiocm_rts;
  Unix.tcflush fd TCIFLUSH;
  Ioctl_bindings.ioctl fd ~cmd:`tiocmbic ~arg:`tiocm_dtr;
  ignore (Unix.write_substring fd "\xa5\xf0\x02\x94\x02\xc1" 0 6);
  ignore (Unix.select [] [ fd ] [] (-1.));
  { fd }

module Descriptor = struct
  type t =
    { size : int
    ; is_single : bool
    ; type_ : [ `info | `health | `scan ]
    }

  let read fd =
    ignore (Unix.select [ fd ] [] [] (-1.));
    let descriptor = Bytes.create 7 in
    ignore (Unix.read fd descriptor 0 7);
    let descriptor = Bytes.to_string descriptor in
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
  ignore (Unix.write_substring t.fd command 0 2);
  ignore (Unix.select [] [ t.fd ] [] (-1.))

module Info = struct
  type lidar = t

  type t =
    { model : int
    ; firmware : int * int
    ; hardware : int
    }

  let get t =
    send_command t `get_info;
    let descriptor = Descriptor.read t.fd in
    if descriptor.size <> 20
    then Printf.failwithf "unexpected info size %d" descriptor.size ();
    let info = Bytes.create 20 in
    ignore (Unix.read t.fd info 0 20);
    let info = Bytes.to_string info in
    { model = Char.to_int info.[0]
    ; firmware = Char.to_int info.[2], Char.to_int info.[1]
    ; hardware = Char.to_int info.[3]
    }
end

let close t = Unix.close t.fd
