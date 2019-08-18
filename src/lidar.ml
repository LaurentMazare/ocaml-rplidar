open Base

type t = { fd : Unix.file_descr }

let really_write fd str =
  let len = String.length str in
  let current = ref 0 in
  while !current < len do
    let written = Unix.write_substring fd str !current (len - !current) in
    current := !current + written
  done

let really_read fd len =
  let buffer = Bytes.create len in
  let current = ref 0 in
  while !current < len do
    let read = Unix.read fd buffer !current (len - !current) in
    current := !current + read
  done;
  Bytes.to_string buffer

let create ?(baudrate = 115200) device_name =
  let fd = Unix.openfile device_name [ O_RDWR; O_NOCTTY; O_CLOEXEC ] 0 in
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
  really_write fd "\xa5\xf0\x02\x94\x02\xc1";
  { fd }

module Descriptor = struct
  type t =
    { size : int
    ; is_single : bool
    ; type_ : [ `info | `health | `scan ]
    }

  let read fd =
    let descriptor = really_read fd 7 in
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
    | `get_health -> "\xA5\x52"
    | `scan -> "\xA5\x20"
    | `force_scan -> "\xA5\x21"
    | `stop -> "\xA5\x25"
    | `reset -> "\xA5\x40"
  in
  really_write t.fd command

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
    let info = really_read t.fd descriptor.size in
    { model = Char.to_int info.[0]
    ; firmware = Char.to_int info.[2], Char.to_int info.[1]
    ; hardware = Char.to_int info.[3]
    }
end

module Health = struct
  type lidar = t

  type t =
    | Good
    | Warning of int
    | Error of int

  let get t =
    send_command t `get_health;
    let descriptor = Descriptor.read t.fd in
    if descriptor.size <> 3
    then Printf.failwithf "unexpected health size %d" descriptor.size ();
    let health = really_read t.fd descriptor.size in
    let error_code = (Char.to_int health.[1] * 256) + Char.to_int health.[2] in
    match Char.to_int health.[0] with
    | 0 -> Good
    | 1 -> Warning error_code
    | 2 -> Error error_code
    | code -> Printf.failwithf "unexpected health status %d" code ()
end

let stop t = send_command t `stop

let reset t =
  send_command t `reset;
  Unix.tcflush t.fd TCIFLUSH

module Scan = struct
  type lidar = t

  type t =
    { quality : int
    ; angle : float
    ; dist : float
    }

  let run t ~f =
    send_command t `scan;
    let descriptor = Descriptor.read t.fd in
    if descriptor.size <> 5
    then Printf.failwithf "unexpected scan size %d" descriptor.size ();
    if descriptor.is_single then failwith "scan does not return a stream of answers";
    let rec loop () =
      let s = really_read t.fd descriptor.size in
      let quality = Char.to_int s.[0] lsr 2 in
      let angle = (Char.to_int s.[1] lsr 1) + (Char.to_int s.[2] lsl 7) in
      let angle = Float.of_int angle /. 64. in
      let dist = Char.to_int s.[3] + (Char.to_int s.[4] lsl 8) in
      let dist = Float.of_int dist /. 4. in
      match f { quality; angle; dist } with
      | `continue -> loop ()
      | `break -> ()
    in
    loop ();
    reset t
end

let close t = Unix.close t.fd
