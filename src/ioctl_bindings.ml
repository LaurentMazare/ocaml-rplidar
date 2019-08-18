open Base

external ioctl : Unix.file_descr -> int -> int -> int = "ml_ioctl"

let ioctl fd ~cmd ~arg =
  let cmd =
    match cmd with
    | `tiocmbis -> 0x5416
    | `tiocmbic -> 0x5417
  in
  let arg =
    match arg with
    | `tiocm_dtr -> 0x002
    | `tiocm_rts -> 0x004
  in
  let res = ioctl fd cmd arg in
  if res <> 0 then Printf.failwithf "ioctl failed with err code %d" res ()
