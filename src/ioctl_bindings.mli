val ioctl
  :  Unix.file_descr
  -> cmd:[ `tiocmbis | `tiocmbic ]
  -> arg:[ `tiocm_dtr | `tiocm_rts ]
  -> unit
