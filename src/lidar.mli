type t

(** [create ?baudrate filename] creates a new Lidar sensor based on
    device [filename], usually /dev/ttyUSB0.
*)
val create : ?baudrate:int -> string -> t
val stop : t -> unit
val reset : ?restart_motor:bool -> t -> unit
val close : t -> unit

module Info : sig
  type lidar = t
  type t =
    { model : int
    ; firmware : int * int
    ; hardware : int
    }

    val get : lidar -> t
end

module Health : sig
  type lidar = t

  type t =
    | Good
    | Warning of int
    | Error of int

  val get : lidar -> t
end

module Scan : sig
  type lidar = t

  type t =
    { quality : int (** The returned quality ranges from 0 to 15. *)
    ; angle : float (** The measurement angle, in degrees. *)
    ; dist : float (* The measured distance, in millimeters. *)
    }

  val run : lidar -> f:(t -> [ `continue | `break ]) -> unit
end
