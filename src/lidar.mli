type t

val create : ?baudrate:int -> string -> t
val stop : t -> unit
val reset : t -> unit
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
    { quality : int
    ; angle : float
    ; dist : float
    }

  val run : lidar -> f:(t -> [ `continue | `break ]) -> unit
end
