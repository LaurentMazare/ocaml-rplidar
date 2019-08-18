type t

val create : ?baudrate:int -> string -> t
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
