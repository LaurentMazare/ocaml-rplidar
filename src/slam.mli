open! Base

module Position : sig
  type t

  val x_mm : t -> float
  val y_mm : t -> float
  val theta_degrees : t -> float
end

module Angle_distance : sig
  type t =
    { angle : float
    ; distance : int
    }
end

module Scan : sig
  type t
  type coordinates_and_value
end

module Map : sig
  type t

  val update : t -> Position.t -> Scan.coordinates_and_value list -> unit
end

type t

val create : map_size_in_pixels:int -> map_size_in_meters:float -> t
val current_position : t -> Position.t
val update : t -> Angle_distance.t list -> unit