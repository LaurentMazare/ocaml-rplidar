(* Adapted from BreezySLAM: https://github.com/simondlevy/BreezySLAM *)
let obstacle = 0
let no_obstacle = 65500

module Position = struct
  type t =
    { x_mm : float
    ; y_mm : float
    ; theta_degrees : float
    }
end

module Map_ = struct
  type pixels = (int, Bigarray.int16_unsigned_elt, Bigarray.c_layout) Bigarray.Array2.t

  type t =
    { pixels : pixels
    ; size_meters : float
    ; scale_pixels_per_mm : float
    }

  let create ~size_pixels ~size_meters =
    let pixels =
      Bigarray.Array2.create Int16_unsigned C_layout size_pixels size_pixels
    in
    Bigarray.Array2.fill pixels ((obstacle + no_obstacle) / 2);
    { pixels
    ; size_meters
    ; scale_pixels_per_mm = Float.of_int size_pixels /. (size_meters *. 1e3)
    }
end
