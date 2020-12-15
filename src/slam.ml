(* Adapted from the CoreSLAM implementation used in BreezySLAM.
   https://github.com/simondlevy/BreezySLAM
*)
open Base

let obstacle = 0
let no_obstacle = 65500

(* Some default parameters, these could be made customizable. *)
let map_quality = 50 (* out of 255 *)

let hole_width_mm = 600.
let distance_no_detection_mm = 10_000.

module Position = struct
  type t =
    { x_mm : float
    ; y_mm : float
    ; theta_degrees : float
    }

  let x_mm t = t.x_mm
  let y_mm t = t.y_mm
  let theta_degrees t = t.theta_degrees
  let theta_radians t = t.theta_degrees *. Float.pi /. 180.
end

module Angle_distance = struct
  type t =
    { angle_degrees : float
    ; distance_mm : float
    }
end

module Scan_point = struct
  type t =
    { x_mm : float
    ; y_mm : float
    ; value : int
    }
end

module Pixel = struct
  type t =
    { xp : int
    ; yp : int
    }
end

let round f = Float.round_down (f +. 0.5) |> Int.of_float

module Map = struct
  type pixels = (int, Bigarray.int16_unsigned_elt, Bigarray.c_layout) Bigarray.Array1.t

  type t =
    { pixels : pixels
    ; size_mm : float
    ; size_pixel : int
    ; scale_pixels_per_mm : float
    }

  let create ~size_pixel ~size_mm =
    let pixels =
      Bigarray.Array1.create Int16_unsigned C_layout (size_pixel * size_pixel)
    in
    Bigarray.Array1.fill pixels ((obstacle + no_obstacle) / 2);
    { pixels
    ; size_mm
    ; size_pixel
    ; scale_pixels_per_mm = Float.of_int size_pixel /. size_mm
    }

  let clip t ~xyc ~yxc ~xy ~yx =
    let sz = t.size_pixel in
    if xyc < 0
    then if xyc = xy then None else Some (0, yxc + ((yxc - yx) * -xyc / (xyc - xy)))
    else if xyc >= sz
    then
      if xyc = xy
      then None
      else Some (sz - 1, yxc + ((yxc - yx) * (sz - 1 - xyc) / (xyc - xy)))
    else Some (xyc, yxc)

  exception No_error_gradient

  let laser_ray t ~x1 ~y1 ~x2 ~y2 ~xp ~yp ~value ~alpha =
    let map_size = t.size_pixel in
    if 0 <= x1 && x1 < map_size && 0 <= y1 && y1 < map_size
    then (
      let x2c, y2c = x2, y2 in
      match clip t ~xyc:x2c ~yxc:y2c ~xy:x1 ~yx:y1 with
      | None -> ()
      | Some (x2c, y2c) ->
        (match clip t ~xyc:y2c ~yxc:x2c ~xy:y1 ~yx:x1 with
        | None -> ()
        | Some (y2c, x2c) ->
          let dx = Int.abs (x2 - x1) in
          let dy = Int.abs (y2 - y1) in
          let dxc = Int.abs (x2c - x1) in
          let dyc = Int.abs (y2c - y1) in
          let incptrx = if x2 > x1 then 1 else -1 in
          let incptry = if y2 > y1 then map_size else -map_size in
          let sincv = if value > no_obstacle then 1 else -1 in
          let derrorv, maybe_flip =
            if dx > dy
            then Int.abs (xp - x2), fun x y -> x, y
            else Int.abs (yp - y2), fun x y -> y, x
          in
          let dx, _dy = maybe_flip dx dy in
          let dxc, dyc = maybe_flip dxc dyc in
          let incptrx, incptry = maybe_flip incptrx incptry in
          if derrorv = 0 then raise No_error_gradient;
          let error = ref ((2 * dyc) - dxc) in
          let horiz = 2 * dyc in
          let diago = 2 * (dyc - dxc) in
          let errorv = ref (derrorv / 2) in
          let incv = (value - no_obstacle) / derrorv in
          let incerrorv = value - no_obstacle - (derrorv * incv) in
          let index = ref ((y1 * map_size) + x1) in
          let pixval = ref no_obstacle in
          for x = 0 to dxc do
            if x > dx - (2 * derrorv)
            then
              if x <= dx - derrorv
              then (
                pixval := !pixval + incv;
                errorv := !errorv + incerrorv;
                if !errorv > derrorv
                then (
                  pixval := !pixval + sincv;
                  errorv := !errorv - derrorv))
              else (
                pixval := !pixval - incv;
                errorv := !errorv - incerrorv;
                if !errorv < 0
                then (
                  pixval := !pixval - sincv;
                  errorv := !errorv + derrorv));
            let v = (t.pixels.{!index} * (256 - alpha)) + (alpha * !pixval) in
            t.pixels.{!index} <- v / 256;
            if !error > 0 then index := !index + incptry;
            error := !error + if !error > 0 then diago else horiz;
            index := !index + incptrx
          done))

  let update t (pos : Position.t) (xyvs : Scan_point.t list) =
    let theta_radians = Position.theta_radians pos in
    let cos_theta = Float.cos theta_radians in
    let sin_theta = Float.sin theta_radians in
    let x1 = round (pos.x_mm *. t.scale_pixels_per_mm) in
    let y1 = round (pos.y_mm *. t.scale_pixels_per_mm) in
    List.map xyvs ~f:(fun { x_mm; y_mm; value } ->
        let x2p = (cos_theta *. x_mm) -. (sin_theta *. y_mm) in
        let y2p = (sin_theta *. x_mm) +. (cos_theta *. y_mm) in
        let xp = round ((pos.x_mm +. x2p) *. t.scale_pixels_per_mm) in
        let yp = round ((pos.y_mm +. y2p) *. t.scale_pixels_per_mm) in
        let dist = Float.sqrt ((x2p *. x2p) +. (y2p *. y2p)) in
        let add = hole_width_mm /. 2. /. dist in
        let x2p = x2p *. t.scale_pixels_per_mm *. (1. +. add) in
        let y2p = y2p *. t.scale_pixels_per_mm *. (1. +. add) in
        let x2 = round ((pos.x_mm *. t.scale_pixels_per_mm) +. x2p) in
        let y2 = round ((pos.y_mm *. t.scale_pixels_per_mm) +. y2p) in
        let value, alpha =
          if value = no_obstacle
          then no_obstacle, map_quality / 4
          else obstacle, map_quality
        in
        (try laser_ray t ~x1 ~y1 ~x2 ~y2 ~xp ~yp ~value ~alpha with
        | No_error_gradient -> ());
        { Pixel.xp; yp })
end

type t =
  { map : Map.t
  ; mutable position : Position.t
  }

let create ~map_size_pixel ~map_size_mm =
  let map = Map.create ~size_pixel:map_size_pixel ~size_mm:map_size_mm in
  let position =
    { Position.x_mm = map_size_mm *. 0.5; y_mm = map_size_mm *. 0.5; theta_degrees = 0. }
  in
  { map; position }

let current_position t = t.position

let update t ads =
  (* TODO: correct for speed/rotation of the robot and the measurements not happening
  all at the same time. *)
  let xyvs =
    List.map ads ~f:(fun { Angle_distance.angle_degrees; distance_mm } ->
        let angle_radians = angle_degrees *. Float.pi /. 180. in
        let value, distance_mm =
          if Float.( = ) distance_mm 0.
          then no_obstacle, distance_no_detection_mm
          else obstacle, distance_mm
        in
        let x_mm = distance_mm *. Float.cos angle_radians in
        let y_mm = distance_mm *. Float.sin angle_radians in
        { Scan_point.x_mm; y_mm; value })
  in
  Map.update t.map t.position xyvs

let map_memory t =
  let genarray = Bigarray.genarray_of_array1 t.map.pixels in
  Bigarray.reshape_2 genarray t.map.size_pixel t.map.size_pixel

(* [obstactle_xys] is in pixels. *)
let distance map position ~obstacle_xys =
  let theta_radians = Position.theta_radians position in
  let { Map.size_pixel; scale_pixels_per_mm; pixels; _ } = map in
  let cos_theta = Float.cos theta_radians in
  let sin_theta = Float.sin theta_radians in
  let x_pix = position.x_mm *. scale_pixels_per_mm in
  let y_pix = position.y_mm *. scale_pixels_per_mm in
  let n_points = ref 0 in
  let sum = ref 0 in
  for i = 0 to Array.length obstacle_xys - 1 do
    let obstacle_x, obstacle_y = obstacle_xys.(i) in
    let x = round (x_pix +. (cos_theta *. obstacle_x) -. (sin_theta *. obstacle_y)) in
    let y = round (y_pix +. (sin_theta *. obstacle_x) +. (cos_theta *. obstacle_y)) in
    if x >= 0 && x < size_pixel && y >= 0 && y < size_pixel
    then (
      Int.incr n_points;
      sum := !sum + pixels.{(y * size_pixel) + x})
  done;
  if !n_points > 0 then Some (Int.to_float !sum /. Int.to_float !n_points) else None

let max_search_iter = 1000
let default_sigma_theta_degrees = 20.
let default_sigma_xy_mm = 100.

module Box_muller = struct
  let state = ref None
  let two_pi = Float.pi *. 2.

  let rnd () =
    match !state with
    | Some value ->
      state := None;
      value
    | None ->
      let u1 = Random.float 1. in
      let u2 = Random.float 1. in
      let s = Float.sqrt (-2. *. Float.log u1) in
      let x = s *. Float.cos (two_pi *. u2) in
      let y = s *. Float.sin (two_pi *. u2) in
      state := Some y;
      x
end

(* Random Monte-Carlo Hill Climb optimization. *)
let rmhc_optimization t ads =
  let obstacle_xys =
    List.filter_map ads ~f:(fun { Angle_distance.angle_degrees; distance_mm } ->
        let angle_radians = angle_degrees *. Float.pi /. 180. in
        if Float.( = ) distance_mm 0.
        then None
        else (
          let distance_pix = distance_mm *. t.map.scale_pixels_per_mm in
          let x = distance_pix *. Float.cos angle_radians in
          let y = distance_pix *. Float.sin angle_radians in
          Some (x, y)))
    |> Array.of_list
  in
  let distance pos = distance t.map pos ~obstacle_xys in
  match distance t.position with
  | None -> t.position
  | Some start_distance ->
    let last_best_position = ref t.position in
    let last_lowest_distance = ref start_distance in
    let best_position = ref !last_best_position in
    let lowest_distance = ref !last_lowest_distance in
    let sigma_xy_mm = ref default_sigma_xy_mm in
    let sigma_theta_degrees = ref default_sigma_theta_degrees in
    let counter = ref 0 in
    while !counter < max_search_iter do
      let current_position =
        let { Position.x_mm; y_mm; theta_degrees } = !last_best_position in
        let x_mm = x_mm +. (Box_muller.rnd () *. !sigma_xy_mm) in
        let y_mm = y_mm +. (Box_muller.rnd () *. !sigma_xy_mm) in
        let theta_degrees =
          theta_degrees +. (Box_muller.rnd () *. !sigma_theta_degrees)
        in
        { Position.x_mm; y_mm; theta_degrees }
      in
      (match distance current_position with
      | Some current_distance when Float.( < ) current_distance !lowest_distance ->
        lowest_distance := current_distance;
        best_position := current_position
      | Some _ | None -> Int.incr counter);
      if !counter > max_search_iter / 2
         && Float.( < ) !lowest_distance !last_lowest_distance
      then (
        last_best_position := !best_position;
        last_lowest_distance := !lowest_distance;
        counter := 0;
        sigma_xy_mm := !sigma_xy_mm *. 0.5;
        sigma_theta_degrees := !sigma_theta_degrees *. 0.5)
    done;
    !last_best_position
