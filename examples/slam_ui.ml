open Base
open Rplidar

let map_size_pixel = 512
let map_size_mm = 10000.

module Sample = struct
  (* TODO: also log the time for the sample? *)
  type t =
    { degrees : float
    ; mm : float
    }
  [@@deriving sexp]
end

module G = struct
  type t =
    { map_memory : (int, Bigarray.int16_unsigned_elt, Bigarray.c_layout) Bigarray.Array2.t
    ; size_x : int
    ; size_y : int
    }

  let create slam =
    Graphics.open_graph "";
    Graphics.set_color Graphics.blue;
    Graphics.auto_synchronize false;
    { size_x = Graphics.size_x ()
    ; size_y = Graphics.size_y ()
    ; map_memory = Slam.map_memory slam
    }

  let _close _t = Graphics.close_graph ()

  let update t ~scan_points ~batch ~pos =
    let scale_pixels_per_mm = Float.of_int map_size_pixel /. map_size_mm in
    Graphics.set_color Graphics.white;
    Graphics.fill_rect 0 0 t.size_x t.size_y;
    Graphics.set_color Graphics.green;
    Graphics.fill_circle (map_size_pixel / 2) (map_size_pixel * 3 / 2) 3;
    Graphics.set_color Graphics.red;
    List.iter batch ~f:(fun { Slam.Angle_distance.angle_degrees; distance_mm } ->
        let angle_radians = angle_degrees *. Float.pi /. 180. in
        let xp = distance_mm *. Float.cos angle_radians *. scale_pixels_per_mm in
        let yp = distance_mm *. Float.sin angle_radians *. scale_pixels_per_mm in
        let xp = Float.to_int xp + (map_size_pixel * 3 / 2) in
        let yp = Float.to_int yp + (map_size_pixel * 1 / 2) in
        Graphics.fill_circle yp xp 2);
    (* TODO: efficient plotting. *)
    for i = 0 to map_size_pixel - 1 do
      for j = 0 to map_size_pixel - 1 do
        (* The value is encoded between 0 and ~2**16 *)
        let v = t.map_memory.{i, j} / 256 in
        Graphics.set_color (Graphics.rgb v v v);
        Graphics.plot i j
      done
    done;
    Graphics.set_color Graphics.blue;
    List.iter scan_points ~f:(fun { Slam.Pixel.xp; yp; _ } ->
        Graphics.fill_circle yp xp 2);
    let { Slam.Position.x_mm; y_mm; theta_degrees } = pos in
    let x_pix = x_mm *. scale_pixels_per_mm in
    let y_pix = y_mm *. scale_pixels_per_mm in
    let theta_radians = theta_degrees *. Float.pi /. 180. in
    let xx_pix = x_pix +. (Float.cos theta_radians *. 20.) in
    let yy_pix = y_pix +. (Float.sin theta_radians *. 20.) in
    let x_pix = Float.to_int x_pix in
    let y_pix = Float.to_int y_pix in
    let xx_pix = Float.to_int xx_pix in
    let yy_pix = Float.to_int yy_pix in
    Graphics.set_color Graphics.red;
    Graphics.fill_circle y_pix x_pix 3;
    Graphics.draw_poly_line [| y_pix, x_pix; yy_pix, xx_pix |];
    Graphics.synchronize ()
end

module Batched_slam : sig
  type t

  val create : unit -> t
  val slam : t -> Slam.t

  val process
    :  t
    -> Sample.t
    -> [ `no_update | `update of Slam.Pixel.t list * Slam.Angle_distance.t list ]
end = struct
  type t =
    { mutable batch : Slam.Angle_distance.t list
    ; mutable updates : int
    ; slam : Slam.t
    }

  let create () =
    let slam = Slam.create ~map_size_pixel ~map_size_mm in
    { slam; batch = []; updates = 0 }

  let slam t = t.slam

  let process t { Sample.degrees; mm } =
    let angle_distance =
      { Slam.Angle_distance.angle_degrees = degrees; distance_mm = mm }
    in
    t.batch <- angle_distance :: t.batch;
    if List.length t.batch > 200
    then (
      if t.updates > 5
      then (
        Slam.rmhc_optimization t.slam t.batch;
        let pos = Slam.current_position t.slam in
        Stdio.printf "%f %f %f\n%!" pos.x_mm pos.y_mm pos.theta_degrees);
      let batch = t.batch in
      t.batch <- [];
      let scan_points = Slam.update t.slam batch in
      t.updates <- 1 + t.updates;
      `update (scan_points, batch))
    else `no_update
end

let replay ~in_channel =
  let slam = Batched_slam.create () in
  let g = Batched_slam.slam slam |> G.create in
  Stdio.In_channel.iter_lines in_channel ~f:(fun line ->
      let sample = Core_kernel.Sexp.of_string line |> Sample.t_of_sexp in
      match Batched_slam.process slam sample with
      | `no_update -> ()
      | `update (scan_points, batch) ->
        G.update
          g
          ~scan_points
          ~batch
          ~pos:(Batched_slam.slam slam |> Slam.current_position))

let run ~out_channel =
  (* Only record without actualising a slam model as not being able to process
     all lidar updates result in incorrect measures.  *)
  let lidar = Lidar.create "/dev/ttyUSB0" in
  Stdio.eprintf "Lidar initialized!\n%!";
  let info = Lidar.Info.get lidar in
  let firmware1, firmware2 = info.firmware in
  Stdio.eprintf
    "Model: %d, Firmware: %d,%d, Hardware: %d.\n%!"
    info.model
    firmware1
    firmware2
    info.hardware;
  let health =
    match Lidar.Health.get lidar with
    | Good -> "good"
    | Warning err -> Printf.sprintf "warning %d" err
    | Error err -> Printf.sprintf "error %d" err
  in
  Stdio.printf "Lidar health: %s\n%!" health;
  Caml.Sys.set_signal
    Caml.Sys.sigint
    (Signal_handle
       (fun id ->
         Stdio.eprintf "caught signal %d, exiting\n%!" id;
         Lidar.stop lidar;
         Lidar.close lidar;
         Stdio.Out_channel.flush out_channel;
         Caml.exit 1));
  try
    Lidar.Scan.run lidar ~f:(fun { quality; angle; dist } ->
        (* TODO: type-safe way to handle the zero distance? *)
        if quality > 0
        then (
          let sample = { Sample.degrees = angle; mm = dist } in
          Stdio.Out_channel.fprintf
            out_channel
            "%s\n"
            (Sample.sexp_of_t sample |> Sexp.to_string_mach));
        `continue)
  with
  | exn ->
    Lidar.stop lidar;
    Lidar.close lidar;
    raise exn

let () =
  match Caml.Sys.argv with
  | [| _; "record"; filename |] ->
    Stdio.Out_channel.with_file filename ~f:(fun out_channel -> run ~out_channel)
  | [| _; "replay"; filename |] ->
    Stdio.In_channel.with_file filename ~f:(fun in_channel -> replay ~in_channel)
  | argv ->
    Stdio.eprintf "usage: %s {record,replay} logfile.sexp" argv.(0);
    Caml.exit 1
