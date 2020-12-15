open Base
open Rplidar

let map_size_pixel = 512
let map_size_mm = 6000.

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

  let close _t = Graphics.close_graph ()

  let update t ~scan_points =
    Graphics.set_color Graphics.white;
    Graphics.fill_rect 0 0 t.size_x t.size_y;
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
    List.iter scan_points ~f:(fun { Slam.Pixel.xp; yp; _ } -> Graphics.plot yp xp);
    Graphics.synchronize ()
end

module Batched_slam : sig
  type t

  val create : unit -> t
  val slam : t -> Slam.t
  val process : t -> Sample.t -> [ `no_update | `update of Slam.Pixel.t list ]
end = struct
  type t =
    { mutable batch : Slam.Angle_distance.t list
    ; slam : Slam.t
    }

  let create () =
    let slam = Slam.create ~map_size_pixel ~map_size_mm in
    { slam; batch = [] }

  let slam t = t.slam

  let process t { Sample.degrees; mm } =
    let angle_distance =
      { Slam.Angle_distance.angle_degrees = degrees; distance_mm = mm }
    in
    t.batch <- angle_distance :: t.batch;
    if List.length t.batch > 200
    then (
      let scan_points = Slam.update t.slam t.batch in
      t.batch <- [];
      `update scan_points)
    else `no_update
end

let replay ~in_channel =
  let slam = Batched_slam.create () in
  let g = Batched_slam.slam slam |> G.create in
  Stdio.In_channel.iter_lines in_channel ~f:(fun line ->
      let sample = Core_kernel.Sexp.of_string line |> Sample.t_of_sexp in
      match Batched_slam.process slam sample with
      | `no_update -> ()
      | `update scan_points -> G.update g ~scan_points)

let run ~out_channel =
  let slam = Batched_slam.create () in
  let g = Batched_slam.slam slam |> G.create in
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
         G.close g;
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
            (Sample.sexp_of_t sample |> Sexp.to_string_mach);
          match Batched_slam.process slam sample with
          | `no_update -> ()
          | `update scan_points -> G.update g ~scan_points);
        `continue)
  with
  | exn ->
    Lidar.stop lidar;
    Lidar.close lidar;
    G.close g;
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
