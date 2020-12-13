open Base
open Rplidar

let map_size_in_pixels = 1024
let map_size_in_meters = 6000.

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

  let update t =
    (* TODO: efficient plotting. *)
    for i = 0 to map_size_in_pixels - 1 do
      for j = 0 to map_size_in_pixels - 1 do
        let v = t.map_memory.{i, j} in
        Graphics.set_color (Graphics.rgb v v v);
        Graphics.plot i j
      done
    done;
    Graphics.synchronize ()
end

let () =
  let slam = Slam.create ~map_size_in_pixels ~map_size_in_meters in
  let g = G.create slam in
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
         G.close g;
         Caml.exit 1));
  let last_angle = ref 0. in
  let batch = ref [] in
  Lidar.Scan.run lidar ~f:(fun { quality; angle; dist } ->
      (* TODO: type-safe way to handle the zero distance? *)
      if quality > 0
      then
        batch
          := { Slam.Angle_distance.angle_degrees = angle; distance_mm = dist } :: !batch;
      if Float.( < ) angle !last_angle
      then
        if not (List.is_empty !batch)
        then (
          Slam.update slam !batch;
          batch := [];
          G.update g);
      last_angle := angle;
      `continue)
