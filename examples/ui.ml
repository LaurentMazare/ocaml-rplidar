open Base
open Rplidar

module G = struct
  type t =
    { size_x : int
    ; size_y : int
    ; scale : float
    }

  let create () =
    Graphics.open_graph "";
    Graphics.set_color Graphics.blue;
    Graphics.auto_synchronize false;
    { size_x = Graphics.size_x (); size_y = Graphics.size_y (); scale = 0.1 }

  let close _t = Graphics.close_graph ()

  let synchronize t =
    Graphics.synchronize ();
    Graphics.set_color Graphics.white;
    Graphics.fill_rect 0 0 t.size_x t.size_y;
    Graphics.set_color Graphics.blue

  let plot t ~angle ~dist =
    let angle = angle /. 180. *. Float.pi in
    let x = dist *. Float.cos angle *. t.scale in
    let y = dist *. Float.sin angle *. t.scale in
    let x = Int.of_float x + (t.size_x / 2) in
    let y = Int.of_float y + (t.size_y / 2) in
    if 0 <= x && x < t.size_x && 0 <= y && y < t.size_y then Graphics.fill_circle x y 3
end

let () =
  let g = G.create () in
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
  let rotations = ref 0 in
  Lidar.Scan.run lidar ~f:(fun { quality; angle; dist } ->
      if quality > 0 then G.plot g ~angle ~dist;
      if Float.( < ) angle !last_angle
      then (
        Int.incr rotations;
        if !rotations % 50 = 0 then G.synchronize g);
      last_angle := angle;
      `continue)
