open Base
open Rplidar

let () =
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
  Stdio.eprintf "Lidar health: %s\n%!" health;
  let cnt = ref 0 in
  Stdio.printf "quality,angle,dist\n%!";
  Lidar.Scan.run lidar ~f:(fun { quality; angle; dist } ->
      Stdio.printf "%d,%f,%f\n%!" quality angle dist;
      Int.incr cnt;
      if !cnt >= 10_000 then `break else `continue);
  Lidar.close lidar
