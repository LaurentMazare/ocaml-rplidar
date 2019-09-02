open Base
open Rplidar

let csv_filename = "/tmp/dists.csv"

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
  Stdio.printf "Lidar health: %s\n%!" health;
  Stdio.Out_channel.with_file csv_filename ~f:(fun out_channel ->
      let cnt = ref 0 in
      Stdio.Out_channel.fprintf out_channel "quality,angle,dist\n%!";
      Lidar.Scan.run lidar ~f:(fun { quality; angle; dist } ->
          Stdio.Out_channel.fprintf out_channel "%d,%f,%f\n%!" quality angle dist;
          Int.incr cnt;
          if !cnt >= 10_000 then `break else `continue));
  Lidar.close lidar
