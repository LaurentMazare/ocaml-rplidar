open! Base
open! Rplidar

let () =
  let lidar = Lidar.create "/dev/ttyUSB0" in
  Stdio.printf "Lidar initialized!\n%!";
  let info = Lidar.Info.get lidar in
  Stdio.printf
    "Model: %d, Firmware: %d,%d, Hardware: %d\n%!"
    info.model
    (fst info.firmware)
    (snd info.firmware)
    info.hardware;
  Lidar.close lidar
