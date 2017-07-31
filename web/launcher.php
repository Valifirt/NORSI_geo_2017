<?php

$lat1 = $_GET['lat1'];
$lon1 = $_GET['lon1'];
$lat2 = $_GET['lat2'];
$lon2 = $_GET['lon2'];

shell_exec("./bin/launch_cuda.sh '".$lat1."' '".$lon1."' '".$lat2."' '".$lon2."'");
shell_exec("./bin/launch_cpp.sh '".$lat1."' '".$lon1."' '".$lat2."' '".$lon2."'");

?>