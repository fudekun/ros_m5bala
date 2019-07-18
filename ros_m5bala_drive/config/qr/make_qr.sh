#!/bin/bash

for i in `seq 999`
do
  rosrun ar_track_alvar createMarker -u 1 -s 240 $i
  id=`printf %03d $i`
  echo $id
  mv MarkerData_$i.png MarkerData_$id.png
done
