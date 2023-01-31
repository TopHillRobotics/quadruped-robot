#!/bin/bash


# Index of first scene
first=$1
# Total number of scenes to generate
scenes=$2
# Number of consecutive scenes generated
increment=$3
# Output directory
output=$4

source setup.sh

for (( i=first; i<scenes; i+=increment))
do
    let "final=($i+1)*$increment" 
    gzserver ./worlds/spawner.world &>/dev/null &
    sleep 2s
    ./build/bin/scene_example -i "$output/images/" -d "$output/annotations/" -s $increment -n $i
    pkill -f gz
    sleep 2s
done
