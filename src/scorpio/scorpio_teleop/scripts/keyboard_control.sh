#!/bin/bash


gnome-terminal --title="scorpio_control" --geometry 34x10+63+305 -- bash -c "rosrun scorpio_teleop scorpio_teleop_node 0.22 1"