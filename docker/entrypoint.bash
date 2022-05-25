#! /bin/bash
service ssh restart
source /opt/ros/melodic/setup.bash
exec $@
