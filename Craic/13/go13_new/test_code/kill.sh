#!/bin/sh

ps -aux | grep point_cloud_node | grep -v grep|head -n 1|awk '{print $2}' | xargs kill -9
ps -auox | grep mqttControlNode | awk '{print $2}' | xargs kill -9
ps -aux | grep live_human_pose | grep -v grep| head -n 1|awk '{print $2}' | xargs kill -9
ps -aux | grep rosnode |grep -v grep|head -n 1| awk '{print $2}' | xargs kill -9
