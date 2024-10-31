#!/bin/sh

ps -aux | grep point_cloud_node | grep -v grep| awk '{print $2}' | xargs kill -9
ps -aux | grep mqttControlNode| grep -v grep| awk '{print $2}' | xargs kill -9
ps -aux | grep live_human_pose | grep -v grep| awk '{print $2}' | xargs kill -9
ps -aux | grep rosnode |grep -v grep| awk '{print $2}' | xargs kill -9
