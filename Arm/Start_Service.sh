#!/bin/bash

LOG_TIME=$(date "+%Y-%m-%d %H:%M:%S")
echo "[$LOG_TIME] Checking Service status..." >> /root/Start_Service.log

if systemctl is-active --quiet jupyterlab; then
    echo "[$LOG_TIME] Jupyter Lab Service is already running, no action needed." >> /root/Start_Service.log
else
    echo "[$LOG_TIME] Jupyter Lab Service not running, starting now..." >> /root/Start_Service.log
    systemctl start jupyterlab >> /root/Start_Service.log
fi

if systemctl is-active --quiet yahboom_oled; then
    echo "[$LOG_TIME] Yahboom OLED Service is already running, no action needed." >> /root/Start_Service.log
else
    echo "[$LOG_TIME] Yahboom OLED Service not running, starting now..." >> /root/Start_Service.log
    systemctl start yahboom_oled >> /root/Start_Service.log
fi
