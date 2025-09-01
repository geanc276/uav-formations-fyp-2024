#!/bin/bash
cd ROMFS/px4fmu_common/init.d

read -p "Enter drone number: " CUSTOM_NUMBER

if test $CUSTOM_NUMBER -eq -1; then
    sed -i '/uxrce_dds_client/c\uxrce_dds_client start -p 8888 -h 10.41.10.1 -n target' rc.mc_apps
else
    sed -i '/uxrce_dds_client/c\uxrce_dds_client start -p 8888 -h 10.41.10.1 -n px4_'$CUSTOM_NUMBER'' rc.mc_apps
fi

cd ../../..

make px4_fmu-v5x_default upload
