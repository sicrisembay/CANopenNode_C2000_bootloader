#!/bin/sh

mkdir -p configs/generated
mkdir -p ../ccs_workspace
mkdir -p artifact
touch ./configs/generated/autoconf.h
/opt/ti/ccs1020/ccs/eclipse/eclipse -noSplash -data ../ccs_workspace -application "com.ti.ccstudio.apps.projectImport" -ccs.location .

configNames=$(find configs/ -type f -name "*_defconfig")
for configName in ${configNames}
do
    name=$(basename $configName)
    echo "***********************************************************"
    echo "Building for $name"
    echo "***********************************************************"
    make defconfig CONF=$name
    /opt/ti/ccs1020/ccs/eclipse/eclipse -noSplash -data ../ccs_workspace -application "com.ti.ccstudio.apps.projectBuild" -ccs.projects "CANopenNode_bootloader"
    mkdir -p artifact/$name
    cp Debug/CANopenNode_bootloader.* artifact/$name
    cp configs/$name artifact/$name/$name
done
