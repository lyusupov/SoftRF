#!/bin/bash

#run e.g. ./copy_smartrf_settings.sh /Applications/ti/tirtos_cc13xx_cc26xx_2_21_00_06/examples/GNU/

echo "Copying smartrf_settings"

dst_dir=../src/smartrf_settings

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    exit 0;
fi

if [ ! -d "$1/CC1310_LAUNCHXL" ]; then
  "The smartrf_directory does not exist in $1. This does not seem to be a valid TI-RTOS dir."
  exit 0
fi

for D in `find ${dst_dir} -mindepth 1 -type d`
do
    echo "Coping files for $(basename ${D})"
    cp ${1}/$(basename ${D})/rfEasyLinkTx/smartrf_settings/* ${D}/

    for F in `find ${dst_dir}/$(basename ${D}) -mindepth 1 -type f`
    do
        gsed -i "1s/^/#ifdef ENERGIA_$(basename ${D})\n/" ${F}
        sh -c "echo '#endif // ENERGIA_$(basename ${D})' >> ${F}"
        echo ${F}
    done
done
