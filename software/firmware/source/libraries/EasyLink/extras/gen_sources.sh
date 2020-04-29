#!/bin/bash

# ./gen_sources.sh /Applications/ti/ccs920/ ~/ti/simplelink_cc13x2_26x2_sdk_3_40_00_02/

FILES=../src/syscfg/*
rm $FILES

"$1/ccs/utils/sysconfig/sysconfig_cli.sh" -s "$2/.metadata/product.json" -o "../src/syscfg" "EasyLink.syscfg"

for F in $FILES
do
  echo "Processing $F file..."
  gsed -i "1s/^/#ifdef ENERGIA_ARCH_CC13X2\n/" ${F}
  sh -c "echo '#endif // ENERGIA_ARCH_CC13X2' >> ${F}"
done

rm ../src/syscfg/ti_drivers_config.*

#Correct #include
gsed -i '/#include <ti_radio_config.h>/c\#include \"ti_radio_config.h\"' ../src/syscfg/ti_easylink_config.c
gsed -i '/#include \"easylink\/EasyLink.h\"/c\#include \"easylink\/EasyLinkv2.h\"' ../src/syscfg/ti_easylink_config.c

