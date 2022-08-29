

```
 __   __     _____                           _      _ _
 \ \ / /    |  __ \                         | |    (_) |
  \ V /_____| |__) |____      _____ _ __ ___| |     _| |__
   > <______|  ___/ _ \ \ /\ / / _ \ '__/ __| |    | | '_ \
  / . \     | |  | (_) \ V  V /  __/ |  \__ \ |____| | |_) |
 /_/ \_\    |_|   \___/ \_/\_/ \___|_|  |___/______|_|_.__/


```



# ❗️❗️❗️  WARN:

```
⚠️ Please do not run the example without knowing the external load voltage of the PMU,
it may burn your external load, please check the voltage setting before running the example,
if there is any loss, please bear it by yourself
```

- Arduino library for x-powers power management series
- Through esp32 verification, other platforms have not been tested. Due to the many functions of the chip, it cannot be verified one by one. Only the functions used in the examples are tested.


# AXP2101 Notes:

- DLDO1/DLDO2 depends on AXP2101 version, not all AXP2101 supports output
- GPIO1 depends on the version of AXP2101, generally used as the feedback pin of DCDC5
- RTCLDO1 generally defaults to output 1.8V, and the adjustment function is only for special versions
- RTCLDO2 depends on AXP2101 version, not all AXP2101 support



