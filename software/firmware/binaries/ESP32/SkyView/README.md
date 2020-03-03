
## Release notes

### revision 0.10

#### New features

- Bluetooth SPP and LE connection methods. Mostly applicable for a 'SoftRF on ESP32' partner - other devices may or may not work ;
- serial data input over EZ built-in micro-USB port ;
- one more radar view's zoom level ;
- 'Wi-Fi off' timer option ;
- traffic filter (by altitude) ;
- team member's aircraft (if any) is depicted in a bit different manner ;
- dual boot option ( demo application is [iArradio](https://github.com/TioRuben/iArradio) ).

#### Known issues and limitations

- same as in 0.9, plus
- first Bluetooth SPP connection may cause restart of a partner SoftRF data source device ;
- due to high RAM memory usage, Bluetooth SPP may coexist with either voice or aircraft's data option but not both ;
- only Bluetooth 'Simple Pairing' (no key) method is currently supported.

#### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'Web Update' UI feature does not work in previous 0.9 revision. Use generic ('cable') method instead.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

&nbsp;&nbsp;&nbsp;&nbsp;**Dual boot:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow [these instructions](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Dual-boot).<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**NOTICE!**: [**iArradio**](https://github.com/TioRuben/iArradio) firmware that comes with it is a DEMO. I will reject your every claim or question.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;If you need a custom weather, playlist or timezone - build your own firmware file by yourself.<br>

### revision 0.9

#### Known issues and limitations

- maximum number of tracked objects is limited to 9 ;
- very short keypress may not work. Make a more distinct one ;
- keypress may not work when audio is playing ;
- view mode change may cause 2-3 seconds of '**NO DATA**' or '**NO FIX**' warning ;
- 'anti-ghosting' full screen refresh may cause 2-3 seconds of '**NO DATA**' or '**NO FIX**' warning ;
- '**NO DATA**' or '**NO FIX**' warning may appear right after voice traffic alert message ;
- **VOICE2** and **VOICE3** may have some WAV files missing ;
- Wi-Fi re-connect may fail sometimes. Reset of SoftRF server and/or SkyView client does typically help ;
- does not work with Stratux yet due to issues with DHCP leases and GDL90 data (over-)flow.

### revision 0.8

Very first deployment of SkyView's firmware binaries.
