![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/LK8000-emu.jpg)

<br>

Emulator is currently implemented as a background Python process being running
on an Android or Raspberry Pi host.

## List of functions:

1) The emulator takes current GNSS position, time stamp and ID to encode them into raw data packet.
Then it gives the outbound packet via Wi-Fi to SoftRF Prime for actual transmittion into air.

2) The emulator does also 
 - receive inbound raw data packets from SoftRF Prime;
 - apply decoding algorithm ;
 - convert decoded data into a fraction of FLARM datalink format ; 
 - deliver the NMEA datalink strings to localy running XCSoar or LK8000 app via UDP stream.

## Prerequisites:

1) have "QPython - Python for Android" App installed from Google Play Market ;
2) "numpy" Python library has to be installed over QPython ;

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/qpython-lib.jpg)

3) SoftRF-Emu .py files from here: https://github.com/lyusupov/SoftRF/tree/master/software/app/Emulator
   has to be transferred into QPython scripts 'home' directory of your Tablet/Cellphone ;

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Emulator-files.jpg)

4) configure your XCSoar or LK8000 data source onto UDP port number 4353.   

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Prime-XCSoar-Settings.jpg)

## Instructions:

1) set the prerequisites above up ;
2) enable your Tablet/Cellphone GNSS ;
3) connect your Tablet/Cellphone to SoftRF Wi-Fi Access Point. <br>
   Default SSID: SoftRF . Default password: 12345678 . URL: http://192.168.4.1/ ;
4) put your "Prime" into "Bridge" mode ;

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/firmware-settings-bridge.jpg)

5) execute "Emulator.py" within QPython ;
6) if no errors appeared at step 5, launch your XCSoar or LK8000 app.

## Optional:

In addition to traditional NMEA data protocol the Emulator is able to speak in Garmin GDL90 language as well.<br>
One known good example of compatible EFB applications is "Avare".

At first, you will need to install and set up an "Avare plug-in" as follows:

![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Avare-plugin.jpg)


"Avare" app will show the FLARM traffic on a moving map:


![](https://github.com/lyusupov/SoftRF/raw/master/documents/images/Avare.jpg)
