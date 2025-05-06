### wireless code upload setup
connect to wifi then adb commands

`adb tcpip 5555`

`adb connect 192.168.43.1:5555`

faq https://www.reddit.com/r/FTC/comments/181l7i6/connecting_to_control_hub_wirelessly_with_adb/

detailed https://blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots/#appendix-usb-wifi-models

once adb established and connected to robot wifi in 'Program Ampersand Manage', build files will automatically upload


### links and stuff
- `192.168.43.1`
- FTC Dashboard at address::8080/dash 8030/dash  ---- this for roadrunner + canvas funcs
- FTControl at :8001 + :5801 can also limelight if want separate ----- this dash better for everything else
- Robot logs at :8080/logs
- Limelight if plugged in to computer at http://limelight.local:5801