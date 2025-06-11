### sloth
https://github.com/Dairy-Foundation/Sloth?tab=readme-ov-file#gradle-tasks

### Axes directions

x+ forward
y+ left
angle+ anti-clockwise

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
- Robot logs at :8080/logs
- 
[//]: # (- FTControl at :8001 + :5801 can also limelight if want separate ----- this dash better for everything else)

[//]: # (- Limelight if plugged in to computer at http://limelight.local:5801)

---

### LimeLight Tools

With these tools you can:

    Set the LimeLight to provide the image source for the drivers station "camera stream" during Init
    Use port forwarding through control hub to access the limelight configuration tool remotely.
    If using Ftc Dash Board, this will stream the limelight camera to dashboard

See the comments in the following files located in the team directory on how to use these tools

    LimeLightToolsTest.java provides an example of setting up the limelight and using the tools class to display images on drivers station and dashboard
    LimeLightImageTools.java the class of tools for getting images from the limelight


**Disclamer and Official event warning**

## Disclaimer - intended for development only. Not recommended for competition or official events. This code has not been tested in event environments and may cause unexpected issues! Using the port forwarding for configuration or streaming to dashboard may use significant bandwidth and cause problems with other robots in a dense competition setting.

We do think that the setDriverStationStreamSource() method utilizes minimal bandwidth, sending single snapshots as intended by the drivers station app and FTC SDK. Its use may be appropriate at event venues.

Following is from any email with Danny Diaz, FTC Senior Engineering Manager "With that in mind, we do have an important request regarding the use of this particular tool at official FTC events. Due to the very limited and critical nature of Wi-Fi bandwidth at our events, tools that place a significant burden on the network can unfortunately impact the overall event experience for everyone. Therefore, we would greatly appreciate it if you could prominently include a statement in your repository explicitly advising users that this tool should absolutely not be used at official FTC events. This includes the competition area, pit area, and practice fields. It's crucial to emphasize that using such a tool at an event is forbidden due to the potential strain on the event's network infrastructure. This is a policy we maintain across the board, and it's the same reason we don't allow streaming to the Driver Station." */