## Load Plugin
add this to the top of your `TeamCode` `build.gradle`
```groovy
buildscript {
    repositories {
        mavenCentral()
        maven {
            url "https://repo.dairy.foundation/releases"
        }
    }
    dependencies {
        classpath "dev.frozenmilk:Load:0.2.1"
    }
}
```

add this after the apply lines in the same file
```groovy
// there should be 2 or 3 more lines that start with apply here
apply plugin: 'dev.frozenmilk.sinister.sloth.load'
```

sync and download onto your robot via standard install

now add the gradle tasks

NOTE: if you use dashboard, install that now, then setup the gradle tasks

## Dashboard
add the dairy releases repository to your `TeamCode` `build.gradle`, above the `dependencies` block (if you already have it, no need to do so again)
```groovy
repositories {
    maven {
        url = "https://repo.dairy.foundation/releases"
    }
}
```

then add dashboard to the `dependencies` block:
```groovy
dependencies {
    implementation("com.acmerobotics.slothboard:dashboard:0.2.1+0.4.16")
}
```

NOTE: if you use a library that imports dashboard via a `implementation` or `api` dependency:

ask the library maintainers to consider changing it to `compileOnly`

change the `implementation` like so:
```groovy
implementation("com.pedropathing:pedro:1.0.8") {
   exclude group: "com.acmerobotics.dashboard"
}
```
```groovy
implementation("com.acmerobotics.roadrunner:ftc:0.1.21") {
   exclude group: "com.acmerobotics.dashboard"
}
implementation ("com.acmerobotics.roadrunner:actions:1.0.1"){
   exclude group: "com.acmerobotics.dashboard"
}
```
note that both pedro and rr require this.

_pedro and rr version numbers may not be up to date._

## Gradle Tasks

edit configurations:

![](image/edit_configurations.png)

add new configuration:

![](image/add_new_configuration.png)

select gradle:

![](image/add_new_gradle_configuration.png)

add `deploySloth` and save it:

![](image/add_deploySloth_task.png)
NOTE: android studio will not auto complete the names of these tasks, just write it and it will work.

edit TeamCode configuration:

![](image/edit_TeamCode_configuration.png)

add new gradle task:

![](image/run_gradle_task.png)

add `removeSlothRemote`:

![](image/add_removeSlothRemote_task.png)

note: type `:TeamCode` into the `Gradle Project` box to get the right contents,
do not copy mine.

put `removeSlothRemote` first and save:

![](image/ensure_order.png)

Run the deploySloth task you just added to deploy the code.


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