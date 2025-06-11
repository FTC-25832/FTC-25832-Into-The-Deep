

Hide navigation sidebar
Hide table of contents sidebar
Skip to content
Dark Logo

Game Manual 0 Table of Contents

    Mission Statement
    Start Here
    Know Your Lingo
    Being a Team

Design Skills
Hardware Components
Custom Manufacturing
Common Mechanisms
Electronics and Motion Components
Software
Getting Started
Tutorials

    Mecanum TeleOp
    Gamepad Usage
    Using Telemetry
    Encoders
    Bulk Reads
    Computer Vision

Programming Concepts
Advanced Control System

    Software Glossary

Awards
Useful Resources
Appendix
Contributor’s Guide

View this page
Toggle table of contents sidebar
Bulk Reads

Bulk reads are a LynxCommand that reads all sensor values (except I2C) on a hub at once. This takes the same amount of time to execute as any other LynxCommand, and can therefore save a lot of time in the execution loop; with a bulk read, reading ten sensors takes as much time as reading one sensor (if they are not I2C and are on the same hub).

This became much simpler to do with SDK versions 5.4 and above, with a built-in way to easily access it. Here is the official example on how to use bulk reads.
Off Mode

This is the default, and the most boring; it means bulk reads are not used by the SDK when calling normal hardware-access methods.

Note

Bulk reads can still be accessed by calling the LynxModule.getBulkInputData() method, however if one wishes to use bulk reads (which we highly recommend) using AUTO or MANUAL modes is simpler.

To manually set OFF mode, you need to run

List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

for (LynxModule hub : allHubs) {
   hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
}

Auto Mode

This is the simplest mode to use that utilizes bulk reads; a new bulk read is done when a hardware read is repeated. As an example of this

List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

for (LynxModule hub : allHubs) {
   hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
}

while (opModeIsActive()) {
   // Will run one bulk read per cycle; however, if e.g.
   // frontLeftMotor.getCurrentPosition() was called again,
   // a new bulk read would be issued
   int frontLeftEncoderPos = frontLeftMotor.getCurrentPosition();
   int frontRightEncoderPos = frontRightMotor.getCurrentPosition();
   int backLeftEncoderPos = backLeftMotor.getCurrentPosition();
   int backRightEncoderPos = backRightMotor.getCurrentPosition();
}

However, this can be problematic, if the same hardware read is called more than once in a given loop; an example of this

List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

for (LynxModule hub : allHubs) {
   hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
}

while (opModeIsActive()) {
   // Will run two bulk read per cycles,
   // as frontLeftMotor.getCurrentPosition() is called twice
   int frontLeftEncoderPos = frontLeftMotor.getCurrentPosition();
   int frontLeftEncoderPos2 = frontLeftMotor.getCurrentPosition();
}

Overall, this is recommended, as it is very unlikely to mess anything up and can give significant performance improvements for little effort. On the user side, one does not need to manually flush the bulk read cache; however, this means you lose some control.
Manual Mode

In manual mode the cache for bulk reads is only reset once manually reset. This can be useful, as it is the way to absolutely minimize extraneous reads, however if the cache is not reset, stale values will be returned. That being said, here’s a proper implementation of MANUAL mode

List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

for (LynxModule hub : allHubs) {
   hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
}

while (opModeIsActive()) {
   // Will run one bulk read per cycle,
   // even as frontLeftMotor.getCurrentPosition() is called twice
   // because the caches are being handled manually and cleared
   // once a loop
   for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
   }

   int frontLeftEncoderPos = frontLeftMotor.getCurrentPosition();
   int frontLeftEncoderPos2 = frontLeftMotor.getCurrentPosition();
}

Warning

When in MANUAL mode, if the cache is not cleared appropriately, stale values will be returned. For that reason, if you are not quite sure what you are doing, we recommend AUTO mode; while MANUAL mode can have some performance improvements if AUTO mode is not used optimally, it has less room for catastrophic error.

Tip

Bulk reads are issued per hub, so you don’t necessarily need to bulk read both hubs at the same time or every loop.
Next
Computer Vision
Previous
Encoders
Copyright © Game Manual 0 Contributors | Hosting Graciously Provided By //Copperforge
Made with Sphinx and @pradyunsg's Furo
On this page

        Off Mode
        Auto Mode
        Manual Mode



Hide navigation sidebar
Hide table of contents sidebar
Skip to content
Dark Logo

Game Manual 0 Table of Contents

    Mission Statement
    Start Here
    Know Your Lingo
    Being a Team

Design Skills
Hardware Components
Custom Manufacturing
Common Mechanisms
Electronics and Motion Components
Software
Getting Started
Tutorials
Programming Concepts

    Control Loops
    Finite State Machines
    Kinematics
    Odometry

Advanced Control System

    Software Glossary

Awards
Useful Resources
Appendix
Contributor’s Guide

View this page
Toggle table of contents sidebar
Finite State Machines

Finite State Machines (FSM) are often used while programming in order to allow for more complex series of actions. This is especially useful when one needs multiple tasks to run at the same time, because it allows for tasks to depend on each other’s execution in a non-linear fashion.
What is a Finite State Machine?

The name of a finite state machine is very descriptive; it’s a state machine, with a finite number of states. It can be in one state at a time, and can transition to a different state once something happens. The Wikipedia example of a turnstile explains the concept very well.
Implementation
Naive Implementation

When first learning about FSMs, it is quite common for programmers to try and use them. Often times, they try to apply an FSM to their autonomous programs by segmenting their autonomous into a giant switch statement, and it often looks something like this:

while (opModeIsActive()) {
    switch (state) {
        case DETECT_SKYSTONE:
            // skystone detection code here
            int position = detectSkystone();

            if (position == 0) {
                state = SKYSTONE_POS_0;
            }
            else if (position == 1) {
                state = SKYSTONE_POS_1;
            }
            else {
                state = SKYSTONE_POS_2;
            }
            break;
        case SKYSTONE_POS_0:
            // skystone position 0 here
            doSkystone(0);
            state = MOVE_FOUNDATION;
            break;
        case SKYSTONE_POS_1:
        case SKYSTONE_POS_2:
            // etc etc
            break;
        case MOVE_FOUNDATION:
            // foundation move code
            state = PARK;
            break;
        case PARK:
            // park the bot
            break;
     }
}

This however does not really have any benefits compared to if the programmer had simply put each of the code’s segments into functions, and executed them in order. In fact, often times, programmers will structure their code like this instead of splitting their code up into functions. The result is an autonomous that’s more difficult to debug, and ultimately harder to fix on the fly during a competition or other time crunch.

If one drew out the state transition diagram for each of the states, for the autonomus above it’d be very linear, and the state transitions always occur because the section of the code finished:
digraph { detect[label="Detect Skystone"]; posZero[label="Skystone Position 0"]; posOne[label="Skystone Position 1"]; posTwo[label="Skystone Position 2"]; move[label="Foundation Move"]; park[label="Park"]; detect->posZero; detect->posOne; detect->posTwo; posZero->move; posOne->move; posTwo->move; move->park; }

In fact, in many implementations, making state transitions for any other reason is often difficult because the code executes linearly and is only in a loop to rerun the switch statements. (Often times, this means the code has a hard time reacting to a stop request in the middle of autonomous.)

Warning

It is unadvisable to write code like this. If your autonomous is synchronous, it is preferable to split your code up into functions and run them in order, as this will be easier to understand and edit on the fly.
Useful Implementation

FSMs are the right tool to use when a robot needs to complete multiple tasks at once; a common example of this is when a robot should have automation in teleop, but still have control over the drivetrain.

Often times, teams have issues because their teleop executes in a loop and their servo logic has sleeps in it. But, we can avoid this if we write code in an asynchronous fashion - where instead of waiting for a task to complete before doing the next one, tasks are performed at the same time, and each task’s state is checked without stopping the other tasks from executing.

An example of this would be that if one had a robot similar to Gluten Free’s Rover Ruckus Robot, and one wanted to automate the scoring lift so that the drivers don’t have to think while the bot deposits the minerals. There are two parts of the bot that are relevant to us in this exercise: the angled scoring lift, and the servo that tips the dumper so the minerals fall out. The goal is to be able to push a button, and then the bot will:

    extend the lift,

    at full lift extension, angle the mineral bucket servo to deposit the minerals,

    wait for the minerals to fall out,

    reset the servo to the original position

    retract the lift

If the drivers press a specific other button, we will stop executing the actions above as a failsafe - in case the robot is breaking somehow and the drivers need to take manual control. All the while, the drivers should still be able to control our drivetrain so we can make adjustments. Now, of course, this is a bit simplified (and probably not entirely what GF did), but it will do for now.

Before anything is programmed, it may be useful draw out the state diagram for this to get a better understanding of what we the robot should actually be doing. This can also add to a Control Award submission.
digraph { start[label="Start"]; extend[label="Extend Lift"]; dump[label="Set Servo Dump"]; reset[label="Reset Servo, Retract Lift"]; start->extend[label="X Pressed"]; extend->dump[label="Lift Fully Extended"]; extend->start[label="Y Pressed"]; dump->start[label="Y Pressed"]; dump->reset[label="Minerals be Dumped"]; reset->start[label="Lift Fully Retracted/Y Pressed"]; }

Notice how resetting the dump servo and retracting the lift share a state. That’s because the robot doesn’t need to wait for the servo to reset before moving the lift down; they can both happen at once.

Now, let’s get into actually implementing the code for this. In a traditional OpMode, which is commonly used for teleop, code runs repeatedly in a loop() function, so instead of waiting for a state transition to happen directly, the code will repeatedly check on each loop() call if it should perform a state transition. This kind of “update our state” pattern keeps code from blocking the rest of the loop() code from running, such as the drivetrain.

/*
 * Some declarations that are boilerplate are
 * skipped for the sake of brevity.
 * Since there are no real values to use, named constants will be used.
 */

@TeleOp(name="FSM Example")
public class FSMExample extends OpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode
    public DcMotorEx liftMotor;

    // the dump servo
    public Servo liftDump;
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    final double DUMP_IDLE; // the idle position for the dump servo
    final double DUMP_DEPOSIT; // the dumping position for the dump servo

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME;

    final int LIFT_LOW; // the low encoder position for the lift
    final int LIFT_HIGH; // the high encoder position for the lift

    public void init() {
        liftTimer.reset();

        // hardware initialization code goes here
        // this needs to correspond with the configuration used
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftDump = hardwareMap.get(Servo.class, "liftDump");

        liftMotor.setTargetPosition(LIFT_LOW);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {
        liftMotor.setPower(1.0);

        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.x) {
                    // x is pressed, start extending
                    liftMotor.setTargetPosition(LIFT_HIGH);
                    liftState = LiftState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                 // check if the lift has finished extending,
                 // otherwise do nothing.
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    liftDump.setTargetPosition(DUMP_DEPOSIT);

                    liftTimer.reset();
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    liftDump.setTargetPosition(DUMP_IDLE);
                    liftMotor.setTargetPosition(LIFT_LOW);
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                 // should never be reached, as liftState should never be null
                 liftState = LiftState.LIFT_START;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad1.y && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
        }

        // mecanum drive code goes here
        // But since none of the stuff in the switch case stops
        // the robot, this will always run!
        updateDrive(gamepad1, gamepad2);
   }
}

Next
Kinematics
Previous
Control Loops
Copyright © Game Manual 0 Contributors | Hosting Graciously Provided By //Copperforge
Made with Sphinx and @pradyunsg's Furo
On this page

        What is a Finite State Machine?
        Implementation
            Naive Implementation
            Useful Implementation

