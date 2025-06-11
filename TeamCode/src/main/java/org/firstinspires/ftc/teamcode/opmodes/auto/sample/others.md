Other teams auton code:
package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Autonomous
public class BlueRRLeft0plus7Auto extends LinearOpMode {

    public enum GrabFromSubmersibleState {
        SEARCHING,
        APPROACHING_HEADING,
        APPROACHING,
        INTAKING_1,
        RETRACTING,
        INTAKING_2,
        EJECTING,
        RESETTING,
        RESETTING_2
    }

    private double targetHeading = Math.toRadians(180);
    private double prevHeading = Math.toRadians(180);

    private final double maxGrabAngle = Math.toRadians(200);
    private final double minGrabAngle = Math.toRadians(155);

    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    VisionSubsystem vision;
    MasterThread masterThread;
    Telemetry.Item loopTimeTelem;

    Boolean blueAlliance = true;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    double extensionDistance = 0;

    private final com.reefsharklibrary.data.Vector2d holdPoint = new com.reefsharklibrary.data.Vector2d(25, 9);

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private final ElapsedTimer totalAutoTimer = new ElapsedTimer();

    double timeThreshold = -1;

    double loopTime = 0;

    boolean skipScore = false;

    double prevExtensionError = 0;

    enum MoveToScoreState {
        DELAY,
        DRIVING_TO_BASKET,
        DROPPING
    }

    enum MoveToGrabState {
        FIRST_LOOP,
        DRIVING_TO_SUB_1,
        DRIVING_TO_SUB_2

    }

    enum ParkState {
        FIRST_LOOP,
        MOVE_TO_SUB_1,
        MOVE_FROM_SUB_1,
        MOVE_TO_SUB_2
    }


    @Override
    public void runOpMode() throws InterruptedException {
        loopTimeTelem = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), outtake, intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);


        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance, false);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake,
                vision
        );

        Action intakeBlock = new IntakeBlock();


        Action preload = drivetrain.drive.actionBuilder(new Pose2d(38.93, 60.23, Math.toRadians(180)))
                .setTangent(Math.toRadians(310))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                    outtake.setClawPitch(NewOuttake.ClawPitch.UP.pos);
                })
                .afterTime(.6, () -> {//0
                    intake.setTargetSlidePos(13.5);
                    extensionDistance = 13.5;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);

                })
                .afterTime(.89, () -> {
                    outtake.setClawPitch(NewOuttake.ClawPitch.BACK2.pos);
                })
                .splineToLinearHeading(new Pose2d(61.5, 56, Math.toRadians(254)), Math.toRadians(0))
                .build();


        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(61.5, 56, Math.toRadians(254)))
                .setTangent(Math.toRadians(254))
                .afterTime(0.2, () -> {
//                    intake.setTargetSlidePos(9);
//                    extensionDistance = 9;
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToConstantHeading(new Vector2d(58, 51.5), Math.toRadians(261))
//                .splineToLinearHeading(new Pose2d(50.5, 50, Math.toRadians(270)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock1 = drivetrain.drive.actionBuilder(new Pose2d(58, 51.5, Math.toRadians(261)))
                .setTangent(Math.toRadians(84))
                .splineToLinearHeading(new Pose2d(62, 54, Math.toRadians(265)), Math.toRadians(85))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(62, 54, Math.toRadians(265)))
                .afterTime(0.2, () -> {
                    autoTimer.reset();
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                })
                .waitSeconds(.1)
                .setTangent(265)
                .splineToConstantHeading(new Vector2d(61, 51), Math.toRadians(265))
                .build();

        Action moveToScoreBlock2 = drivetrain.drive.actionBuilder(new Pose2d(61, 51, Math.toRadians(265)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(63, 54.5, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(63, 54.5, Math.toRadians(270)))
                .afterTime(.5, () -> {
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .waitSeconds(.1)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(63, 50, Math.toRadians(282)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock3 = drivetrain.drive.actionBuilder(new Pose2d(63, 50, Math.toRadians(282)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62.5, 55, Math.toRadians(270)), Math.toRadians(90))
                .build();


        Action park = new Action() {
                ParkState parkState = ParkState.FIRST_LOOP;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                    switch (parkState) {
                        case FIRST_LOOP:
                            intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                            intake.setIntakeState(NewIntake.IntakeState.RETRACTING_INTAKE);
                            intake.setTargetSlidePos(0);
                            extensionDistance = 0;

                            if (drivetrain.getPoseEstimate().getX()>40) {
                                //park from bucket
                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(32, holdPoint.getY(), Math.toRadians(235)));
                                parkState = ParkState.MOVE_TO_SUB_1;
                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                            } else {
                                parkState = ParkState.MOVE_FROM_SUB_1;
                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(32, holdPoint.getY(), Math.toRadians(180)));
                            }

                            autoTimer.reset();
                            return true;
                        case MOVE_TO_SUB_1:
                            if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(8, 8, Math.toRadians(8))))
                                    || autoTimer.seconds()>1.5) {

                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(21, 7, Math.toRadians(180)));
                                parkState = ParkState.MOVE_TO_SUB_2;

                            }
                            return true;
                        case MOVE_FROM_SUB_1:
                            if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(2, 8, Math.toRadians(8))))
                                    || autoTimer.seconds()>1.5) {
                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(19, 7, Math.toRadians(180)));
                                parkState = ParkState.MOVE_TO_SUB_2;

                            }
                            return true;
                        case MOVE_TO_SUB_2:
                            if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(2, 1.5, Math.toRadians(3))) && drivetrain.getPoseVelocity().inRange(new com.reefsharklibrary.data.Pose2d(3, 3, Math.toRadians(4)))) {
                                return  false;
                            }
                            return true;

                    }

                    return false;
                }
            };
//hi brett
        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        intake.setEjectOnIntake(false);

        waitForStart();

        totalAutoTimer.reset();

        drivetrain.drive.setPoseEstimate(new Pose2d(38.93, 60.23, Math.toRadians(180)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_BEHIND);

        drivetrain.followPath(new SequentialAction(
                new RunToTimeThreshold(
                        new SequentialAction(
                                preload,
                                new ScoreBlock(),
                                moveToGrabBlock1,
                                intakeBlock,
                                new InstantAction(() -> {
                                    extensionDistance = 15;
                                }),
                                new ParallelAction(
                                    moveToScoreBlock1,
                                    new ExtendWhenTransfer()
                                ),
                                new ScoreBlock(),
                                moveToGrabBlock2,
                                intakeBlock,
                                new InstantAction(()-> {
                                    extensionDistance = 11.5;
                                }),
                                new ParallelAction(
                                    moveToScoreBlock2,
                                    new ExtendWhenTransfer()
                                ),
                                new ScoreBlock(),
                                moveToGrabBlock3,
                                intakeBlock,
                                moveToScoreBlock3,
                                new ScoreBlock(),
                                new InstantAction(() -> {
                                    intake.setEjectOnIntake(true);
                                }),
                                new SleepAction(.1),

                                new moveToGrabFromSub(),
                                new GrabFromSubmersible(),
                                new moveToScoreFromSub(),

                                new moveToGrabFromSub(),
                                new GrabFromSubmersible(),
                                new moveToScoreFromSub(),

                                new moveToGrabFromSub(),
                                new GrabFromSubmersible(),
                                new moveToScoreFromSub()
                        )
                ),
                park
        ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime = loopTimer.seconds();
            loopTimeTelem.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }

//        intake.overrideSpinOut();
//
//        drivetrain.stopMotors();
//        intake.stopMotors();
//        outtake.stopMotors();
//
//        sleep(250);
    }

    public class RunToTimeThreshold implements Action {
        Action action;

        public RunToTimeThreshold(Action action) {
            this.action = action;
        }

        boolean breakMotorsAct = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (30 - totalAutoTimer.seconds() < 0.1 && !breakMotorsAct) {
                drivetrain.breakMoters();
                breakMotorsAct = true;
            }
            return action.run(telemetryPacket) && ((30-totalAutoTimer.seconds())>timeThreshold);
        }
    }

    public class IntakeBlock implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                return false;
            } else if (autoTimer.seconds()>2) {
                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
//                extensionDistance = Math.max(extensionDistance-5, 0);
//                intake.setTargetSlidePos(extensionDistance);
//                skipScore = true;
                autoTimer.reset();
                return false;
            } else {
                extensionDistance = MathUtil.clip(extensionDistance + 4 * Math.max(.015, loopTime), -.5, 18.5);
                intake.setTargetSlidePos(extensionDistance);
                return true;
            }
        }
    }

    public class ScoreBlock implements Action {
        private boolean firstLoop = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                if ((skipScore && !intake.isBreakBeam()) || (!intake.isBreakBeam() && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER))) {//!intake.isBreakBeam() && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER)
                    skipScore = false;
                    autoTimer.reset();
                    return false;
                } else {
                    skipScore = false;
                    firstLoop = false;
                    return true;
                }
            } else {
                if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer()) {// || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    autoTimer.reset();
                    skipScore = false;
                    return false;
                } else {
                    return true;
                }
            }
        }
    }

    public class ExtendWhenTransfer implements Action {
        boolean notExtended = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (notExtended) {
                if((outtake.getOuttakeState() == NewOuttake.OuttakeState.EXTENDING_PLACE_BEHIND || outtake.getOuttakeState() == NewOuttake.OuttakeState.EXTENDING_V4BAR_PLACE_BEHIND || outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND)) {
                    notExtended = false;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.STOP_INTAKING);
                    intake.setTargetSlidePos(extensionDistance);
                    return false;
                } else if ((!intake.isBreakBeam() && autoTimer.seconds() > .5 && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER || intake.getPrevIntakeState() == NewIntake.IntakeState.INTAKING))) {
                    notExtended = false;
                    skipScore = true;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.STOP_INTAKING);
                    intake.setTargetSlidePos(extensionDistance);
                    return false;
                }
            }

            return true;
        }
    }

    public class moveToScoreFromSub implements Action {
        MoveToScoreState moveToScoreState = MoveToScoreState.DELAY;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (moveToScoreState) {
                case DELAY:
                    if (autoTimer.seconds()>.35) {
                        moveToScoreState = MoveToScoreState.DRIVING_TO_BASKET;
                        drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(59.5, 55, Math.toRadians(235)));
                    }
                    return true;
                case DRIVING_TO_BASKET:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(3, 3, Math.toRadians(4))) && drivetrain.getPoseVelocity().inRange(new com.reefsharklibrary.data.Pose2d(6, 6, Math.toRadians(8)))
                     && (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer())
                    ) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        autoTimer.reset();
                        moveToScoreState = MoveToScoreState.DROPPING;

                    }
                    return true;
                case DROPPING:
                    if (.1 < autoTimer.seconds()) {
                        drivetrain.cancelHoldPoint();
                        timeThreshold = 2.5;
                        return  false;
                    }
                    return true;
            }

            return false;
        }
    }

    public class moveToGrabFromSub implements Action {
        MoveToGrabState moveToGrabState = MoveToGrabState.FIRST_LOOP;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (moveToGrabState) {
                case FIRST_LOOP:
                    drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(32, holdPoint.getY()+2, Math.toRadians(235)));// holdPoint.toPose(Math.toRadians(235)));
                    autoTimer.reset();

                    moveToGrabState = MoveToGrabState.DRIVING_TO_SUB_1;

                    intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                    extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;

                    return true;
                case DRIVING_TO_SUB_1:
                    if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(10, 10, Math.toRadians(10))))
                            || autoTimer.seconds()>1.5) {

                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
                        moveToGrabState = MoveToGrabState.DRIVING_TO_SUB_2;

                    }
                    return true;
                case DRIVING_TO_SUB_2:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(2, 2, Math.toRadians(7))) && drivetrain.getPoseVelocity().inRange(new com.reefsharklibrary.data.Pose2d(4, 4, Math.toRadians(6)))) {
                        drivetrain.cancelHoldPoint();
                        vision.recenter();
                        return  false;
                    }
                    return true;
            }

            return false;
        }
    }

    public class GrabFromSubmersible implements Action {
        BlueRRLeft0plus7Auto.GrabFromSubmersibleState grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;

        double holdAngle = Math.toRadians(180);


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_EJECTING || intake.getPrevIntakingState() == NewIntake.IntakingState.START_EJECTING) {
                grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.EJECTING;
            }

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING_A_LITTLE_MORE || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING_SPIN_OUT || intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_INTAKING) {
                timeThreshold = -1;
                drivetrain.cancelHoldPoint();
                autoTimer.reset();
                intake.toIntakeState(NewIntake.ToIntakeState.IDLE);
                return false;
            }


            switch (grabFromSubmersibleState) {
                case SEARCHING:
                    if (vision.hasSample()) {
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*1, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING;

                        extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset()-2.4, 2.5);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
                        prevExtensionError = 0;
                        intake.setTargetSlidePos(extensionDistance);

                        autoTimer.reset();

                        // intake.setTargetSlidePos(extensionDistance);
//                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING_HEADING;
                    } else if (autoTimer.seconds()>.5) {
                        if (holdAngle<Math.toRadians(182)) {
                            holdAngle = Math.toRadians(190);
                            drivetrain.holdPoint(holdPoint.toPose(holdAngle));
                            grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                            autoTimer.reset();
                        } else {
                            holdAngle = Math.toRadians(170);
                            drivetrain.holdPoint(holdPoint.toPose(holdAngle));
                            grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                            autoTimer.reset();
                        }
                    }
                    break;
                case APPROACHING:
                    if (intake.getTargetSlidePos()-intake.getActualSlidePos() < 3) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
                        intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                        autoTimer.reset();
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING_2;
                    }
                    break;
                case INTAKING_2:

                    if (autoTimer.seconds() > 1) {
//                        double curHeading = drivetrain.getPoseEstimate().getHeading();
//
//                        if (curHeading > Math.toRadians(183)) {
//                            targetHeading = Math.toRadians(176);
//                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(176)));
//                        } else if (curHeading < Math.toRadians(177)) {
//                            targetHeading = Math.toRadians(183);
//                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(185)));
//                        } else {
//                            targetHeading = Math.toRadians(175);
//                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(175)));
//                        }
//
                        grabFromSubmersibleState = GrabFromSubmersibleState.RETRACTING;
//                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION_KEEP_SPINNING);
//                        extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;
                        autoTimer.reset();

                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 6 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }

                    break;
                case RETRACTING:
                    if (autoTimer.seconds() > .3) {
                        double curHeading = drivetrain.getPoseEstimate().getHeading();

                        if (curHeading > Math.toRadians(183)) {
                            targetHeading = Math.toRadians(176);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(176)));
                        } else if (curHeading < Math.toRadians(177)) {
                            targetHeading = Math.toRadians(183);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(185)));
                        } else {
                            targetHeading = Math.toRadians(175);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(175)));
                        }

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION_KEEP_SPINNING);
                        extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;
                        autoTimer.reset();
                    }
                    else {
                        extensionDistance = MathUtil.clip(extensionDistance - 6 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }

                    break;
                case EJECTING:
                    if (intake.getPrevIntakingState() == NewIntake.IntakingState.IDLE) {

                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;
                        intake.setTargetSlidePos(extensionDistance);
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                    }
                    break;
                case RESETTING:
                    if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5))) && Math.abs(intake.getTargetSlidePos()-intake.getActualSlidePos())<1) || autoTimer.seconds()>1) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING_2;
                        extensionDistance = 0;
                        autoTimer.reset();
                    }
                    break;
                case RESETTING_2:
                    if (autoTimer.seconds()>.25) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;
                        extensionDistance = 0;
                        autoTimer.reset();
                    }
                    break;
            }

            prevHeading = targetHeading;

            return true;


        }
    }

}

package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.reefsharklibrary.data.Rotation;
import com.reefsharklibrary.misc.ElapsedTimer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.NewOuttake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.threading.MasterThread;

@Autonomous
public class BlueRRLeft0plus7Auto extends LinearOpMode {

    public enum GrabFromSubmersibleState {
        SEARCHING,
        APPROACHING_HEADING,
        APPROACHING,
        INTAKING_1,
        RETRACTING,
        INTAKING_2,
        EJECTING,
        RESETTING,
        RESETTING_2
    }

    private double targetHeading = Math.toRadians(180);
    private double prevHeading = Math.toRadians(180);

    private final double maxGrabAngle = Math.toRadians(200);
    private final double minGrabAngle = Math.toRadians(155);

    NewDrivetrain drivetrain;
    NewIntake intake;
    NewOuttake outtake;
    VisionSubsystem vision;
    MasterThread masterThread;
    Telemetry.Item loopTimeTelem;

    Boolean blueAlliance = true;

    private Encoder verticalSlideEncoder, horizontalSlideEncoder;

    private TouchSensor breakBeam;

    double extensionDistance = 0;

    private final com.reefsharklibrary.data.Vector2d holdPoint = new com.reefsharklibrary.data.Vector2d(25, 9);

    private final ElapsedTimer loopTimer = new ElapsedTimer();

    private final ElapsedTimer autoTimer = new ElapsedTimer();

    private final ElapsedTimer totalAutoTimer = new ElapsedTimer();

    double timeThreshold = -1;

    double loopTime = 0;

    boolean skipScore = false;

    double prevExtensionError = 0;

    enum MoveToScoreState {
        DELAY,
        DRIVING_TO_BASKET,
        DROPPING
    }

    enum MoveToGrabState {
        FIRST_LOOP,
        DRIVING_TO_SUB_1,
        DRIVING_TO_SUB_2

    }

    enum ParkState {
        FIRST_LOOP,
        MOVE_TO_SUB_1,
        MOVE_FROM_SUB_1,
        MOVE_TO_SUB_2
    }


    @Override
    public void runOpMode() throws InterruptedException {
        loopTimeTelem = telemetry.addData("Loop time:", loopTimer.milliSeconds());

        masterThread = new MasterThread(hardwareMap, telemetry, gamepad1, gamepad2);

        horizontalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "horizontalLeft"));
        breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");

        verticalSlideEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "verticalLeft"));

        intake = new NewIntake(masterThread.getData(), horizontalSlideEncoder, breakBeam, blueAlliance, false, true, () -> drivetrain.getVoltage());

        outtake = new NewOuttake(masterThread.getData(), intake, verticalSlideEncoder, blueAlliance, false, true, true, true, () -> drivetrain.getVoltage());

        drivetrain = new NewDrivetrain(masterThread.getData(), outtake, intake);
        drivetrain.setDriveState(NewDrivetrain.DriveState.FOLLOW_PATH);


        vision = new VisionSubsystem(drivetrain, masterThread.getData(), blueAlliance, false);

        //its important that outtake is added after intake for update order purposes
        masterThread.addSubSystems(
                drivetrain,
                intake,
                outtake,
                vision
        );

        Action intakeBlock = new IntakeBlock();


        Action preload = drivetrain.drive.actionBuilder(new Pose2d(38.93, 60.23, Math.toRadians(180)))
                .setTangent(Math.toRadians(310))
                .afterTime(.3, () -> {
                    intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                    outtake.setClawPitch(NewOuttake.ClawPitch.UP.pos);
                })
                .afterTime(.6, () -> {//0
                    intake.setTargetSlidePos(13.5);
                    extensionDistance = 13.5;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);

                })
                .afterTime(.89, () -> {
                    outtake.setClawPitch(NewOuttake.ClawPitch.BACK2.pos);
                })
                .splineToLinearHeading(new Pose2d(61.5, 56, Math.toRadians(254)), Math.toRadians(0))
                .build();


        Action moveToGrabBlock1 = drivetrain.drive.actionBuilder(new Pose2d(61.5, 56, Math.toRadians(254)))
                .setTangent(Math.toRadians(254))
                .afterTime(0.2, () -> {
//                    intake.setTargetSlidePos(9);
//                    extensionDistance = 9;
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .splineToConstantHeading(new Vector2d(58, 51.5), Math.toRadians(261))
//                .splineToLinearHeading(new Pose2d(50.5, 50, Math.toRadians(270)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock1 = drivetrain.drive.actionBuilder(new Pose2d(58, 51.5, Math.toRadians(261)))
                .setTangent(Math.toRadians(84))
                .splineToLinearHeading(new Pose2d(62, 54, Math.toRadians(265)), Math.toRadians(85))
                .build();

        Action moveToGrabBlock2 = drivetrain.drive.actionBuilder(new Pose2d(62, 54, Math.toRadians(265)))
                .afterTime(0.2, () -> {
                    autoTimer.reset();
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                })
                .waitSeconds(.1)
                .setTangent(265)
                .splineToConstantHeading(new Vector2d(61, 51), Math.toRadians(265))
                .build();

        Action moveToScoreBlock2 = drivetrain.drive.actionBuilder(new Pose2d(61, 51, Math.toRadians(265)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(63, 54.5, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Action moveToGrabBlock3 = drivetrain.drive.actionBuilder(new Pose2d(63, 54.5, Math.toRadians(270)))
                .afterTime(.5, () -> {
                    intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                    autoTimer.reset();
                })
                .waitSeconds(.1)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(63, 50, Math.toRadians(282)), Math.toRadians(270))
                .build();

        Action moveToScoreBlock3 = drivetrain.drive.actionBuilder(new Pose2d(63, 50, Math.toRadians(282)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62.5, 55, Math.toRadians(270)), Math.toRadians(90))
                .build();


        Action park = new Action() {
                ParkState parkState = ParkState.FIRST_LOOP;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                    switch (parkState) {
                        case FIRST_LOOP:
                            intake.toIntakeState(NewIntake.ToIntakeState.RAISE_INTAKE);
                            intake.setIntakeState(NewIntake.IntakeState.RETRACTING_INTAKE);
                            intake.setTargetSlidePos(0);
                            extensionDistance = 0;

                            if (drivetrain.getPoseEstimate().getX()>40) {
                                //park from bucket
                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(32, holdPoint.getY(), Math.toRadians(235)));
                                parkState = ParkState.MOVE_TO_SUB_1;
                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                            } else {
                                parkState = ParkState.MOVE_FROM_SUB_1;
                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(32, holdPoint.getY(), Math.toRadians(180)));
                            }

                            autoTimer.reset();
                            return true;
                        case MOVE_TO_SUB_1:
                            if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(8, 8, Math.toRadians(8))))
                                    || autoTimer.seconds()>1.5) {

                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(21, 7, Math.toRadians(180)));
                                parkState = ParkState.MOVE_TO_SUB_2;

                            }
                            return true;
                        case MOVE_FROM_SUB_1:
                            if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(2, 8, Math.toRadians(8))))
                                    || autoTimer.seconds()>1.5) {
                                outtake.toOuttakeState(NewOuttake.ToOuttakeState.TOUCH_BAR);
                                drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(19, 7, Math.toRadians(180)));
                                parkState = ParkState.MOVE_TO_SUB_2;

                            }
                            return true;
                        case MOVE_TO_SUB_2:
                            if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(2, 1.5, Math.toRadians(3))) && drivetrain.getPoseVelocity().inRange(new com.reefsharklibrary.data.Pose2d(3, 3, Math.toRadians(4)))) {
                                return  false;
                            }
                            return true;

                    }

                    return false;
                }
            };
//hi brett
        drivetrain.drive.setPoseEstimate(new Pose2d(1, 1, 0));
        drivetrain.drive.pinpoint.update();

        if (drivetrain.drive.pinpoint.fastIsPinpointCooked()) {
            throw new RuntimeException("pinpoint cooked");
        }

        intake.setEjectOnIntake(false);

        waitForStart();

        totalAutoTimer.reset();

        drivetrain.drive.setPoseEstimate(new Pose2d(38.93, 60.23, Math.toRadians(180)));

        masterThread.clearBulkCache();

        intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
        outtake.toOuttakeState(NewOuttake.ToOuttakeState.PLACE_BEHIND);

        drivetrain.followPath(new SequentialAction(
                new RunToTimeThreshold(
                        new SequentialAction(
                                preload,
                                new ScoreBlock(),
                                moveToGrabBlock1,
                                intakeBlock,
                                new InstantAction(() -> {
                                    extensionDistance = 15;
                                }),
                                new ParallelAction(
                                    moveToScoreBlock1,
                                    new ExtendWhenTransfer()
                                ),
                                new ScoreBlock(),
                                moveToGrabBlock2,
                                intakeBlock,
                                new InstantAction(()-> {
                                    extensionDistance = 11.5;
                                }),
                                new ParallelAction(
                                    moveToScoreBlock2,
                                    new ExtendWhenTransfer()
                                ),
                                new ScoreBlock(),
                                moveToGrabBlock3,
                                intakeBlock,
                                moveToScoreBlock3,
                                new ScoreBlock(),
                                new InstantAction(() -> {
                                    intake.setEjectOnIntake(true);
                                }),
                                new SleepAction(.1),

                                new moveToGrabFromSub(),
                                new GrabFromSubmersible(),
                                new moveToScoreFromSub(),

                                new moveToGrabFromSub(),
                                new GrabFromSubmersible(),
                                new moveToScoreFromSub(),

                                new moveToGrabFromSub(),
                                new GrabFromSubmersible(),
                                new moveToScoreFromSub()
                        )
                ),
                park
        ));

        while ( !isStopRequested()) {

            masterThread.unThreadedUpdate();

            loopTime = loopTimer.seconds();
            loopTimeTelem.setValue(loopTimer.milliSeconds());

            loopTimer.reset();
        }

//        intake.overrideSpinOut();
//
//        drivetrain.stopMotors();
//        intake.stopMotors();
//        outtake.stopMotors();
//
//        sleep(250);
    }

    public class RunToTimeThreshold implements Action {
        Action action;

        public RunToTimeThreshold(Action action) {
            this.action = action;
        }

        boolean breakMotorsAct = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (30 - totalAutoTimer.seconds() < 0.1 && !breakMotorsAct) {
                drivetrain.breakMoters();
                breakMotorsAct = true;
            }
            return action.run(telemetryPacket) && ((30-totalAutoTimer.seconds())>timeThreshold);
        }
    }

    public class IntakeBlock implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (intake.getPrevIntakingState() != NewIntake.IntakingState.INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.START_INTAKING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_START_UNJAMMING && intake.getPrevIntakingState() != NewIntake.IntakingState.SERVO_STALL_UNJAMMING_SPIN_OUT) {
                return false;
            } else if (autoTimer.seconds()>2) {
                intake.toIntakeState(NewIntake.ToIntakeState.RETRACT);
//                extensionDistance = Math.max(extensionDistance-5, 0);
//                intake.setTargetSlidePos(extensionDistance);
//                skipScore = true;
                autoTimer.reset();
                return false;
            } else {
                extensionDistance = MathUtil.clip(extensionDistance + 4 * Math.max(.015, loopTime), -.5, 18.5);
                intake.setTargetSlidePos(extensionDistance);
                return true;
            }
        }
    }

    public class ScoreBlock implements Action {
        private boolean firstLoop = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstLoop) {
                if ((skipScore && !intake.isBreakBeam()) || (!intake.isBreakBeam() && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER))) {//!intake.isBreakBeam() && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER)
                    skipScore = false;
                    autoTimer.reset();
                    return false;
                } else {
                    skipScore = false;
                    firstLoop = false;
                    return true;
                }
            } else {
                if (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer()) {// || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING
                    outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                    autoTimer.reset();
                    skipScore = false;
                    return false;
                } else {
                    return true;
                }
            }
        }
    }

    public class ExtendWhenTransfer implements Action {
        boolean notExtended = true;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (notExtended) {
                if((outtake.getOuttakeState() == NewOuttake.OuttakeState.EXTENDING_PLACE_BEHIND || outtake.getOuttakeState() == NewOuttake.OuttakeState.EXTENDING_V4BAR_PLACE_BEHIND || outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND)) {
                    notExtended = false;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.STOP_INTAKING);
                    intake.setTargetSlidePos(extensionDistance);
                    return false;
                } else if ((!intake.isBreakBeam() && autoTimer.seconds() > .5 && (intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING_INTAKE || intake.getPrevIntakeState() == NewIntake.IntakeState.RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_AFTER_RETRACTING || intake.getPrevIntakeState() == NewIntake.IntakeState.WAITING_FOR_TRANSFER || intake.getPrevIntakeState() == NewIntake.IntakeState.INTAKING))) {
                    notExtended = false;
                    skipScore = true;
                    intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
                    intake.setIntakingState(NewIntake.IntakingState.STOP_INTAKING);
                    intake.setTargetSlidePos(extensionDistance);
                    return false;
                }
            }

            return true;
        }
    }

    public class moveToScoreFromSub implements Action {
        MoveToScoreState moveToScoreState = MoveToScoreState.DELAY;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (moveToScoreState) {
                case DELAY:
                    if (autoTimer.seconds()>.35) {
                        moveToScoreState = MoveToScoreState.DRIVING_TO_BASKET;
                        drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(59.5, 55, Math.toRadians(235)));
                    }
                    return true;
                case DRIVING_TO_BASKET:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(3, 3, Math.toRadians(4))) && drivetrain.getPoseVelocity().inRange(new com.reefsharklibrary.data.Pose2d(6, 6, Math.toRadians(8)))
                     && (outtake.getOuttakeState() == NewOuttake.OuttakeState.WAITING_PLACE_BEHIND || outtake.getFailedToTransfer())
                    ) {
                        outtake.toClawPosition(NewOuttake.ClawPosition.OPEN);
                        autoTimer.reset();
                        moveToScoreState = MoveToScoreState.DROPPING;

                    }
                    return true;
                case DROPPING:
                    if (.1 < autoTimer.seconds()) {
                        drivetrain.cancelHoldPoint();
                        timeThreshold = 2.5;
                        return  false;
                    }
                    return true;
            }

            return false;
        }
    }

    public class moveToGrabFromSub implements Action {
        MoveToGrabState moveToGrabState = MoveToGrabState.FIRST_LOOP;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (moveToGrabState) {
                case FIRST_LOOP:
                    drivetrain.holdPoint(new com.reefsharklibrary.data.Pose2d(32, holdPoint.getY()+2, Math.toRadians(235)));// holdPoint.toPose(Math.toRadians(235)));
                    autoTimer.reset();

                    moveToGrabState = MoveToGrabState.DRIVING_TO_SUB_1;

                    intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                    extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;

                    return true;
                case DRIVING_TO_SUB_1:
                    if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(10, 10, Math.toRadians(10))))
                            || autoTimer.seconds()>1.5) {

                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));
                        moveToGrabState = MoveToGrabState.DRIVING_TO_SUB_2;

                    }
                    return true;
                case DRIVING_TO_SUB_2:
                    if (drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(2, 2, Math.toRadians(7))) && drivetrain.getPoseVelocity().inRange(new com.reefsharklibrary.data.Pose2d(4, 4, Math.toRadians(6)))) {
                        drivetrain.cancelHoldPoint();
                        vision.recenter();
                        return  false;
                    }
                    return true;
            }

            return false;
        }
    }

    public class GrabFromSubmersible implements Action {
        BlueRRLeft0plus7Auto.GrabFromSubmersibleState grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;

        double holdAngle = Math.toRadians(180);


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_EJECTING || intake.getPrevIntakingState() == NewIntake.IntakingState.START_EJECTING) {
                grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.EJECTING;
            }

            if (intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING_A_LITTLE_MORE || intake.getPrevIntakingState() == NewIntake.IntakingState.INTAKING_SPIN_OUT || intake.getPrevIntakingState() == NewIntake.IntakingState.FINISH_INTAKING) {
                timeThreshold = -1;
                drivetrain.cancelHoldPoint();
                autoTimer.reset();
                intake.toIntakeState(NewIntake.ToIntakeState.IDLE);
                return false;
            }


            switch (grabFromSubmersibleState) {
                case SEARCHING:
                    if (vision.hasSample()) {
                        targetHeading = MathUtil.clip(Rotation.inRange(prevHeading+Rotation.inRange((vision.getTargetRobotPose().getHeading()-prevHeading), Math.PI, -Math.PI)*1, 2*Math.PI, 0), minGrabAngle, maxGrabAngle);

                        drivetrain.holdPoint(holdPoint.toPose(targetHeading));

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING;

                        extensionDistance = Math.max(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset()-2.4, 2.5);//Math.max(extensionDistance+(vision.getSampleRobotDiff().getMagnitude() - 9.59029 - intake.getIntakeHorizontalOffset() - 4)*.125, 3);
                        prevExtensionError = 0;
                        intake.setTargetSlidePos(extensionDistance);

                        autoTimer.reset();

                        // intake.setTargetSlidePos(extensionDistance);
//                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.APPROACHING_HEADING;
                    } else if (autoTimer.seconds()>.5) {
                        if (holdAngle<Math.toRadians(182)) {
                            holdAngle = Math.toRadians(190);
                            drivetrain.holdPoint(holdPoint.toPose(holdAngle));
                            grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                            autoTimer.reset();
                        } else {
                            holdAngle = Math.toRadians(170);
                            drivetrain.holdPoint(holdPoint.toPose(holdAngle));
                            grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                            autoTimer.reset();
                        }
                    }
                    break;
                case APPROACHING:
                    if (intake.getTargetSlidePos()-intake.getActualSlidePos() < 3) {
                        intake.toIntakeState(NewIntake.ToIntakeState.DROP_AND_INTAKE);
                        intake.setIntakingState(NewIntake.IntakingState.START_INTAKING);
                        autoTimer.reset();
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.INTAKING_2;
                    }
                    break;
                case INTAKING_2:

                    if (autoTimer.seconds() > 1) {
//                        double curHeading = drivetrain.getPoseEstimate().getHeading();
//
//                        if (curHeading > Math.toRadians(183)) {
//                            targetHeading = Math.toRadians(176);
//                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(176)));
//                        } else if (curHeading < Math.toRadians(177)) {
//                            targetHeading = Math.toRadians(183);
//                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(185)));
//                        } else {
//                            targetHeading = Math.toRadians(175);
//                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(175)));
//                        }
//
                        grabFromSubmersibleState = GrabFromSubmersibleState.RETRACTING;
//                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION_KEEP_SPINNING);
//                        extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;
                        autoTimer.reset();

                    } else {
                        extensionDistance = MathUtil.clip(extensionDistance + 6 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }

                    break;
                case RETRACTING:
                    if (autoTimer.seconds() > .3) {
                        double curHeading = drivetrain.getPoseEstimate().getHeading();

                        if (curHeading > Math.toRadians(183)) {
                            targetHeading = Math.toRadians(176);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(176)));
                        } else if (curHeading < Math.toRadians(177)) {
                            targetHeading = Math.toRadians(183);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(185)));
                        } else {
                            targetHeading = Math.toRadians(175);
                            drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(175)));
                        }

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION_KEEP_SPINNING);
                        extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;
                        autoTimer.reset();
                    }
                    else {
                        extensionDistance = MathUtil.clip(extensionDistance - 6 * loopTime, -.5, 18.5);
                        intake.setTargetSlidePos(extensionDistance);
                    }

                    break;
                case EJECTING:
                    if (intake.getPrevIntakingState() == NewIntake.IntakingState.IDLE) {

                        intake.toIntakeState(NewIntake.ToIntakeState.SEARCH_POSITION);
                        extensionDistance = NewIntake.HorizontalSlide.SEARCH_POS.length;
                        intake.setTargetSlidePos(extensionDistance);
                        drivetrain.holdPoint(holdPoint.toPose(Math.toRadians(180)));

                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING;
                    }
                    break;
                case RESETTING:
                    if ((drivetrain.getHoldPointError().minimizeHeading(Math.PI, -Math.PI).inRange(new com.reefsharklibrary.data.Pose2d(1, 1, Math.toRadians(3.5))) && Math.abs(intake.getTargetSlidePos()-intake.getActualSlidePos())<1) || autoTimer.seconds()>1) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.RESETTING_2;
                        extensionDistance = 0;
                        autoTimer.reset();
                    }
                    break;
                case RESETTING_2:
                    if (autoTimer.seconds()>.25) {
                        grabFromSubmersibleState = BlueRRLeft0plus7Auto.GrabFromSubmersibleState.SEARCHING;
                        extensionDistance = 0;
                        autoTimer.reset();
                    }
                    break;
            }

            prevHeading = targetHeading;

            return true;


        }
    }

}
