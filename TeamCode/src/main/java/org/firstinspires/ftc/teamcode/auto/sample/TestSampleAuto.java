package org.firstinspires.ftc.teamcode.auto.sample;

import android.icu.text.CaseMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.UpperSlide;

@Autonomous
public final class TestSampleAuto extends LinearOpMode {
    // Common positions and headings for the robot
    private static final class RobotPosition {
        public final Vector2d pos;
        public final double heading;
        public final Pose2d pose;

        public RobotPosition(double x, double y, double headingDegrees) {
            this.pos = new Vector2d(x, y);
            this.heading = Math.toRadians(headingDegrees);
            this.pose = new Pose2d(x, y, this.heading);
        }
    }

    // Define common positions
    private static final RobotPosition START = new RobotPosition(30.5, 62, 270);
    private static final RobotPosition PREPLACED = new RobotPosition(47, 46, -90);
    private static final RobotPosition POS1 = new RobotPosition(60, 60, 225);
    private static final RobotPosition POS2 = new RobotPosition(55, 55, 225);
    private static final RobotPosition PICKUP = new RobotPosition(58, 46, -55);
    private static final RobotPosition ENDPOINT = new RobotPosition(38, 32, 180);
    private static final RobotPosition FULLSEND_START = new RobotPosition(38, 12, 180);
    private static final RobotPosition FULLSEND_END = new RobotPosition(23, 10, 180);

    public class lowSlide {
        private LowerSlide lowslide;
        public lowSlide(HardwareMap hardwareMap) {
            lowslide = new LowerSlide();
            lowslide.initialize(hardwareMap);
        }
        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowslide.setPositionCM(34.0);
                return false;
            }
        }

        public Action extend() {
            return new Extend();
        }

        public class Comeback implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowslide.setSlidePos1();
                lowslide.pos_hover();
                return false;
            }
        }

        public Action comeback() {
            return new Comeback();
        }

        public class Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowslide.pos_grab();
                lowslide.openClaw();
                lowslide.closeClaw();
                lowslide.pos_up();
                return false;
            }
        }

        public Action grab() {
            return new Grab();
        }

        public class Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowslide.openClaw();
                return false;
            }
        }

        public Action transfer() {
            return new Transfer();
        }

    }

    public class upSlide {
        private UpperSlide upslide;

        public upSlide(HardwareMap hardwareMap) {
            upslide = new UpperSlide();
            upslide.initialize(hardwareMap);
        }
        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upslide.pos2();
                upslide.front();
                return false;
            }
        }

        public Action extend() {
            return new Extend();
        }

        public class Comeback implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upslide.pos0();
                return false;
            }
        }

        public Action comeback() {
            return new Comeback();
        }

        public class Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upslide.transfer();
                return false;
            }
        }

        public Action transfer() {
            return new Transfer();
        }

        public class TransferReceive implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upslide.closeClaw();
                return false;
            }
        }

        public Action transferReceive() {
            return new Transfer();
        }

        public class Drop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upslide.openClaw();
                return false;
            }
        }

        public Action drop() {
            return new Drop();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, START.pose);
        lowSlide lowerslide = new lowSlide(hardwareMap);
        upSlide upslide = new upSlide(hardwareMap);

        TrajectoryActionBuilder prePlaced1 = drive.actionBuilder(PREPLACED.pose)
                .strafeToLinearHeading(POS1.pos, POS1.heading)
                .strafeToConstantHeading(POS2.pos);

        TrajectoryActionBuilder prePlaced2 = drive.actionBuilder(POS2.pose)
                .strafeToLinearHeading(PICKUP.pos, PICKUP.heading)
                .strafeToLinearHeading(POS1.pos, POS1.heading)
                .strafeToConstantHeading(POS2.pos);

        TrajectoryActionBuilder prePlaced3 = drive.actionBuilder(POS2.pose)
                .strafeToLinearHeading(PICKUP.pos, PICKUP.heading)
                .strafeToLinearHeading(POS1.pos, POS1.heading)
                .strafeToConstantHeading(POS2.pos)
                .strafeToLinearHeading(ENDPOINT.pos, ENDPOINT.heading);

        TrajectoryActionBuilder FULLSEND = drive.actionBuilder(POS2.pose)
                .strafeToConstantHeading(FULLSEND_START.pos)
                .strafeToConstantHeading(FULLSEND_END.pos);

        TrajectoryActionBuilder BOXBOX = drive.actionBuilder(FULLSEND_END.pose)
                .strafeToConstantHeading(FULLSEND_START.pos)
                .strafeToLinearHeading(POS1.pos, POS1.heading)
                .strafeToConstantHeading(POS2.pos);



        Actions.runBlocking(lowerslide.comeback());


        while (!isStopRequested() && !opModeIsActive()) {
            // int position = visionOutputPosition;
            // telemetry.addData("Position during Init", position);
            // telemetry.update();
        }

        // int startPosition = visionOutputPosition;
        // telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested())
            return;

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(START.pose)
                                .strafeToLinearHeading(PREPLACED.pos,
                                        PREPLACED.heading)
                                .build(),
                        lowerslide.extend(),
                        lowerslide.grab(),
                        drive.actionBuilder(PREPLACED.pose)
                                .strafeToLinearHeading(POS1.pos,
                                        POS1.heading)
                                .build(),
                        lowerslide.comeback(),
                        upslide.transfer(),
                        lowerslide.transfer(),
                        upslide.transferReceive(),
                        upslide.extend(),
                        upslide.drop(),
                        drive.actionBuilder(POS1.pose)
                                .strafeToConstantHeading(POS2.pos)
                                .build(),
                        upslide.comeback(),


                        drive.actionBuilder(POS2.pose)
                                .strafeToLinearHeading(PICKUP.pos, PICKUP.heading)
                                .build(),
                        lowerslide.extend(),

                        drive.actionBuilder(PICKUP.pose)
                                .strafeToLinearHeading(POS1.pos,
                                        POS1.heading)
                                .build(),
                        lowerslide.comeback(),
                        upslide.transfer(),
                        lowerslide.transfer(),
                        upslide.transferReceive(),
                        upslide.extend(),
                        upslide.drop(),
                        drive.actionBuilder(POS1.pose)
                                .strafeToConstantHeading(POS2.pos)
                                .build(),
                        upslide.comeback(),


                        drive.actionBuilder(POS2.pose)
                                .strafeToLinearHeading(PICKUP.pos,
                                        Math.toRadians(55))
                                .build(),
                        lowerslide.extend(),

                        drive.actionBuilder(PICKUP.pose)
                                .strafeToLinearHeading(POS1.pos,
                                        POS1.heading)
                                .build(),

                        lowerslide.comeback(),
                        upslide.transfer(),
                        lowerslide.transfer(),
                        upslide.transferReceive(),
                        upslide.extend(),
                        upslide.drop(),
                        drive.actionBuilder(POS1.pose)
                                .strafeToConstantHeading(POS2.pos)
                                .build(),
                        upslide.comeback(),
                        drive.actionBuilder(POS2.pose)
                                .strafeToLinearHeading(ENDPOINT.pos, ENDPOINT.heading)
                                .build()

                ));
    }
}
