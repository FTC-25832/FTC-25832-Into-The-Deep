package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

@Autonomous(name = "SplineTest", group = "tuning")
public final class SplineTest extends LinearOpMode {
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

        private LowerSlide lowSlide;
        private UpperSlide upSlide;

        @Override
        public void runOpMode() throws InterruptedException {
                MecanumDrive drive = new MecanumDrive(hardwareMap, START.pose);

                // Initialize slides
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                Action autonomous = drive.actionBuilder(START.pose)
                                // Initial setup and first specimen
                                .strafeToLinearHeading(PREPLACED.pos, PREPLACED.heading)
                                .build();

                // Pre-build other movement actions
                Action moveToPos1 = drive.actionBuilder(PREPLACED.pose)
                                .strafeToLinearHeading(POS1.pos, POS1.heading)
                                .build();

                Action moveToPickup = drive.actionBuilder(POS1.pose)
                                .strafeToLinearHeading(PICKUP.pos, PICKUP.heading)
                                .build();

                Action moveToEndpoint = drive.actionBuilder(PICKUP.pose)
                                .strafeToLinearHeading(ENDPOINT.pos, ENDPOINT.heading)
                                .build();

                telemetry.addData("Status", "Initialized");
                telemetry.update();

                waitForStart();

                if (opModeIsActive()) {
                        // Execute movement sequence
                        Actions.runBlocking(autonomous);

                        // Operate slides and move
                        lowSlide.setPositionCM(34.0);
                        sleep(500);

                        Actions.runBlocking(moveToPos1);

                        lowSlide.setSlidePos1();
                        lowSlide.pos_hover();
                        sleep(500);
                        upSlide.transfer();
                        sleep(300);

                        Actions.runBlocking(moveToPickup);

                        lowSlide.openClaw();
                        sleep(300);

                        Actions.runBlocking(moveToEndpoint);

                        telemetry.addData("Status", "Complete");
                        telemetry.update();
                }
        }
}
