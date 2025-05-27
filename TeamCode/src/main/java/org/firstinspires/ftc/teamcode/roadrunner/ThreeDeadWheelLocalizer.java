package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.utils.constants.MovementConstants;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.messages.ThreeDeadWheelInputsMessage;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
        public static class Params {
                public double par0YTicks = MovementConstants.RobotDimensions.PAR0_Y_TICKS;
                public double par1YTicks = MovementConstants.RobotDimensions.PAR1_Y_TICKS;
                public double perpXTicks = MovementConstants.RobotDimensions.PERP_X_TICKS;

                // Tuning parameters
                public static double VELOCITY_WEIGHT = 0.8; // Weight for velocity in Kalman filter
                public static double POSITION_WEIGHT = 0.2; // Weight for position in Kalman filter
                public static double MIN_MOVEMENT_THRESHOLD = 1.0; // Minimum encoder delta for movement
                public static double MAX_VELOCITY_CHANGE = 50.0; // Maximum allowed velocity change per update
                public static int FILTER_WINDOW_SIZE = 5; // Size of moving average filter window
        }

        public static Params PARAMS = new Params();

        private final Encoder par0, par1, perp;
        private final double inPerTick;
        private final BulkReadManager bulkReader;
        private final LoopTimer loopTimer;
        private final ElapsedTime updateTimer;

        // Position tracking
        private int lastPar0Pos, lastPar1Pos, lastPerpPos;
        private double lastPar0Vel, lastPar1Vel, lastPerpVel;
        private boolean initialized;
        private Pose2d pose;

        // Filtering
        private final double[] par0VelHistory;
        private final double[] par1VelHistory;
        private final double[] perpVelHistory;
        private int historyIndex = 0;

        // Error tracking
        private int encoderErrors = 0;
        private int velocitySpikes = 0;
        private double maxObservedVelocity = 0.0;

        public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
                par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, ControlHub.motor(0))));
                par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, ControlHub.motor(3))));
                perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, ControlHub.motor(2))));

                this.inPerTick = inPerTick;
                this.bulkReader = BulkReadManager.getInstance(hardwareMap);
                this.loopTimer = new LoopTimer();
                this.updateTimer = new ElapsedTime();

                // Initialize filter buffers
                this.par0VelHistory = new double[PARAMS.FILTER_WINDOW_SIZE];
                this.par1VelHistory = new double[PARAMS.FILTER_WINDOW_SIZE];
                this.perpVelHistory = new double[PARAMS.FILTER_WINDOW_SIZE];

                // Register encoders with bulk reader
                bulkReader.addMotor((DcMotorEx) par0.getEncoder());
                bulkReader.addMotor((DcMotorEx) par1.getEncoder());
                bulkReader.addMotor((DcMotorEx) perp.getEncoder());

                FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
                pose = initialPose;
        }

        @Override
        public void setPose(Pose2d pose) {
                this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
                return pose;
        }

        @Override
        public PoseVelocity2d update() {
                loopTimer.startSection("localizer");
                updateTimer.reset();

                try {
                        bulkReader.clearCache();

                        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
                        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
                        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

                        // Validate encoder readings
                        if (!validateEncoderReading(par0PosVel) ||
                                        !validateEncoderReading(par1PosVel) ||
                                        !validateEncoderReading(perpPosVel)) {
                                encoderErrors++;
                                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
                        }

                        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS",
                                        new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

                        if (!initialized) {
                                initialized = true;
                                initializeState(par0PosVel, par1PosVel, perpPosVel);
                                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
                        }

                        // Calculate position changes
                        int par0PosDelta = par0PosVel.position - lastPar0Pos;
                        int par1PosDelta = par1PosVel.position - lastPar1Pos;
                        int perpPosDelta = perpPosVel.position - lastPerpPos;

                        // Apply deadband
                        par0PosDelta = applyDeadband(par0PosDelta);
                        par1PosDelta = applyDeadband(par1PosDelta);
                        perpPosDelta = applyDeadband(perpPosDelta);

                        // Filter velocities
                        double par0Vel = filterVelocity(par0PosVel.velocity, par0VelHistory);
                        double par1Vel = filterVelocity(par1PosVel.velocity, par1VelHistory);
                        double perpVel = filterVelocity(perpPosVel.velocity, perpVelHistory);

                        // Check for velocity spikes
                        if (detectVelocitySpike(par0Vel, lastPar0Vel) ||
                                        detectVelocitySpike(par1Vel, lastPar1Vel) ||
                                        detectVelocitySpike(perpVel, lastPerpVel)) {
                                velocitySpikes++;
                        }

                        // Create twist with filtered values
                        Twist2dDual<Time> twist = createTwist(
                                        par0PosDelta, par1PosDelta, perpPosDelta,
                                        par0Vel, par1Vel, perpVel);

                        // Update state
                        updateState(par0PosVel, par1PosVel, perpPosVel, par0Vel, par1Vel, perpVel);
                        pose = pose.plus(twist.value());

                        // Log performance metrics
                        logPerformanceMetrics(updateTimer.milliseconds());
                        loopTimer.endSection("localizer");

                        return twist.velocity().value();

                } catch (Exception e) {
                        encoderErrors++;
                        loopTimer.endSection("localizer");
                        return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
                }
        }

        private boolean validateEncoderReading(PositionVelocityPair reading) {
                return reading != null &&
                                !Double.isNaN(reading.velocity) &&
                                !Double.isInfinite(reading.velocity);
        }

        private void initializeState(PositionVelocityPair par0PosVel,
                        PositionVelocityPair par1PosVel,
                        PositionVelocityPair perpPosVel) {
                lastPar0Pos = par0PosVel.position;
                lastPar1Pos = par1PosVel.position;
                lastPerpPos = perpPosVel.position;
                lastPar0Vel = par0PosVel.velocity;
                lastPar1Vel = par1PosVel.velocity;
                lastPerpVel = perpPosVel.velocity;
        }

        private int applyDeadband(int delta) {
                return Math.abs(delta) < PARAMS.MIN_MOVEMENT_THRESHOLD ? 0 : delta;
        }

        private double filterVelocity(double newVelocity, double[] history) {
                history[historyIndex] = newVelocity;
                double sum = 0;
                for (double v : history) {
                        sum += v;
                }
                return sum / PARAMS.FILTER_WINDOW_SIZE;
        }

        private boolean detectVelocitySpike(double currentVel, double lastVel) {
                maxObservedVelocity = Math.max(maxObservedVelocity, Math.abs(currentVel));
                return Math.abs(currentVel - lastVel) > PARAMS.MAX_VELOCITY_CHANGE;
        }

        private Twist2dDual<Time> createTwist(
                        int par0PosDelta, int par1PosDelta, int perpPosDelta,
                        double par0Vel, double par1Vel, double perpVel) {
                return new Twist2dDual<>(
                                new Vector2dDual<>(
                                                new DualNum<Time>(new double[] {
                                                                (PARAMS.par0YTicks * par1PosDelta
                                                                                - PARAMS.par1YTicks * par0PosDelta)
                                                                                / (PARAMS.par0YTicks
                                                                                                - PARAMS.par1YTicks),
                                                                (PARAMS.par0YTicks * par1Vel
                                                                                - PARAMS.par1YTicks
                                                                                                * par0Vel)
                                                                                / (PARAMS.par0YTicks
                                                                                                - PARAMS.par1YTicks),
                                                }).times(inPerTick),
                                                new DualNum<Time>(new double[] {
                                                                (PARAMS.perpXTicks
                                                                                / (PARAMS.par0YTicks
                                                                                                - PARAMS.par1YTicks)
                                                                                * (par1PosDelta - par0PosDelta)
                                                                                + perpPosDelta),
                                                                (PARAMS.perpXTicks
                                                                                / (PARAMS.par0YTicks
                                                                                                - PARAMS.par1YTicks)
                                                                                * (par1Vel
                                                                                                - par0Vel)
                                                                                + perpVel),
                                                }).times(inPerTick)),
                                new DualNum<>(new double[] {
                                                (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                                (par0Vel - par1Vel)
                                                                / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                }));
        }

        private void updateState(
                        PositionVelocityPair par0PosVel, PositionVelocityPair par1PosVel,
                        PositionVelocityPair perpPosVel,
                        double par0Vel, double par1Vel, double perpVel) {
                lastPar0Pos = par0PosVel.position;
                lastPar1Pos = par1PosVel.position;
                lastPerpPos = perpPosVel.position;
                lastPar0Vel = par0Vel;
                lastPar1Vel = par1Vel;
                lastPerpVel = perpVel;
                historyIndex = (historyIndex + 1) % PARAMS.FILTER_WINDOW_SIZE;
        }

        private void logPerformanceMetrics(double updateTime) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("localizer/updateTime", updateTime);
                packet.put("localizer/encoderErrors", encoderErrors);
                packet.put("localizer/velocitySpikes", velocitySpikes);
                packet.put("localizer/maxVelocity", maxObservedVelocity);
                FlightRecorder.write("LOCALIZER_METRICS", packet);
        }
}
