package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Base class for all commands
 */
public abstract class CommandBase implements Command {
        private final Set<SubsystemBase> requirements = new HashSet<>();
        private long lastLoopTime = 0;
        private long loopCount = 0;
        private double averageLoopTime = 0;
        private static List<LynxModule> allHubs = null;

        /**
         * Initialize bulk reads for all hubs
         */
        public static void initializeBulkReads(HardwareMap hardwareMap) {
                if (allHubs == null) {
                        allHubs = hardwareMap.getAll(LynxModule.class);
                        for (LynxModule hub : allHubs) {
                                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                        }
                }
        }

        /**
         * Gets the subsystems required by this command
         */
        public Set<SubsystemBase> getRequirements() {
                return requirements;
        }

        /**
         * Adds subsystem requirements
         */
        protected void addRequirement(SubsystemBase... requirements) {
                Collections.addAll(this.requirements, requirements);
        }

        @Override
        public void initialize() {
                lastLoopTime = System.nanoTime();
                loopCount = 0;
                averageLoopTime = 0;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Calculate loop time
                long currentTime = System.nanoTime();
                long loopTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime;

                // Update average loop time
                loopCount++;
                averageLoopTime = (averageLoopTime * (loopCount - 1) + loopTime) / loopCount;

                // Add telemetry
                packet.put("CommandBase/loopTime_ms", loopTime / 1e6);
                packet.put("CommandBase/avgLoopTime_ms", averageLoopTime / 1e6);
                packet.put("CommandBase/loopCount", loopCount);
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
        }

        @Override
        public Action toAction() {
                return Command.super.toAction();
        }
}
