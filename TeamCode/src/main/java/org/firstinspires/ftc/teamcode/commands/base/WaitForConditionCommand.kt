package org.firstinspires.ftc.teamcode.commands.base

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase
import java.util.function.BooleanSupplier
import java.util.function.Consumer

class WaitForConditionCommand(
    private val condition: BooleanSupplier,
    private val timeout: Long,
    private val command: Command
) : CommandBase() {
    private var startTime: Long = 0
    private var conditionMet = false
    private var isFinished = false
    private var commandStarted = false

    init {
        command.getRequirements()
            .forEach(Consumer { requirements: SubsystemBase? -> this.addRequirement(requirements) })
    }

    override fun initialize() {
        startTime = System.currentTimeMillis()
        conditionMet = false
        isFinished = false
        commandStarted = false
    }

    override fun execute(packet: TelemetryPacket?) {
        if (isFinished) {
            return
        }

        if (command.timeout > 0 && (System.currentTimeMillis() - startTime) >= command.timeout) {
            isFinished = true
            return
        }

        if (!conditionMet) {
            conditionMet = condition.asBoolean
            if (conditionMet) {
                command.initialize()
                commandStarted = true
            }
            return
        }

        if (commandStarted) {
            command.execute(packet)

            if (command.isFinished()) {
                command.end(false)
                isFinished = true
            }
        }
    }

    override fun isFinished(): Boolean {
        return isFinished
    }

    override fun end(interrupted: Boolean) {
        if (commandStarted) {
            command.end(interrupted)
        }
    }

    override fun getTimeout(): Long {
        return timeout
    }
}