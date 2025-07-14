package org.firstinspires.ftc.teamcode.commands.base

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase
import java.util.function.BooleanSupplier
import java.util.function.Consumer

class ConditionalCommand(private val condition: BooleanSupplier, command: Command) : CommandBase() {
    private val command: Command?
    private var shouldExecute = false
    private var isFinished = false

    init {
        this.command = command
        // 合并子命令的requirements
        command.getRequirements()
            .forEach(Consumer { requirements: SubsystemBase? -> this.addRequirement(requirements) })
    }

    override fun initialize() {
        shouldExecute = condition.asBoolean
        isFinished = !shouldExecute

        if (shouldExecute) {
            command!!.initialize()
        }
    }

    override fun execute(packet: TelemetryPacket?) {
        if (!shouldExecute || isFinished) {
            return
        }

        command!!.execute(packet)

        if (command.isFinished()) {
            command.end(false)
            isFinished = true
        }
    }

    override fun isFinished(): Boolean {
        return isFinished
    }

    override fun end(interrupted: Boolean) {
        if (shouldExecute && command != null) {
            command.end(interrupted)
        }
    }

    override fun getTimeout(): Long {
        return if (shouldExecute) command!!.getTimeout() else 0
    }
}