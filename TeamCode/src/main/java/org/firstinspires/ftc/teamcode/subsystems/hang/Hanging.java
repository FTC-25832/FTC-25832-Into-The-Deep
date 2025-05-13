package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;

public class Hanging extends SubsystemBase {
    HardwareMap hardwareMap;
    public ServoImplEx left, right;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);

    public Hanging() {
        super("hanging");
    }

    @Override
    public void initialize(HardwareMap map) {
        hardwareMap = map;

        // Initialize servos
        left = hardwareMap.get(ServoImplEx.class, ExpansionHub.servo(5));
        right = hardwareMap.get(ServoImplEx.class, ControlHub.servo(5));
        right.setDirection(Servo.Direction.REVERSE);
        right.setPwmRange(v4range);
        left.setDirection(Servo.Direction.FORWARD);
        left.setPwmRange(v4range);
    }
    public void setPwm(double pwm){
        left.setPosition(pwm);
        right.setPosition(pwm);
    }
    public void turnForward() {
        left.setPosition(ConfigVariables.General.HANGING_SERVOS_SPEED);
        right.setPosition(ConfigVariables.General.HANGING_SERVOS_SPEED);
    }

    @Override
    public void stop() {
        // Use a safe neutral position (0.5) instead of 0 to avoid sudden movements
        left.setPosition(0.5);
        right.setPosition(0.5);
    }

    public void turnBackward() {
        left.setPosition(-ConfigVariables.General.HANGING_SERVOS_SPEED);
        right.setPosition(-ConfigVariables.General.HANGING_SERVOS_SPEED);
    }

}
