package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoSelection extends Thread {
        public enum AutoType {
                FULL_SAMPLE, SPECIMEN
        }

        public enum ColorType {
                RED, BLUE
        }

        private Gamepad gamepad;
        private ElapsedTime elapsedTime;
        private Select select;

        private AutoType auto;
        private ColorType color;

        private enum Select {
                AUTO, COLOR
        }

        public AutoSelection(Gamepad _gamepad) {
                gamepad = _gamepad;
                elapsedTime = new ElapsedTime();
                select = Select.AUTO;
                auto = AutoType.FULL_SAMPLE;
                color = ColorType.RED;
                start();
        }

        @Override
        public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                        if (buttonTime(gamepad.dpad_up || gamepad.dpad_down)) {
                                select = select == Select.AUTO ? Select.COLOR : Select.AUTO;
                        }
                        if (buttonTime(gamepad.dpad_left || gamepad.dpad_right)) {
                                switch (select) {
                                        case AUTO:
                                                auto = auto == AutoType.FULL_SAMPLE ? AutoType.SPECIMEN
                                                                : AutoType.FULL_SAMPLE;
                                                break;
                                        case COLOR:
                                                color = color == ColorType.RED ? ColorType.BLUE : ColorType.RED;
                                                break;
                                }
                        }
                }
        }

        private boolean buttonTime(boolean bool) {
                if (0.25 < elapsedTime.seconds() && bool) {
                        elapsedTime.reset();
                        return true;
                }
                return false;
        }

        public void updateTelemetry(Telemetry telemetry) {
                telemetry.addData("Selection", select == Select.AUTO ? "Auto" : "Color");
                telemetry.addData("Auto", auto == AutoType.FULL_SAMPLE ? "Sample" : "Specimen");
                telemetry.addData("Color", color == ColorType.RED ? "Red" : "Blue");
        }

        public AutoType getAuto() {
                return auto;
        }

        public ColorType getColor() {
                return color;
        }
}