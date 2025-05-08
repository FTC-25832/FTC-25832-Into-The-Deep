package org.firstinspires.ftc.teamcode.auto.classes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.auto.Autos;
import org.firstinspires.ftc.teamcode.collections.Color;

public class AutoSelection extends Thread {
        private Gamepad gamepad;
        private ElapsedTime elapsedTime;
        private Select select;

        private Autos auto;
        private Color color;

        private enum Select {
                Auto, Color
        }

        public AutoSelection(Gamepad _gamepad) {
                gamepad = _gamepad;

                elapsedTime = new ElapsedTime();

                select = Select.Auto;
                auto = Autos.fullSample;
                color = Color.Red;

                start();
        }

        @Override
        public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                        if (buttonTime(gamepad.dpad_up || gamepad.dpad_down)) {
                                select = select == Select.Auto ? Select.Color : Select.Auto;
                        }

                        if (buttonTime(gamepad.dpad_left || gamepad.dpad_right)) {
                                switch (select) {
                                        case Auto:
                                                auto = auto == Autos.fullSample ? Autos.specimen : Autos.fullSample;
                                                break;
                                        case Color:
                                                color = color == Color.Red ? Color.Blue : Color.Red;
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
                telemetry.addData("Selection", select == Select.Auto ? "Auto" : "Color");
                telemetry.addData("Auto", auto == Autos.fullSample ? "Sample" : "Specimen");
                telemetry.addData("Color", color == Color.Red ? "Red" : "Blue");
        }

        public Autos getAuto() {
                return auto;
        }

        public Color getColor() {
                return color;
        }
}