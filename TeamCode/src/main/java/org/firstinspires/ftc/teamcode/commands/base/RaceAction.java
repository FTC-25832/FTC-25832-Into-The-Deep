package org.firstinspires.ftc.teamcode.commands.base;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import java.util.ArrayList;
import java.util.List;

/**
 * An action that runs multiple actions in parallel and exits when any one of
 * them completes.
 * The other actions will continue running in the background.
 */
public class RaceAction implements Action {
        private final List<Action> actions;
        private final List<Boolean> actionResults;
        private boolean hasExited = false;

        public RaceAction(Action... actions) {
                this.actions = new ArrayList<>();
                this.actionResults = new ArrayList<>();
                for (Action action : actions) {
                        this.actions.add(action);
                        this.actionResults.add(false);
                }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                if (hasExited) {
                        return false;
                }

                boolean anyActionFinished = false;

                // Run all actions and check their results
                for (int i = 0; i < actions.size(); i++) {
                        if (!actionResults.get(i)) {
                                boolean result = actions.get(i).run(packet);
                                actionResults.set(i, !result); // Store if action is finished
                                if (!result) {
                                        anyActionFinished = true;
                                }
                        }
                }

                // If any action finished, mark this action as exited
                if (anyActionFinished) {
                        hasExited = true;
                        return false;
                }

                return true;
        }
}