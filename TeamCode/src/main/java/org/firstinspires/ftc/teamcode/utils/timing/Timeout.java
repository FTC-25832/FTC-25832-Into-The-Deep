package org.firstinspires.ftc.teamcode.utils.timing;

import java.util.Timer;
import java.util.TimerTask;

public class Timeout {

    private Timer timer;

    public Timeout() {
    }

    public Timeout(Runnable callback, long delay) {
        timer = new Timer();

        TimerTask timerTask = new TimerTask() {
            @Override
            public void run() {
                callback.run();
                timer.cancel();
            }
        };
        timer.schedule(timerTask, delay);
    }

    public void cancel() {
        if (timer != null) {
            timer.cancel();
        }
    }
}

// usage
// Timeout timeout = new Timeout(() -> {
// // do something
// }, 1000); // 1 second delay
// timeout.cancel(); // cancel the timeout if needed