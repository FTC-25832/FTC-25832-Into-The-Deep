package org.firstinspires.ftc.teamcode.utils.timing;
import java.util.Timer;
import java.util.TimerTask;

public class Interval {

    private Timer timer;

    public Interval() {
    }

    public Interval(Runnable callback, long period) {
        timer = new Timer();

        TimerTask timerTask = new TimerTask() {
            @Override
            public void run() {
                callback.run();
            }
        };
        timer.schedule(timerTask,0,period);
    }

    public void cancel(){
        if(timer!=null){
            timer.cancel();
        }
    }
}

// usage
// Interval interval = new Interval(() -> {
//     // do something
// }, 1000); // 1 second interval
// interval.cancel(); // cancel the interval if needed
