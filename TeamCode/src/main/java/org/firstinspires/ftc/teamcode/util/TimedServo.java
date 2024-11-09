package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedServo {
    public ServoImplEx servo;
    private ElapsedTime clock;
    public double tgtPos;
    private double prevTime;
    private double rate;

    public TimedServo( ServoImplEx servo, ElapsedTime clock ) {
        // Save clock source and init variables
        this.servo = servo;
        this.clock = clock;
        tgtPos = 0.0;
        prevTime = 0.0;
        rate = 0.0;
    }

    public double getTgtPos() {
        return tgtPos;
    }

    public void update() {
        double currPos = servo.getPosition();

        update(currPos);
    }

    // Move towards our target
    public void update(double currPos) {
        // Current time and position
        double now = clock.seconds();
        double deltaP, deltaT, newPos;

        // If we're not there and can move
        if ((currPos != tgtPos) && (rate != 0.0)) {
            if (!servo.isPwmEnabled()) {
                servo.setPwmEnable();
            }

            // Measure elapsed time, compute allowed movement, record time
            deltaT = now - prevTime;
            deltaP = rate * deltaT;
            prevTime = now;

            if (currPos < tgtPos) {
                // If we are going upwards, add the deltaP and take the min
                newPos = Math.min(tgtPos, (currPos + deltaP));
            } else {
                // If we are going downwards, subtract the deltaP and take the max
                newPos = Math.max(tgtPos, (currPos - deltaP));
            }

            // Set new position
            servo.setPosition(newPos);

            // If we reached the end, clear the rate
            if (newPos == tgtPos) {
                rate = 0.0;
            }
        }
    }

    public void setTimedPosition(double position, double ms) {
        double currPos = servo.getPosition();

        setTimedPosition(currPos, position, ms);
    }

    // Set position and time to get there
    public void setTimedPosition(double currPos, double position, double ms) {
        // Get the time, save the target, compute a rate
        prevTime = clock.seconds();
        tgtPos = position;
        this.rate = Math.abs((tgtPos - currPos) / (ms / 1000.0));
        if (!servo.isPwmEnabled()) {
            servo.setPwmEnable();
        }
    }
}
