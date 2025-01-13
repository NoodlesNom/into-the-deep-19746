package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

public class TimedProfiledServo
{
    public ServoImplEx servo;
    private ElapsedTime clock;
    public double tgtPos;
    public double startPos;
    private double startTime, msTgt;
    private boolean reachedTgt;
    private double[] profile;
    private double[] profileTgt;
    private double[] profileSums;

    public TimedProfiledServo(ServoImplEx servo, ElapsedTime clock ) {
        double[] dfltProfile = {1.0};

        // Save clock source and init variables
        this.servo = servo;
        this.clock = clock;
        tgtPos = 0.0;
        startPos = 0.0;
        startTime = 0.0;
        msTgt = 0.0;
        reachedTgt = true;
        setProfile(dfltProfile);
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
        double newPos;
        double elapsedT, elapsedP;
        int profileIdx;

        // If we're not there and can move
        if ((currPos != tgtPos) && (!reachedTgt)) {
            if (!servo.isPwmEnabled()) {
                servo.setPwmEnable();
            }

            // Measure elapsed time, compute allowed movement, record time
            elapsedT = (now - startTime)*1000;
            elapsedP = Range.clip(elapsedT / msTgt, 0.0, 1.0);
            profileIdx = (int)(Math.min(Math.floor(profile.length*elapsedP), profile.length-1));

            double lwrTgt = profileTgt[profileIdx] ;
            double uprTgt = profileTgt[profileIdx+1] ;

            double lwrT = Range.clip(profileIdx * msTgt/profile.length, 0, msTgt);
            double uprT = Range.clip((profileIdx+1) * msTgt/profile.length, 0, msTgt);
            double partialP = Range.clip((elapsedT-lwrT) / (uprT-lwrT),0.0,1.0);
            partialP = lwrTgt + ((uprTgt-lwrTgt) * partialP);
            newPos = partialP;

            // Set new position
            servo.setPosition(newPos);

            // If we reached the end, clear the rate
            if (newPos == tgtPos) {
                reachedTgt = true;
            }

            // BotLog.logD("PT     tgtPos :: ", String.valueOf(tgtPos));
            // BotLog.logD("PT    currPos :: ", String.valueOf(currPos));
            // BotLog.logD("PT     newPos :: ", String.valueOf(newPos));
            // BotLog.logD("PT   ElapsedT :: ", String.valueOf(elapsedT));
            // BotLog.logD("PT   ElapsedP :: ", String.valueOf(elapsedP));
            // BotLog.logD("PT profileIdx :: ", String.valueOf(profileIdx));
            // BotLog.logD("PT     lwrTgt :: ", String.valueOf(lwrTgt));
            // BotLog.logD("PT     uprTgt :: ", String.valueOf(uprTgt));
            // BotLog.logD("PT       lwrT :: ", String.valueOf(lwrT));
            // BotLog.logD("PT       uprT :: ", String.valueOf(uprT));
            // BotLog.logD("PT   partialP :: ", String.valueOf(partialP));
        }
    }

    public void setTimedPosition(double position, double ms) {
        double currPos = servo.getPosition();

        setTimedPosition(currPos, position, ms);
    }

    public void setProfile(double[] profile) {
        double sum = 0.0;

        this.profile = profile;
        profileSums = new double[profile.length];

        for (int i = 0; i < profile.length; i++) {
            sum += profile[i];
            profileSums[i] = sum;
        }
        // BotLog.logD("PT Profile :: ", Arrays.toString(profile));
        // BotLog.logD("PT Sums :: ", Arrays.toString(profileSums));
    }

    // Set position and time to get there
    public void setTimedPosition(double currPos, double position, double ms) {
        // Get the time, save the target, compute a rate
        startTime = clock.seconds();
        tgtPos = position;
        startPos = servo.getPosition();
        msTgt = ms;
        reachedTgt = (tgtPos == currPos);

        double deltaTgt = (tgtPos - currPos);
        profileTgt = new double[profile.length+1];
        profileTgt[0] = currPos;

        for (int i = 0; i < profile.length; i++) {
            profileTgt[i+1] = currPos + (deltaTgt * (profileSums[i]/profileSums[profileSums.length-1]));
        }
        profileTgt[profileTgt.length-1] = tgtPos;
        // BotLog.logD("PT PTgt :: ", Arrays.toString(profileTgt));

        if (!servo.isPwmEnabled()) {
            servo.setPwmEnable();
        }
    }
}
