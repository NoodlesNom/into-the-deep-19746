package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.TimedProfiledServo;

@Config
@TeleOp(name = "Pivot Profile Tuner")
public class PivotProfiledTuner extends LinearOpMode {

    private ServoImplEx pivotR;
    private ServoImplEx pivotL;
    private TimedProfiledServo pivotRPT;
    private TimedProfiledServo pivotLPT;

    public static double[] myProfile = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

    public static double myPos = 0.5;
    public static double duration = 1000.0;
    public static boolean move = false;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        pivotR = hardwareMap.get(ServoImplEx .class, "pivotR");
        pivotR.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));
        pivotR.setDirection(ServoImplEx.Direction.REVERSE);

        pivotL = hardwareMap.get(ServoImplEx .class, "pivotL");
        pivotL.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));

        pivotR.setPosition(myPos);
        pivotL.setPosition(myPos-0.02);

        pivotRPT = new TimedProfiledServo(pivotR, new ElapsedTime());
        pivotLPT = new TimedProfiledServo(pivotL, new ElapsedTime());

        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            pivotRPT.update();
            pivotLPT.update();
            if (move) {
                pivotRPT.setProfile(myProfile);
                pivotRPT.setTimedPosition(myPos, duration);

                pivotLPT.setProfile(myProfile);
                pivotLPT.setTimedPosition(myPos-0.02, duration);
                move = false;
                telemetry.update();
            }
        }
    }
}