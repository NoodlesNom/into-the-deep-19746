package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.TimedProfiledServo;
import org.firstinspires.ftc.teamcode.util.TimedServo;

@Config
@Disabled
@TeleOp(name = "Profiled Time Servo Tuner")
public class ProfiledTimeServoTuner extends LinearOpMode {

    private ServoImplEx myServo;
    private TimedProfiledServo myServoPT;
    private TimedServo myServoT;
    public static double[] myProfile = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

    public static double myPos = 0.5;
    public static double duration = 1000.0;
    public static boolean move = false;
    public static boolean mode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        myServo = hardwareMap.get(ServoImplEx .class, "myServo");
        myServoPT = new TimedProfiledServo(myServo, new ElapsedTime());
        myServoT = new TimedServo(myServo, new ElapsedTime());

        myServo.setPosition(0.5);

        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            if(mode)
            {
                myServoPT.update();
            } else
            {
                myServoT.update();
            }
            if (move) {
                telemetry.addLine("Moving to:" + myServoPT.getTgtPos());
                if(mode)
                {
                    myServoPT.setProfile(myProfile);
                    myServoPT.setTimedPosition(myPos, duration);
                } else
                {
                    myServoT.setTimedPosition(myPos, duration);
                }
                move = false;
                telemetry.update();
            }
        }
    }
}