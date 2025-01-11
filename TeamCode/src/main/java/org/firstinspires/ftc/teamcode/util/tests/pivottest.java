package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// @Disabled
@Config
@TeleOp(name = "pivot tester")
public class pivottest extends LinearOpMode {

    private ServoImplEx pivotL;
    private ServoImplEx pivotR;
    private ServoImplEx diffyL;
    private ServoImplEx diffyR;
    public static double upper = 2400;
    public static double lower = 500;

    public static int pitch = 0;
    public static int roll = 0;

    public static double pos = 0.5;
    public static boolean move = false;
    public static boolean random = false;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        pivotL = hardwareMap.get(ServoImplEx .class, "pivotL");
        pivotR = hardwareMap.get(ServoImplEx .class, "pivotR");
        pivotR.setDirection(ServoImplEx.Direction.REVERSE);
        pivotL.setPosition(0.447);
        pivotR.setPosition(0.467);

        diffyL = hardwareMap.get(ServoImplEx .class, "diffyL");
        diffyR = hardwareMap.get(ServoImplEx .class, "diffyR");
        diffyR.setDirection(ServoImplEx.Direction.REVERSE);
        diffyL.setPosition(0.5);
        diffyR.setPosition(0.5);


        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.update();
            if (move) {
                pivotL.setPosition(pos - 0.02);
                pivotR.setPosition(pos);

                double targetL = 0.5+((pitch/340.0)+(roll/320.0));
                double targetR = 0.5+((pitch/340.0)-(roll/320.0));
                diffyL.setPosition(targetL);
                diffyR.setPosition(targetR);
            }
            pivotL.setPwmRange(new PwmControl.PwmRange(lower,upper));
            pivotR.setPwmRange(new PwmControl.PwmRange(lower,upper));
        }
    }
}