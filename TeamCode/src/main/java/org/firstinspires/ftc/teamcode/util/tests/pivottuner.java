package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// @Disabled
@Config
@TeleOp(name = "pivot tuner")
public class pivottuner extends LinearOpMode {

    private ServoImplEx pivotL;
    private ServoImplEx pivotR;

    public static double posL = 0.5;
    public static double posR = 0.5;
    public static boolean move = false;
    public static boolean random = false;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        pivotL = hardwareMap.get(ServoImplEx .class, "pivotL");
        pivotR = hardwareMap.get(ServoImplEx .class, "pivotR");
        pivotR.setDirection(ServoImplEx.Direction.REVERSE);
        pivotL.setPosition(posL);
        pivotR.setPosition(posR);

        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.update();
            if (move) {
                pivotL.setPosition(posL);
                pivotR.setPosition(posR);
            }
            pivotL.setPwmRange(new PwmControl.PwmRange(500,2400));
            pivotR.setPwmRange(new PwmControl.PwmRange(500,2400));
        }
    }
}