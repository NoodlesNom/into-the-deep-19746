package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// @Disabled
@Config
@TeleOp(name = "lift test")
public class lifttest extends LinearOpMode {

    private DcMotorEx extendo;
    public static int target = 0;
    public static double pow = 0;
    ElapsedTime test = new ElapsedTime();
    public static boolean move = false;
    public static boolean up = false;
    @Override
    public void runOpMode() throws InterruptedException {

        extendo = hardwareMap.get(DcMotorEx .class, "lift");
        extendo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.addData("pos", extendo.getCurrentPosition());
            telemetry.update();
        }
    }
}