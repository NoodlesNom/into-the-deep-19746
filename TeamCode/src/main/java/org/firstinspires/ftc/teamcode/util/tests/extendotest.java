package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

// @Disabled
@Config
@TeleOp(name = "extendo test")
public class extendotest extends LinearOpMode {

    private DcMotorEx extendo;
    public static int target = 0;
    public static double pow = 0.5;
    ElapsedTime test = new ElapsedTime();
    public static boolean move = false;
    public static boolean up = false;
    @Override
    public void runOpMode() throws InterruptedException {

        extendo = hardwareMap.get(DcMotorEx .class, "extendo");
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.addData("pos", extendo.getCurrentPosition());
            telemetry.update();
            if (move) {
                if(test.seconds()<pow){
                    extendo.setPower(1);
                }else{
                    extendo.setPower(0);
                }

            }else{
                test.reset();
            }
        }
    }
}