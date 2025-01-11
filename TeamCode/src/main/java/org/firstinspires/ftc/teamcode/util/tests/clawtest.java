package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@TeleOp(name = "pivot tester")
public class clawtest extends LinearOpMode {

    private ServoImplEx claw;


    public static double pos = 0;
    public static boolean move = false;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        claw = hardwareMap.get(ServoImplEx .class, "claw");
        claw.setPosition(0);


        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.update();
            if (move) {
                claw.setPosition(pos);
            }
        }
    }
}