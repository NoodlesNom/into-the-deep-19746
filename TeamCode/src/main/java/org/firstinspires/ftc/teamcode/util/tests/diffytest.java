package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// @Disabled
@Config
@TeleOp(name = "diffy tester")
public class diffytest extends LinearOpMode {

    private ServoImplEx diffyL;
    private ServoImplEx diffyR;

    public static int position = 3;
    public static boolean move = false;
    public static boolean random = false;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        diffyL = hardwareMap.get(ServoImplEx .class, "diffyL");
        diffyR = hardwareMap.get(ServoImplEx .class, "diffyR");
        diffyR.setDirection(ServoImplEx.Direction.REVERSE);
        diffyL.setPosition(0.5);
        diffyR.setPosition(0.5);
        int pos = position;
        double targetL;
        double targetR;
        int pitch = 0;
        int roll = 0;
        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            targetL = 0.5+((pitch/340.0)+(roll/320.0));
            targetR = 0.5+((pitch/340.0)-(roll/320.0));
            telemetry.update();
            if (move) {
                if(random) {
                    if (timer.seconds() > 0.75) {
                        int temp = (int)(6*Math.random());
                        if (temp != pos){
                            timer.reset();
                            pos = temp;
                        }

                    }
                }else{
                    timer.reset();
                    pos = position;
                }
                diffyL.setPosition(targetL);
                diffyR.setPosition(targetR);
                switch (pos % 6) {
                    case 0: {
                        pitch = -60;
                        roll = 90;
                        break;
                    }
                    case 1: {
                        pitch = -60;
                        roll = 0;
                        break;
                    }
                    case 2: {
                        pitch = 0;
                        roll = 90;
                        break;
                    }
                    case 3: {
                        pitch = 0;
                        roll = 0;
                        break;
                    }
                    case 4: {
                        pitch = 60;
                        roll = 90;
                        break;
                    }
                    case 5: {
                        pitch = 60;
                        roll = 0;
                        break;
                    }
                }
            }
        }
    }
}