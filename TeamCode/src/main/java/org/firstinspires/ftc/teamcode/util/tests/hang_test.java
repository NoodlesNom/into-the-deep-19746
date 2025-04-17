package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "hang servo tester")
public class hang_test extends LinearOpMode {

    private AnalogInput leftwinchencoder;
    private AnalogInput rightwinchencoder;

    private CRServoImplEx winchL;
    private CRServoImplEx winchR;
    private ServoImplEx pto;


    private double leftdelta;
    private double rightdelta;
    public double leftwinchadjusted = 0;
    public double rightwinchadjusted = 0;

    public static double pos = 55;

    public static double ptoPos = 0.81;

    public static double posdiviser = 150;

    private double leftwinchprevious = 0;
    private double rightwinchprevious = 0;

    private double leftwinchpos = -720;
    private double rightwinchpos = -720;
    public static boolean move = true;
    public static boolean auto = false;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        leftwinchencoder = hardwareMap.get(AnalogInput.class, "leftwinchencoder");

        pto = hardwareMap.get(ServoImplEx.class, "PTO");
        rightwinchencoder = hardwareMap.get(AnalogInput.class, "rightwinchencoder");
        winchL = hardwareMap.get(CRServoImplEx.class, "hangL");
        winchR = hardwareMap.get(CRServoImplEx.class, "hangR");
        winchL.setDirection(CRServoImplEx.Direction.REVERSE);
        //pto.setPwmRange(new PwmControl.PwmRange(500,2400));
        pto.setPosition(0.81);



        waitForStart();
        timer.reset();

        if (isStopRequested()) return;
        while (!isStopRequested()) {
            telemetry.update();
            if (move) {
                pto.setPosition(ptoPos);
                if (!auto) {
                    if (gamepad1.right_trigger > 0.1) {
                        winchL.setPower(-1 * gamepad1.right_trigger);
                        winchR.setPower(-1 * gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger > 0.1) {
                        winchL.setPower(1 * gamepad1.left_trigger);
                        winchR.setPower(1 * gamepad1.left_trigger);
                    } else {
                        winchL.setPower(gamepad1.left_stick_y);
                        winchR.setPower(gamepad1.right_stick_y);
                    }
                }else{
                    winchL.setPower(-(pos - (leftwinchadjusted)) / posdiviser);
                    winchR.setPower(-(pos - (rightwinchadjusted+20)) / posdiviser);

                }
            }
            if  (leftwinchpos==-720){
                leftwinchpos = -(leftwinchencoder.getVoltage() / 3.3 * 360)+360;
                rightwinchpos = (rightwinchencoder.getVoltage() / 3.3 * 360);
                leftwinchprevious=leftwinchpos;
                leftwinchadjusted=leftwinchpos;
                rightwinchprevious=rightwinchpos;
                rightwinchadjusted=rightwinchpos;
            }else {
                leftwinchpos = -(leftwinchencoder.getVoltage() / 3.3 * 360) + 360;
                rightwinchpos = (rightwinchencoder.getVoltage() / 3.3 * 360);
            }

            leftdelta = leftwinchpos - leftwinchprevious;

            if (leftdelta > 180) leftdelta -= 360;
            if (leftdelta < -180) leftdelta += 360;
            leftwinchadjusted += leftdelta;
            leftwinchprevious = leftwinchpos;

            rightdelta = rightwinchpos - rightwinchprevious;
            if (rightdelta > 180) rightdelta -= 360;
            if (rightdelta < -180) rightdelta += 360;
            rightwinchadjusted += rightdelta;
            rightwinchprevious = rightwinchpos;

            telemetry.addData("left winch pos: ", leftwinchadjusted);
            telemetry.addData("right winch pos: ", rightwinchadjusted);
        }
    }
}