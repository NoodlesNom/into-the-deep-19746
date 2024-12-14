package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.autonomous.gf.SlipTuner;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

// @Disabled
@Config
@TeleOp(name = "motorSpeedTuner")
public class motorSpeedTuner extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private ElapsedTime myTimer;
    private enum State {
        quarter,
        half,
        threeQuarter,
        full,
        finished
    }
    private State auto = State.quarter;
    private double stateStartTime = 0;

    public static double lf = 0.8979;
    public static double lr = 1;
    public static double rf = 0.8696;
    public static double rr = 0.8684;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx .class, "fl");
        leftBack = hardwareMap.get(DcMotorEx.class, "bl");
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myTimer = new ElapsedTime();
        telemetry.addLine("Press play to begin the tuning opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        myTimer.reset();
        telemetry.clearAll();

        double multipliers[][] = new double[4][4];
        ArrayList<Double> lfspeeds = new ArrayList<>();
        ArrayList<Double> lrspeeds = new ArrayList<>();
        ArrayList<Double> rfspeeds = new ArrayList<>();
        ArrayList<Double> rrspeeds = new ArrayList<>();
        stateStartTime = myTimer.seconds();
        while (!isStopRequested()) {
            switch(auto){
                case quarter:
                {

                    leftFront.setPower(0.25*lf);
                    leftBack.setPower(0.25*lr);
                    rightBack.setPower(0.25*rr);
                    rightFront.setPower(0.25*rf);
                    if (myTimer.seconds() - stateStartTime > 2)
                    {
                        for (int i = 0; i<lfspeeds.size(); i++){
                            multipliers[0][0] += lfspeeds.get(i);
                            multipliers[0][1] += lrspeeds.get(i);
                            multipliers[0][2] += rfspeeds.get(i);
                            multipliers[0][3] += rrspeeds.get(i);
                        }
                        multipliers[0][0] /= lfspeeds.size();
                        multipliers[0][1] /= lfspeeds.size();
                        multipliers[0][2] /= lfspeeds.size();
                        multipliers[0][3] /= lfspeeds.size();
                        
                        double min = Math.min(multipliers[0][0], Math.min(multipliers[0][1], Math.min(multipliers[0][2], multipliers[0][3])));

                        multipliers[0][0] = multipliers[0][1] / multipliers[0][0];
                        multipliers[0][1] = multipliers[0][1];
                        multipliers[0][2] = multipliers[0][1] / multipliers[0][2];
                        multipliers[0][3] = multipliers[0][1] / multipliers[0][3];

                        lfspeeds.clear();
                        lrspeeds.clear();
                        rfspeeds.clear();
                        rrspeeds.clear();

                        stateStartTime = myTimer.seconds();
                        auto = State.half;
                    }else if (myTimer.seconds() - stateStartTime > 1){
                        lfspeeds.add(leftFront.getVelocity());
                        lrspeeds.add(leftBack.getVelocity());
                        rrspeeds.add(rightBack.getVelocity());
                        rfspeeds.add(rightFront.getVelocity());
                    }
                    break;

                }
                case half:
                {
                    leftFront.setPower(0.5*lf);
                    leftBack.setPower(0.5*lr);
                    rightBack.setPower(0.5*rr);
                    rightFront.setPower(0.5*rf);
                    if (myTimer.seconds() - stateStartTime > 2)
                    {
                        for (int i = 0; i<lfspeeds.size(); i++){
                            multipliers[1][0] += lfspeeds.get(i);
                            multipliers[1][1] += lrspeeds.get(i);
                            multipliers[1][2] += rfspeeds.get(i);
                            multipliers[1][3] += rrspeeds.get(i);
                        }
                        multipliers[1][0] /= lfspeeds.size();
                        multipliers[1][1] /= lfspeeds.size();
                        multipliers[1][2] /= lfspeeds.size();
                        multipliers[1][3] /= lfspeeds.size();

                        double min = Math.min(multipliers[1][0], Math.min(multipliers[1][1], Math.min(multipliers[1][2], multipliers[1][3])));

                        multipliers[1][0] = multipliers[1][1]/ multipliers[1][0];
                        multipliers[1][1] = multipliers[1][1];
                        multipliers[1][2] = multipliers[1][1] / multipliers[1][2];
                        multipliers[1][3] = multipliers[1][1] / multipliers[1][3];

                        lfspeeds.clear();
                        lrspeeds.clear();
                        rfspeeds.clear();
                        rrspeeds.clear();

                        stateStartTime = myTimer.seconds();
                        auto = State.threeQuarter;
                    }else if (myTimer.seconds() - stateStartTime > 1){
                        lfspeeds.add(leftFront.getVelocity());
                        lrspeeds.add(leftBack.getVelocity());
                        rrspeeds.add(rightBack.getVelocity());
                        rfspeeds.add(rightFront.getVelocity());
                    }
                    break;



                }
                case threeQuarter:
                {
                    leftFront.setPower(0.75*lf);
                    leftBack.setPower(0.75*lr);
                    rightBack.setPower(0.75*rr);
                    rightFront.setPower(0.75*rf);
                    if (myTimer.seconds() - stateStartTime > 2)
                    {
                        for (int i = 0; i<lfspeeds.size(); i++){
                            multipliers[2][0] += lfspeeds.get(i);
                            multipliers[2][1] += lrspeeds.get(i);
                            multipliers[2][2] += rfspeeds.get(i);
                            multipliers[2][3] += rrspeeds.get(i);
                        }
                        multipliers[2][0] /= lfspeeds.size();
                        multipliers[2][1] /= lfspeeds.size();
                        multipliers[2][2] /= lfspeeds.size();
                        multipliers[2][3] /= lfspeeds.size();

                        double min = Math.min(multipliers[2][0], Math.min(multipliers[2][1], Math.min(multipliers[2][2], multipliers[2][3])));

                        multipliers[2][0] = multipliers[2][1] / multipliers[2][0];
                        multipliers[2][1] = multipliers[2][1];
                        multipliers[2][2] = multipliers[2][1] / multipliers[2][2];
                        multipliers[2][3] = multipliers[2][1] / multipliers[2][3];

                        lfspeeds.clear();
                        lrspeeds.clear();
                        rfspeeds.clear();
                        rrspeeds.clear();

                        stateStartTime = myTimer.seconds();
                        auto = State.full;
                    }else if (myTimer.seconds() - stateStartTime > 1){
                        lfspeeds.add(leftFront.getVelocity());
                        lrspeeds.add(leftBack.getVelocity());
                        rrspeeds.add(rightBack.getVelocity());
                        rfspeeds.add(rightFront.getVelocity());
                    }
                    break;

                }
                case full:
                {
                    leftFront.setPower(lf);
                    leftBack.setPower(lr);
                    rightBack.setPower(rr);
                    rightFront.setPower(rf);
                    if (myTimer.seconds() - stateStartTime > 2)
                    {
                        for (int i = 0; i<lfspeeds.size(); i++){
                            multipliers[3][0] += lfspeeds.get(i);
                            multipliers[3][1] += lrspeeds.get(i);
                            multipliers[3][2] += rfspeeds.get(i);
                            multipliers[3][3] += rrspeeds.get(i);
                        }
                        multipliers[3][0] /= lfspeeds.size();
                        multipliers[3][1] /= lfspeeds.size();
                        multipliers[3][2] /= lfspeeds.size();
                        multipliers[3][3] /= lfspeeds.size();

                        double min = Math.min(multipliers[3][0], Math.min(multipliers[3][1], Math.min(multipliers[3][2], multipliers[3][3])));

                        multipliers[3][0] = multipliers[3][1] / multipliers[3][0];
                        multipliers[3][1] = multipliers[3][1];
                        multipliers[3][2] = multipliers[3][1] / multipliers[3][2];
                        multipliers[3][3] = multipliers[3][1] / multipliers[3][3];

                        lfspeeds.clear();
                        lrspeeds.clear();
                        rfspeeds.clear();
                        rrspeeds.clear();

                        stateStartTime = myTimer.seconds();
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        auto = State.finished;
                    }else if (myTimer.seconds() - stateStartTime > 1){
                        lfspeeds.add(leftFront.getVelocity());
                        lrspeeds.add(leftBack.getVelocity());
                        rrspeeds.add(rightBack.getVelocity());
                        rfspeeds.add(rightFront.getVelocity());
                    }
                    break;

                }
                case finished:
                {
                    telemetry.addLine("multipliers");
                    telemetry.addLine("front left: "+ (multipliers[0][0]+multipliers[1][0]+multipliers[2][0]+multipliers[3][0])/4);
                    telemetry.addLine("back left: "+ (multipliers[0][1]+multipliers[1][1]+multipliers[2][1]+multipliers[3][1])/4);
                    telemetry.addLine("front right: "+ (multipliers[0][2]+multipliers[1][2]+multipliers[2][2]+multipliers[3][2])/4);
                    telemetry.addLine("back right: "+ (multipliers[0][3]+multipliers[1][3]+multipliers[2][3]+multipliers[3][3])/4);

                    telemetry.addData("tests", Arrays.deepToString(multipliers));

                    telemetry.update();
                    break;
                }


            }
        }
    }
}