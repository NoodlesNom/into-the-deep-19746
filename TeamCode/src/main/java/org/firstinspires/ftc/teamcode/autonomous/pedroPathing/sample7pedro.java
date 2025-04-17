/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous.pedroPathing;

import static org.firstinspires.ftc.teamcode.autonomous.gf.OldAutoMaster.team;
import static org.firstinspires.ftc.teamcode.teleop.BSTEMTELE.transferendheight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="PEDRO 7 SAMPLE", group="Autonomous")
public class sample7pedro extends OpMode
{
    // Declare OpMode members.
    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetryA;
    private Timer pathTimer, updateTimer, opmodeTimer;

    private enum PathState{
        START,
        PRELOAD,
        INTAKE1,
        TRANSFER2,
        SAMPLE2,
        INTAKE2,
        TRANSFER3,
        SAMPLE3,
        INTAKE3,
        TRANSFER4,
        SAMPLE4,
        SUB1,
        TRANSFER5,
        SAMPLE5UP,
        SAMPLE5DOWN,
        SUB2,
        TRANSFER6,
        SAMPLE6UP,
        SAMPLE6DOWN,
        SUB3,
        TRANSFER7,
        SAMPLE7UP,
        SAMPLE7DOWN,
        END
    }
    private PathState pathState = PathState.START;
    private PathChain preload, intake1, sample2, intake2, sample3, intake3, sample4, sub1, sample5, sub2, sample6, sub3, sample7;


    private RobotPedro robot;
    private boolean intaken = false;
    private boolean stopresetting = false;
    private boolean reject = false;
    private boolean detected = false;
    private boolean failed;
    private boolean genericboolean = false;
    public static int blockx1 = 2;
    private String failedtimes = "0";
    public static int blockx2 = 2;
    public static int blockx3 = 2;
    private int block = 0;
    public static boolean has_adjust_2 = false;
    private StickyButton intakeposup = new StickyButton();
    private StickyButton intakeposdown = new StickyButton();
    private StickyButton intakeposleft = new StickyButton();

    private double hangpwr = 1;
    private StickyButton switchblock = new StickyButton();
    private StickyButton intakeposright = new StickyButton();
    public static int blocky1 = 0;
    public static int blocky2 = 0;
    public static int blocky3 = 1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        block = 0;
        has_adjust_2 = false;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        updateTimer = new Timer();
        updateTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(8.75, 112.25-72, Math.toRadians(0)));
        robot = new RobotPedro(this);

        //robot.teleopInit();

        timer.reset();
        robot.update(timer.seconds());
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+8.75, 112.25-75, Point.CARTESIAN),
                                new Point(2+12, 136.000-75, Point.CARTESIAN)
                        )
                )
                .addParametricCallback(0.90, () -> robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal()))
                .addParametricCallback(0.95, () -> {
                    robot.mDeposit.setDiffyPos(80, -90);
                    robot.mDeposit.setClawPos(0);
                })

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-27))
                .build();

        intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+11.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+20.000, 132.000-75, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-27), Math.toRadians(-19))
                .build();

        sample2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+20.000, 132.000-75, Point.CARTESIAN),
                                new Point(2+11.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-19), Math.toRadians(-30))
                .build();

        intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+11.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+18.000, 135.000-75, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-3))
                .build();

        sample3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+18.000, 135.000-75, Point.CARTESIAN),
                                new Point(2+11.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(-30))
                .build();

        intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+11.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+18.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(15))
                .build();

        sample4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(2+18.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(-28))
                .build();

        sub1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+60.000, 120-75, Point.CARTESIAN),
                                new Point(2+60.000, 100-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        sample5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+60.000, 100-75, Point.CARTESIAN),
                                new Point(2+60.000, 120-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        sub2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+60.000, 120-75, Point.CARTESIAN),
                                new Point(2+60.000, 100-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        sample6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+60.000, 100-75, Point.CARTESIAN),
                                new Point(2+60.000, 120-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        sub3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+60.000, 120-75, Point.CARTESIAN),
                                new Point(2+60.000, 100-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        sample7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+60.000, 100-75, Point.CARTESIAN),
                                new Point(2+60.000, 120-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        intakeposleft.update(gamepad1.dpad_left);
        intakeposright.update(gamepad1.dpad_right);
        intakeposup.update(gamepad1.dpad_up);
        intakeposdown.update(gamepad1.dpad_down);
        switchblock.update(gamepad1.x);
        robot.update(timer.seconds());
        //timer.reset();
        robot.autoInit();
        robot.mDeposit.setLiveLed(team);
        telemetry.addLine("PRESS A (BOTTOM) FOR BLUE");
        telemetry.addLine("PRESS B (RIGHT) FOR RED");
        telemetry.addLine("CURRENT COLOR: " + team.name());
        if (gamepad1.a){
            team = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }
        if (gamepad1.b){
            team = RevBlinkinLedDriver.BlinkinPattern.RED;
        }
        if (switchblock.getState()){
            block++;
        }
        telemetry.addLine("USE DPAD TO ADJUST INTAKE POS BY INCHES");
        telemetry.addLine("PRESS X TO ADJUST WHICH INTAKE");
        telemetry.addLine("IF DONT ADJUST INTAKE 2, WILL BE SAME AS 1. 2 ADJUSTED = " +has_adjust_2);
        if (block%3==0){
            telemetry.addLine("ADJUSTING FIRST INTAKE");
            if (intakeposup.getState()){
                blocky1 +=1;
            }else if (intakeposdown.getState()){
                blocky1 -=1;
            }else if (intakeposleft.getState()&& blockx1 >0){
                blockx1 -=1;
            }else if (intakeposright.getState()){
                blockx1 +=1;
            }
        }else if  (block % 3 == 1){
            has_adjust_2 = true;
            telemetry.addLine("ADJUSTING SECOND INTAKE");
            if (intakeposup.getState()){
                blocky2 +=1;
            }else if (intakeposdown.getState()){
                blocky2 -=1;
            }else if (intakeposleft.getState()&& blockx2 >0){
                blockx2 -=1;
            }else if (intakeposright.getState()){
                blockx2 +=1;
            }
        }else if  (block % 3 == 2){
            telemetry.addLine("ADJUSTING FALLBACK INTAKE");
            if (intakeposup.getState()){
                blocky3 +=1;
            }else if (intakeposdown.getState()){
                blocky3 -=1;
            }else if (intakeposleft.getState()&& blockx3 >0){
                blockx3 -=1;
            }else if (intakeposright.getState()){
                blockx3 +=1;
            }
        }

        telemetry.addLine("CURR INTAKE POS 1: "+ (blocky1) + ", " + blockx1);
        telemetry.addLine("CURR INTAKE POS 2: "+ (blocky2) + ", " + blockx2);
        telemetry.addLine("CURR FALLBACK POS: "+ (blocky3) + ", " + blockx3);
        telemetry.addLine("DO NOT MAKE FALLBACK FIRST NUMBER SAME AS ANY OTHERS, WILL ADD ONE IF YOU DO");
        if (!has_adjust_2){
            blockx2 =blockx1;
            blocky2 =blocky1;
        }


        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        if (blocky2==blocky3){
            blocky3++;
        }
        if (blocky1==blocky3){
            blocky3++;
        }
        block = 0;

        sub1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky1, 120-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky1, 100-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        sample5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+60.000+blocky1, 100-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky1, 120-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        sub2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky2, 120-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky2, 100-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        sample6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+60+blocky2, 100-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky2, 120-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        sub3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky3, 120-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky3, 100-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        sample7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(2+60.000+blocky3, 100-75, Point.CARTESIAN),
                                new Point(2+60.000+blocky3, 120-75, Point.CARTESIAN),
                                new Point(2+10.000, 136.000-75, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        follower.update();
        robot.update(timer.seconds());
        switch (pathState) {
            case START:

                follower.followPath(preload, true);
                robot.mIntake.setClawPos(0);
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 500, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                robot.mDeposit.setDiffyPos(-70,-90);
                setPathState(PathState.PRELOAD);
                break;
            case PRELOAD:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 850, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                    robot.mDeposit.setDiffyPos(30, -90);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(intake1,true);
                    setPathState(PathState.INTAKE1);
                }
                break;
            case INTAKE1:
                if (!follower.isBusy()) {
                    if (robot.mLift.closeEnough() || robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()) {
                        if (robot.mDeposit.servoDone() || robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()) {
                            if (pathTimer.getElapsedTimeSeconds() > 1.1 || (intaken && pathTimer.getElapsedTimeSeconds() > 0.1)) {
                                robot.mIntake.setIntakeOpenLoop(0);
                                robot.mIntake.setClawPos(1);
                                follower.followPath(sample2, true);
                                setPathState(PathState.TRANSFER2);
                            } else if (pathTimer.getElapsedTimeSeconds() > 1 || (intaken)) {
                                robot.mIntake.setClawPos(1);

                            } else if (pathTimer.getElapsedTimeSeconds() > 0) {
                                robot.mIntake.setIntakeOpenLoop(1);
                                robot.mIntake.setClawPos(0);
                                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                                robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                                robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                            }
                        } else {
                            pathTimer.resetTimer();

                        }
                        if (robot.mIntake.detectedYellow() && !intaken) {
                            intaken = true;
                            pathTimer.resetTimer();
                        }
                    } else {
                        pathTimer.resetTimer();
                        if (robot.mLift.getLiftTicks() > 700) {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});

                        }
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                    }
                }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                    robot.mIntake.setIntakeOpenLoop(1);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                break;
            case TRANSFER2:
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    pathTimer.resetTimer();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (pathTimer.getElapsedTimeSeconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                        robot.mIntake.setClawPos(0);

                    }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    pathTimer.resetTimer();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (robot.mLift.getLiftTicks()>transferendheight&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(2,timer.seconds());
                    pathTimer.resetTimer();
                    setPathState(PathState.SAMPLE2);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                break;
            case SAMPLE2:
                if (robot.mLift.getLiftTicks()>390||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (!genericboolean) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 650, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-70,-90);
                        pathTimer.resetTimer();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        genericboolean=true;
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (pathTimer.getElapsedTimeSeconds()>0.2){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            //genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            follower.followPath(intake2, true);
                            setPathState(PathState.INTAKE2);
                        }else if (pathTimer.getElapsedTimeSeconds()>0.15){
                            robot.mDeposit.setClawPos(2);
                            //robot.mIntake.setIntakeOpenLoop(1);
                            //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mIntake.setClawPos(0);
                        }
                    }else{
                        pathTimer.resetTimer();
                    }
                }else{
                    pathTimer.resetTimer();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                break;
            case INTAKE2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    if (robot.mLift.closeEnough() || robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()) {
                        if (robot.mDeposit.servoDone() || robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()) {
                            if (pathTimer.getElapsedTimeSeconds() > 1.1 || (intaken && pathTimer.getElapsedTimeSeconds() > 0.1)) {
                                robot.mIntake.setIntakeOpenLoop(0);
                                robot.mIntake.setClawPos(1);
                                follower.followPath(sample3, true);
                                setPathState(PathState.TRANSFER3);
                            } else if (pathTimer.getElapsedTimeSeconds() > 1 || (intaken)) {
                                robot.mIntake.setClawPos(1);

                            } else if (pathTimer.getElapsedTimeSeconds() > 0) {
                                robot.mIntake.setIntakeOpenLoop(1);
                                robot.mIntake.setClawPos(0);
                                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                                robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                                robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                            }
                        } else {
                            pathTimer.resetTimer();

                        }
                        if (robot.mIntake.detectedYellow() && !intaken) {
                            intaken = true;
                            pathTimer.resetTimer();
                        }
                    } else {
                        pathTimer.resetTimer();
                        if (robot.mLift.getLiftTicks() > 700) {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});

                        }
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                    }
                }else if (pathTimer.getElapsedTimeSeconds()>0.4){
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                    robot.mIntake.setIntakeOpenLoop(1);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                }
                break;
            case TRANSFER3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    pathTimer.resetTimer();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (pathTimer.getElapsedTimeSeconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                        robot.mIntake.setClawPos(0);

                    }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    pathTimer.resetTimer();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (robot.mLift.getLiftTicks()>transferendheight&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(2,timer.seconds());
                    pathTimer.resetTimer();
                    setPathState(PathState.SAMPLE3);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARESHORT.getVal(), timer.seconds());
                }
                break;
            case SAMPLE3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (robot.mLift.getLiftTicks()>390||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (!genericboolean) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 650, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-70,-90);
                        pathTimer.resetTimer();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        genericboolean=true;
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (pathTimer.getElapsedTimeSeconds()>0.2){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            //genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            follower.followPath(intake3, true);
                            setPathState(PathState.INTAKE3);
                        }else if (pathTimer.getElapsedTimeSeconds()>0.15){
                            robot.mDeposit.setClawPos(2);
                            //robot.mIntake.setIntakeOpenLoop(1);
                            //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mIntake.setClawPos(0);
                        }
                    }else{
                        pathTimer.resetTimer();
                    }
                }else{
                    pathTimer.resetTimer();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                break;
            case INTAKE3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (robot.mLift.closeEnough() || robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()) {
                        if (robot.mDeposit.servoDone() || robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()) {
                            if (pathTimer.getElapsedTimeSeconds() > 1.1 || (intaken && pathTimer.getElapsedTimeSeconds() > 0.1)) {
                                robot.mIntake.setIntakeOpenLoop(0);
                                robot.mIntake.setClawPos(1);
                                follower.followPath(sample4, true);
                                setPathState(PathState.TRANSFER4);
                            } else if (pathTimer.getElapsedTimeSeconds() > 1 || (intaken)) {
                                robot.mIntake.setClawPos(1);

                            } else if (pathTimer.getElapsedTimeSeconds() > 0) {
                                robot.mIntake.setIntakeOpenLoop(1);
                                robot.mIntake.setClawPos(0);
                                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                                robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                                robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                            }
                        } else {
                            pathTimer.resetTimer();

                        }
                        if (robot.mIntake.detectedYellow() && !intaken) {
                            intaken = true;
                            pathTimer.resetTimer();
                        }
                    } else {
                        pathTimer.resetTimer();
                        if (robot.mLift.getLiftTicks() > 700) {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});

                        }
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                    }
                }else if (pathTimer.getElapsedTimeSeconds()>0.5){
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                    robot.mIntake.setIntakeOpenLoop(1);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                }
                break;
            case TRANSFER4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    pathTimer.resetTimer();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (pathTimer.getElapsedTimeSeconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                        robot.mIntake.setClawPos(0);

                    }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    pathTimer.resetTimer();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (robot.mLift.getLiftTicks()>transferendheight&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(2,timer.seconds());
                    pathTimer.resetTimer();
                    setPathState(PathState.SAMPLE4);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                }
                break;
            case SAMPLE4:
                if (robot.mLift.getLiftTicks()>390||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (!genericboolean) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 650, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-70,-90);
                        pathTimer.resetTimer();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        genericboolean=true;
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (pathTimer.getElapsedTimeSeconds()>0.2){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            //genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            follower.followPath(sub1,true);
                            setPathState(PathState.SUB1);
                        }else if (pathTimer.getElapsedTimeSeconds()>0.15){
                            robot.mDeposit.setClawPos(2);
                            //robot.mIntake.setIntakeOpenLoop(1);
                            //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setClawPos(0);
                        }
                    }else{
                        pathTimer.resetTimer();
                    }
                }else{
                    pathTimer.resetTimer();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                break;
            case SUB1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if (detected){
                        robot.mIntake.pwmenable();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                        if (pathTimer.getElapsedTimeSeconds()> 0.1) {
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            robot.mIntake.setIntakeOpenLoop(-1);
                            //block++;
                            follower.followPath(sample5,true);
                            setPathState(PathState.TRANSFER5);
                        }else{
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setOutputLimits(-1, 1);
                            robot.mIntake.setClawPos(1);
                        }
                    }else if (pathTimer.getElapsedTimeSeconds() > 1) {


                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        //block++;
                        if (robot.mIntake.detectedImmediateYellow()||(robot.mIntake.detectedImmediateBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedImmeditaeRed()&&team.name().equals("RED"))){
                            if (!detected){
                                pathTimer.resetTimer();
                            }
                            detected= true;
                        }
                        if (!detected){
                            //failed = true;
                            robot.mIntake.setClawPos(0);
                            pathTimer.resetTimer();
                            if (!reject) {
                                robot.mIntake.setIntakeOpenLoop(0);
                            }

                            follower.followPath(sample5,true);
                            setPathState(PathState.TRANSFER5);
                        }else{
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setIntakeOpenLoop(1);
                        }


                    } else if (pathTimer.getElapsedTimeSeconds() > 0.4) {

                        robot.mIntake.setOutputLimits(-1, 1);
                        if (!reject) robot.mIntake.setClawPos(1);
                    }else if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                        if (failedtimes.equals("1")||(failedtimes.equals("01"))||(failedtimes.equals("00"))){
                            robot.mIntake.setExtendoTicks((int) (((blockx3+7)/0.033)), timer.seconds());

                        }else if(failedtimes.equals("11")||failed||failedtimes.equals("10")){
                            robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());
                        }else {
                            if (block == 0) {
                                robot.mIntake.setExtendoTicks((int) (((blockx1+7)/0.033)), timer.seconds());
                            } else {
                                robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());

                            }
                        }

                    } else if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());


                    } else if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.mIntake.pwmdisable();
                    }  else if (robot.mIntake.closeEnough() && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.5);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        if (failedtimes.equals("1")||(failedtimes.equals("01"))||failedtimes.equals("00")){
                            robot.mIntake.setExtendoTicks((int) ((blockx3/0.033)), timer.seconds());
                            //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky3 )+" VS " + drive.pose.position.y);

                        }else if(failedtimes.equals("11")||failed||failedtimes.equals("10")){
                            robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());
                            //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky2 )+" VS " + drive.pose.position.y);

                        }else {
                            if (block == 0) {
                                robot.mIntake.setExtendoTicks((int) ((blockx1/0.033)), timer.seconds());
                                //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky1 )+" VS " + drive.pose.position.y);
                            } else {
                                robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());
                                //otLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky2 )+" VS " + drive.pose.position.y);

                            }
                        }

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                        robot.mIntake.setOutputLimits(-1, 0.9);
                    }
                    if (!stopresetting) {
                        pathTimer.resetTimer();
                    }
                    if (robot.mIntake.detectedYellow()||(robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))){
                        if (!detected){
                            pathTimer.resetTimer();
                        }
                        detected= true;
                        genericboolean=true;
                        stopresetting =true;
                    }
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */

                }
                break;
            case TRANSFER5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    pathTimer.resetTimer();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (pathTimer.getElapsedTimeSeconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                        robot.mIntake.setClawPos(0);

                    }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    pathTimer.resetTimer();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (robot.mLift.getLiftTicks()>transferendheight&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(2,timer.seconds());
                    pathTimer.resetTimer();
                    setPathState(PathState.SAMPLE5UP);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                }

                break;
            case SAMPLE5UP:
                if (robot.mLift.getLiftTicks()>390||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (!genericboolean) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 650, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-70,-90);
                        pathTimer.resetTimer();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        genericboolean=true;
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if(!follower.isBusy()) {
                            /* Score Sample */
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                            setPathState(PathState.SAMPLE5DOWN);
                        }
                    }else{
                        pathTimer.resetTimer();
                    }
                }else{
                    pathTimer.resetTimer();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                break;
            case SAMPLE5DOWN:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds()>0.15){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    //robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setDiffyPos(30,-90);
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                    follower.followPath(sub2,true);
                    setPathState(PathState.SUB2);
                }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                    robot.mDeposit.setClawPos(2);
                    //robot.mDeposit.setDiffyPos(-50,-90);
                }else if (pathTimer.getElapsedTimeSeconds()>0){
                    //robot.mDeposit.setClawPos(2);
                    robot.mDeposit.setDiffyPos(-50,-90);
                }
                break;
            case SUB2:
                if(!follower.isBusy()) {
                    if (detected){
                        robot.mIntake.pwmenable();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                        if (pathTimer.getElapsedTimeSeconds()> 0.1) {
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            robot.mIntake.setIntakeOpenLoop(-1);
                            //block++;
                            follower.followPath(sample6,true);
                            setPathState(PathState.TRANSFER6);
                        }else{
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setOutputLimits(-1, 1);
                            robot.mIntake.setClawPos(1);
                        }
                    }else if (pathTimer.getElapsedTimeSeconds() > 1) {


                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        //block++;
                        if (robot.mIntake.detectedImmediateYellow()||(robot.mIntake.detectedImmediateBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedImmeditaeRed()&&team.name().equals("RED"))){
                            if (!detected){
                                pathTimer.resetTimer();
                            }
                            detected= true;
                        }
                        if (!detected){
                            //failed = true;
                            robot.mIntake.setClawPos(0);
                            pathTimer.resetTimer();
                            if (!reject) {
                                robot.mIntake.setIntakeOpenLoop(0);
                            }

                            follower.followPath(sample6,true);
                            setPathState(PathState.TRANSFER6);
                        }else{
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setIntakeOpenLoop(1);
                        }


                    } else if (pathTimer.getElapsedTimeSeconds() > 0.4) {

                        robot.mIntake.setOutputLimits(-1, 1);
                        if (!reject) robot.mIntake.setClawPos(1);
                    }else if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                        if (failedtimes.equals("1")||(failedtimes.equals("01"))||(failedtimes.equals("00"))){
                            robot.mIntake.setExtendoTicks((int) (((blockx3+7)/0.033)), timer.seconds());

                        }else if(failedtimes.equals("11")||failed||failedtimes.equals("10")){
                            robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());
                        }else {
                            if (block == 0) {
                                robot.mIntake.setExtendoTicks((int) (((blockx1+7)/0.033)), timer.seconds());
                            } else {
                                robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());

                            }
                        }

                    } else if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());


                    } else if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.mIntake.pwmdisable();
                    }  else if (robot.mIntake.closeEnough() && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.5);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        if (failedtimes.equals("1")||(failedtimes.equals("01"))||failedtimes.equals("00")){
                            robot.mIntake.setExtendoTicks((int) ((blockx3/0.033)), timer.seconds());
                            //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky3 )+" VS " + drive.pose.position.y);

                        }else if(failedtimes.equals("11")||failed||failedtimes.equals("10")){
                            robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());
                            //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky2 )+" VS " + drive.pose.position.y);

                        }else {
                            if (block == 0) {
                                robot.mIntake.setExtendoTicks((int) ((blockx1/0.033)), timer.seconds());
                                //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky1 )+" VS " + drive.pose.position.y);
                            } else {
                                robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());
                                //otLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky2 )+" VS " + drive.pose.position.y);

                            }
                        }

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                        robot.mIntake.setOutputLimits(-1, 0.9);
                    }
                    if (!stopresetting) {
                        pathTimer.resetTimer();
                    }
                    if (robot.mIntake.detectedYellow()||(robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))){
                        if (!detected){
                            pathTimer.resetTimer();
                        }
                        detected= true;
                        genericboolean=true;
                        stopresetting =true;
                    }
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */

                }
                break;
            case TRANSFER6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    pathTimer.resetTimer();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (pathTimer.getElapsedTimeSeconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                        robot.mIntake.setClawPos(0);

                    }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    pathTimer.resetTimer();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (robot.mLift.getLiftTicks()>transferendheight&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(2,timer.seconds());
                    pathTimer.resetTimer();
                    setPathState(PathState.SAMPLE6UP);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                }

                break;
            case SAMPLE6UP:
                if (robot.mLift.getLiftTicks()>390||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (!genericboolean) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 650, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-70,-90);
                        pathTimer.resetTimer();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        genericboolean=true;
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if(!follower.isBusy()) {
                            /* Score Sample */
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                            setPathState(PathState.SAMPLE6DOWN);
                        }
                    }else{
                        pathTimer.resetTimer();
                    }
                }else{
                    pathTimer.resetTimer();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                break;
            case SAMPLE6DOWN:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds()>0.15){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    //robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setDiffyPos(30,-90);
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                    follower.followPath(sub3,true);
                    failedtimes="00";
                    setPathState(PathState.SUB3);
                }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                    robot.mDeposit.setClawPos(2);
                    //robot.mDeposit.setDiffyPos(-50,-90);
                }else if (pathTimer.getElapsedTimeSeconds()>0){
                    //robot.mDeposit.setClawPos(2);
                    robot.mDeposit.setDiffyPos(-50,-90);
                }
                break;
            case SUB3:
                if(!follower.isBusy()) {
                    if (detected){
                        robot.mIntake.pwmenable();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                        if (pathTimer.getElapsedTimeSeconds()> 0.1) {
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            robot.mIntake.setIntakeOpenLoop(-1);
                            //block++;
                            follower.followPath(sample7,true);
                            setPathState(PathState.TRANSFER7);
                        }else{
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setOutputLimits(-1, 1);
                            robot.mIntake.setClawPos(1);
                        }
                    }else if (pathTimer.getElapsedTimeSeconds() > 1) {


                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        //block++;
                        if (robot.mIntake.detectedImmediateYellow()||(robot.mIntake.detectedImmediateBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedImmeditaeRed()&&team.name().equals("RED"))){
                            if (!detected){
                                pathTimer.resetTimer();
                            }
                            detected= true;
                        }
                        if (!detected){
                            //failed = true;
                            robot.mIntake.setClawPos(0);
                            pathTimer.resetTimer();
                            if (!reject) {
                                robot.mIntake.setIntakeOpenLoop(0);
                            }

                            follower.followPath(sample7,true);
                            setPathState(PathState.TRANSFER7);
                        }else{
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setIntakeOpenLoop(1);
                        }


                    } else if (pathTimer.getElapsedTimeSeconds() > 0.4) {

                        robot.mIntake.setOutputLimits(-1, 1);
                        if (!reject) robot.mIntake.setClawPos(1);
                    }else if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                        if (failedtimes.equals("1")||(failedtimes.equals("01"))||(failedtimes.equals("00"))){
                            robot.mIntake.setExtendoTicks((int) (((blockx3+7)/0.033)), timer.seconds());

                        }else if(failedtimes.equals("11")||failed||failedtimes.equals("10")){
                            robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());
                        }else {
                            if (block == 0) {
                                robot.mIntake.setExtendoTicks((int) (((blockx1+7)/0.033)), timer.seconds());
                            } else {
                                robot.mIntake.setExtendoTicks((int) (((blockx2+7)/0.033)), timer.seconds());

                            }
                        }

                    } else if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());


                    } else if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.mIntake.pwmdisable();
                    }  else if (robot.mIntake.closeEnough() && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.5);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        if (failedtimes.equals("1")||(failedtimes.equals("01"))||failedtimes.equals("00")){
                            robot.mIntake.setExtendoTicks((int) ((blockx3/0.033)), timer.seconds());
                            //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky3 )+" VS " + drive.pose.position.y);

                        }else if(failedtimes.equals("11")||failed||failedtimes.equals("10")){
                            robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());
                            //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky2 )+" VS " + drive.pose.position.y);

                        }else {
                            if (block == 0) {
                                robot.mIntake.setExtendoTicks((int) ((blockx1/0.033)), timer.seconds());
                                //BotLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky1 )+" VS " + drive.pose.position.y);
                            } else {
                                robot.mIntake.setExtendoTicks((int) ((blockx2/0.033)), timer.seconds());
                                //otLog.logD("TARGET VS ACTUAL:   ", (60.5 + blocky2 )+" VS " + drive.pose.position.y);

                            }
                        }

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                        robot.mIntake.setOutputLimits(-1, 0.9);
                    }
                    if (!stopresetting) {
                        pathTimer.resetTimer();
                    }
                    if (robot.mIntake.detectedYellow()||(robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))){
                        if (!detected){
                            pathTimer.resetTimer();
                        }
                        detected= true;
                        genericboolean=true;
                        stopresetting =true;
                    }
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */

                }
                break;
            case TRANSFER7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    pathTimer.resetTimer();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (pathTimer.getElapsedTimeSeconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (pathTimer.getElapsedTimeSeconds()>0.2){
                        robot.mIntake.setClawPos(0);

                    }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    pathTimer.resetTimer();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (robot.mLift.getLiftTicks()>transferendheight&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(2,timer.seconds());
                    pathTimer.resetTimer();
                    setPathState(PathState.SAMPLE7UP);
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                }

                break;
            case SAMPLE7UP:
                if (robot.mLift.getLiftTicks()>390||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (!genericboolean) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 650, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-70,-90);
                        pathTimer.resetTimer();
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        genericboolean=true;
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if(!follower.isBusy()) {
                            /* Score Sample */
                            /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                            setPathState(PathState.SAMPLE7DOWN);
                        }
                    }else{
                        pathTimer.resetTimer();
                    }
                }else{
                    pathTimer.resetTimer();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                break;
            case SAMPLE7DOWN:
                if (pathTimer.getElapsedTimeSeconds()>0.15){
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOEND.getVal(),800, new double[] {1,2,3,4,4,4,3,2,1,1});
                    robot.mIntake.setExtendoPos(0,timer.seconds());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mDeposit.setDiffyPos(30,-90);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                    //robot.mDeposit.setClawPos(1);
                    robot.mIntake.setClawPos(0);
                    setPathState(PathState.END);
                }else if (pathTimer.getElapsedTimeSeconds()>0.1){
                    robot.mDeposit.setClawPos(2);
                    //robot.mDeposit.setDiffyPos(-50,-90);
                }else if (pathTimer.getElapsedTimeSeconds()>0){
                    //robot.mDeposit.setClawPos(2);
                    robot.mDeposit.setDiffyPos(-50,-90);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                break;
        }
        follower.drawOnDashBoard();
    }
    public void setPathState(PathState pState) {
        pathState = pState;
        intaken = false;
        detected=false;
        reject=false;
        stopresetting=false;
        genericboolean=false;
        pathTimer.resetTimer();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {
    }

}
