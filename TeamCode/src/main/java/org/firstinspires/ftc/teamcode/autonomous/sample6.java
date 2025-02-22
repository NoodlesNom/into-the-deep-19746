package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.gf.OldAutoMaster.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "6 sample 💸💸", group = "Autonomous")

public class sample6 extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    private boolean genericboolean = false;

    public static int blockx1 = 2;
    public static int blockx2 = 2;
    private StickyButton intakeposup = new StickyButton();
    private StickyButton intakeposdown = new StickyButton();
    private StickyButton intakeposleft = new StickyButton();
    private StickyButton switchblock = new StickyButton();
    private StickyButton intakeposright = new StickyButton();
    public static int blocky1 = 10;
    public static int blocky2 = 10;
    private PinpointDrive drive;

    private int block = 0;
    ElapsedTime generaltimer = new ElapsedTime();
    Deadline logging = new Deadline(250, TimeUnit.MILLISECONDS);
    boolean debug = true;
    ElapsedTime loopTimer = new ElapsedTime();
    public class robotController {

        public class Update implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.update(timer.seconds());
                if(debug) {
                    if(logging.hasExpired())
                    {
                        logging.reset();
                        String debugMsg = robot.getTelem(timer.seconds());
                        debugMsg += String.format("loopTime = %4.0f\n", loopTimer.milliseconds());
                        telemetry.addLine(debugMsg);
                        BotLog.logD("BSDbg", debugMsg);
                    }
                    loopTimer.reset();
                }
                telemetry.update();
                return true;
            }
        }

        public Action updateRobot() {
            return new Update();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(2);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setClawPos(1);
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class PivotDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 850, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                robot.mDeposit.setDiffyPos(30, -90);
                return false;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

        public class PivotIdle implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 750, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return false;
            }
        }

        public Action pivotIdle() {
            return new PivotIdle();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 400, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return robot.mDeposit.servoDone();
            }
        }
        public Action pivotUp() {
            return new PivotUp();
        }

        public class DiffyIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(70,-90);
                return false;
            }
        }
        public Action diffyIntake() {
            return new DiffyIntake();
        }

        public class DiffyPlace implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(0,-90);
                return false;
            }
        }
        public Action diffyPlace() {
            return new DiffyPlace();
        }

        public class DiffyRelease implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                return false;
            }
        }
        public Action diffyRelease() {
            return new DiffyRelease();
        }

        public class LiftDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                return robot.mLift.getLiftTicks()<600;
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftIntakeInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftIntakeInstant() {
            return new LiftIntakeInstant();
        }


        public class LiftClearInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
                return false;
            }
        }

        public Action liftClearInstant() {
            return new LiftClearInstant();
        }



        public class LiftPlaceInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(1, timer.seconds());
                return false;
            }
        }

        public Action liftPlaceInstant() {
            return new LiftPlaceInstant();
        }


        public class Pull implements Action {
            private boolean stopresetting = false;
            private boolean reject = false;
            private boolean detected = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if ((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    reject = true;
                }

                if (generaltimer.seconds()>1.9){

                    if(!reject){robot.mIntake.setIntakeOpenLoop(0);}
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setExtendoPos(0,timer.seconds());
                    block++;
                    return false;
                }else if(generaltimer.seconds()>1.8){
                    robot.mIntake.setOutputLimits(-1,1);
                    robot.mIntake.setClawPos(1);
                }else if(generaltimer.seconds()>1){
                    if (block==0) {
                        robot.mIntake.setExtendoTicks((int) (((blockx1+5) * 25.4) / 1.25), timer.seconds());
                    }else{
                        robot.mIntake.setExtendoTicks((int) (((blockx2+10) * 25.4) / 1.25), timer.seconds());
                    }
                }else if(generaltimer.seconds()>0.7){
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    if(!reject){robot.mIntake.setIntakeOpenLoop(1);}


                }else if(generaltimer.seconds()>0.1){
                    if (block==0) {
                        robot.mIntake.setExtendoTicks((int) (((blockx1-2) * 25.4) / 1.25), timer.seconds());
                    }else{
                        robot.mIntake.setExtendoTicks((int) (((blockx2-2) * 25.4) / 1.25), timer.seconds());
                    }
                }else if(robot.mIntake.closeEnough()&&genericboolean){
                    if(!reject){robot.mIntake.setIntakeOpenLoop(0.5);}

                    stopresetting=true;
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                }else if(!genericboolean&&robot.mIntake.getTargetExtendoIdx()!=Intake.EXTEND_POS.INTAKING.getVal()) {
                    if (block==0) {
                        robot.mIntake.setExtendoTicks((int) ((blockx1 * 25.4) / 1.25), timer.seconds());
                        BotLog.logD("TARGET VS ACTUAL: ", "" + (60.5+blocky1-10), drive.pose.position.y);
                    }else{
                        robot.mIntake.setExtendoTicks((int) ((blockx2 * 25.4) / 1.25), timer.seconds());
                        BotLog.logD("TARGET VS ACTUAL: ", "" + (60.5+blocky2-10), drive.pose.position.y);

                    }

                    genericboolean = true;
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                    robot.mIntake.setOutputLimits(-1,0.5);
                }
                if (!stopresetting){
                    generaltimer.reset();
                }
                return true;
            }
        }

        public Action pull() {
            return new Pull();
        }


        public class Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!robot.mIntake.closeEnoughAuto()||robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.STOWED.getVal()) {
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    generaltimer.reset();
                }
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (generaltimer.seconds()>0.4){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                    }else if (generaltimer.seconds()>0.3){
                        robot.mIntake.setClawPos(0);
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (generaltimer.seconds()>0.2){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    generaltimer.reset();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (generaltimer.seconds()>0.6&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0,timer.seconds());
                    generaltimer.reset();
                    return false;
                }
                return true;
            }
        }

        public Action transfer() {
            return new Transfer();
        }


        public class Sample1andIntake2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.getLiftTicks()>500||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()&&!robot.mLift.closeEnough()) {
                        robot.mIntake.setClawPos(0);
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        generaltimer.reset();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1.2){
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setClawPos(1);
                            return false;
                        }
                        else if (generaltimer.seconds()>1.1){
                            robot.mIntake.setClawPos(1);
                        }else if (generaltimer.seconds()>1){
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                        }else if (generaltimer.seconds()>0.15){
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                        }else if (generaltimer.seconds()>0.1){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>300){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample1andIntake2() {
            return new Sample1andIntake2();
        }

        public class Sample3 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.getLiftTicks()>500||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()&&!robot.mLift.closeEnough()) {
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        generaltimer.reset();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.15){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            return false;
                        }else if (generaltimer.seconds()>0.1){
                            robot.mDeposit.setClawPos(2);
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                            robot.mIntake.setClawPos(0);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>300){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample3() {
            return new Sample3();
        }

        public class Sample4 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.getLiftTicks()>500||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()&&!robot.mLift.closeEnough()) {
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        generaltimer.reset();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.15){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setExtendoTicks((int) ((blockx1 * 25.4) / 1.25), timer.seconds());

                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            return false;
                        }else if (generaltimer.seconds()>0.1){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>300){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample4() {
            return new Sample4();
        }

        public class Sample5 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.getLiftTicks()>500||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()&&!robot.mLift.closeEnough()) {
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        generaltimer.reset();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.25){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setExtendoTicks((int) ((blockx2 * 25.4) / 1.25), timer.seconds());

                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            return false;
                        }else if (generaltimer.seconds()>0.2){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>300){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample5() {
            return new Sample5();
        }

        public class Sample6 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (robot.mLift.getLiftTicks()>500||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&(!robot.mLift.closeEnough()||robot.mDeposit.getPivotPos()!= Deposit.PIVOT_POS.AUTOSAMPLE.getVal())) {
                        robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLE.getVal(), 900, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(0,-90);
                        generaltimer.reset();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.3){
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOEND.getVal(),800, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setExtendoPos(0,timer.seconds());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                            robot.mDeposit.setDiffyPos(20,90);
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                            robot.mDeposit.setClawPos(1);
                            robot.mIntake.setClawPos(0);
                            robot.mDeposit.setDiffyPos(30,-90);
                            genericboolean = true;
                            return false;
                        }else if (generaltimer.seconds()>0.2){
                            robot.mDeposit.setClawPos(2);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>300){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action sample6() {
            return new Sample6();
        }


        public class TelopPrep implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (generaltimer.seconds()>0.5){
                    robot.mLift.zerofinish(timer.seconds());
                    robot.mIntake.zerofinish(timer.seconds());
                    return false;
                } else if (generaltimer.seconds()>0.3) {
                    robot.mIntake.rezero();
                    robot.mLift.rezero();
                }else{
                    robot.mIntake.setExtendoOpenLoop(-0.7);
                    robot.mLift.setOpenLoop(-0.7);
                }
                return true;
            }
        }


        public Action teleopPrep() {

            return new TelopPrep();
        }
        public class RobotReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                robot.mIntake.setExtendoOpenLoop(-0.4);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOEND.getVal());
                robot.mDeposit.setDiffyPos(0,0);
                robot.mDeposit.setClawPos(1);

                return false;
            }
        }

        public Action robotReset() {
            return new RobotReset();
        }


        public class Intakeing implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1.3){
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setClawPos(1);
                            return false;
                        }
                        else if (generaltimer.seconds()>1.1){
                            robot.mIntake.setClawPos(1);
                        }else if (generaltimer.seconds()>0.9){
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                        }else if (generaltimer.seconds()>0){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setClawPos(0);
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),900, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>700){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 800, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }


        public Action intakeing() {
            return new Intakeing();
        }


        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                generaltimer.reset();
                genericboolean = false;
                return false;
            }
        }

        public Action resetTimer() {
            return new ResetTimer();
        }

        public class LiftUpInstant implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action liftUpInstant() {
            return new LiftUpInstant();
        }

        public class LiftTransferPrepInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                robot.mIntake.setClawPos(0);
                return false;
            }
        }

        public Action liftTransferPrepInstant() {
            return new LiftTransferPrepInstant();
        }

        public class LiftDownInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return false;
            }
        }

        public Action liftDownInstant() {
            return new LiftDownInstant();
        }

        public class ExtendoOutInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());

                robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                return false;
            }
        }

        public Action extendoOutInstant() {
            return new ExtendoOutInstant();
        }

        public class ExtendoPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action extendoPrepareInstant() {
            return new ExtendoPrepareInstant();
        }

        public class IntakeDownInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                return false;
            }
        }

        public Action intakeDownInstant() {
            return new IntakeDownInstant();
        }
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        robotController controller = new robotController();
        Pose2d initialPose = new Pose2d((24+7.25), 7.25, Math.toRadians(90));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        drive = new PinpointDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action preload_intake1 = drive.actionBuilder(initialPose)
                .stopAndAdd(new SequentialAction(
                        controller.liftUpInstant(),
                        controller.closeClaw(),
                        controller.diffyPlace(),
                        controller.pivotIdle()
                ))
                .strafeToLinearHeading(new Vector2d(11,18), Math.toRadians(69))
                .stopAndAdd(new SequentialAction(
                        controller.liftUp(),
                        controller.pivotUp(),
                        controller.extendoPrepareInstant(),
                        controller.intakeDownInstant()

                ))
                .waitSeconds(0.8)
                .stopAndAdd(new SequentialAction(
                        controller.openClaw()
                ))
                .waitSeconds(0.1)
                .stopAndAdd(new SequentialAction(
                        controller.pivotDown(),
                        controller.liftTransferPrepInstant()
                ))
                .strafeToLinearHeading(new Vector2d(12,20), Math.toRadians(69))
                .stopAndAdd(controller.resetTimer())
                .stopAndAdd(controller.intakeing())
                .build();

        Action sample1_intake2 = drive.actionBuilder(new Pose2d(12,20, Math.toRadians(69)))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(7,19), Math.toRadians(86))
                .build();

        Action intake4 = drive.actionBuilder(new Pose2d(7,19, Math.toRadians(86)))
                .strafeToLinearHeading(new Vector2d(10,19), Math.toRadians(110))
                .build();

        Action sample3 = drive.actionBuilder(new Pose2d(10,19, Math.toRadians(110)))
                .strafeToLinearHeading(new Vector2d(7,19), Math.toRadians(83))
                .build();



        boolean block1 = true;
        boolean has_adjust_2 = false;
        while (!isStopRequested() && !opModeIsActive()) {
            intakeposleft.update(gamepad1.dpad_left);
            intakeposright.update(gamepad1.dpad_right);
            intakeposup.update(gamepad1.dpad_up);
            intakeposdown.update(gamepad1.dpad_down);
            switchblock.update(gamepad1.x);
            robot.update(timer.seconds());
            timer.reset();
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
                block1=!block1;
            }
            telemetry.addLine("USE DPAD TO ADJUST INTAKE POS BY INCHES");
            telemetry.addLine("PRESS X TO ADJUST WHICH INTAKE");
            telemetry.addLine("IF DONT ADJUST INTAKE 2, WILL BE SAME AS 1. 2 ADJUSTED = " +has_adjust_2);
            if (block1){
                telemetry.addLine("ADJUSTING FIRST INTAKE");
                if (intakeposup.getState()){
                    blocky1 +=1;
                }else if (intakeposdown.getState()&& blocky1 >0){
                    blocky1 -=1;
                }else if (intakeposleft.getState()&& blockx1 >0){
                    blockx1 -=1;
                }else if (intakeposright.getState()){
                    blockx1 +=1;
                }
            }else{
                has_adjust_2 = true;
                telemetry.addLine("ADJUSTING SECOND INTAKE");
                if (intakeposup.getState()){
                    blocky2 +=1;
                }else if (intakeposdown.getState()&& blocky2 >0){
                    blocky2 -=1;
                }else if (intakeposleft.getState()&& blockx2 >0){
                    blockx2 -=1;
                }else if (intakeposright.getState()){
                    blockx2 +=1;
                }
            }

            telemetry.addLine("CURR INTAKE POS 1: "+ (blocky1-10) + ", " + blockx1);
            telemetry.addLine("CURR INTAKE POS 2: "+ (blocky2-10) + ", " + blockx2);
            if (!has_adjust_2){
                blockx2 =blockx1;
                blocky2 =blocky1;
            }


            telemetry.update();

        }

        Action sub1 = drive.actionBuilder(new Pose2d(7,19, Math.toRadians(83)))
                .splineTo(new Vector2d(24,48), Math.toRadians(45),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineTo(new Vector2d(45,60.5+ blocky1 -10), Math.toRadians(0),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))

                .splineTo(new Vector2d(48,60.5+ blocky1 -10), Math.toRadians(0),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))
                //splineTo(new Vector2d(55,60.5+ blocky1 -10), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                .build();

        Action sample5 = drive.actionBuilder(new Pose2d(48,60.5+ blocky1 -10, Math.toRadians(0)))
                .setReversed(true)
                .splineTo(new Vector2d(10,18), Math.toRadians(70-180),new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-30, 80))
                .build();

        Action park = drive.actionBuilder(new Pose2d(12,20, Math.toRadians(83)))
                .splineTo(new Vector2d(42,60.5), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                .splineTo(new Vector2d(47,60.5), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))

                .build();
        Action sub2 = drive.actionBuilder(new Pose2d(10,18, Math.toRadians(70)))
                .setReversed(false)
                .splineTo(new Vector2d(15,41), Math.toRadians(70),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))
                .splineTo(new Vector2d(45,60.5+ blocky2 -10), Math.toRadians(0),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))

                .splineTo(new Vector2d(48,60.5+ blocky2 -10), Math.toRadians(0),new TranslationalVelConstraint(55 ), new ProfileAccelConstraint(-45, 80))
                //.splineTo(new Vector2d(55,60.5+ blocky2 -10), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                .build();
        Action sample6 = drive.actionBuilder(new Pose2d(48,60.5+ blocky2 -10, Math.toRadians(0)))
                .setReversed(true)
                .splineTo(new Vector2d(10,18), Math.toRadians(70-180),new TranslationalVelConstraint(50 ), new ProfileAccelConstraint(-30, 80))

                .build();

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        robot.mIntake.setExtendoPos(0, timer.seconds());
        blockx1+=7;
        blockx2+=7;
        blocky1+=0;
        blocky2+=0;
        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        controller.updateRobot(),
                        new SequentialAction(
                                preload_intake1,
                                new ParallelAction(
                                        sample1_intake2,
                                        new SequentialAction(
                                                controller.resetTimer(),
                                                controller.transfer()
                                        )
                                ),
                                controller.resetTimer(),
                                controller.sample1andIntake2(),
                                controller.resetTimer(),
                                controller.transfer(),
                                controller.resetTimer(),
                                controller.sample3(),
                                intake4,
                                controller.resetTimer(),
                                controller.intakeing(),
                                new ParallelAction(
                                        sample3,
                                        new SequentialAction(
                                                controller.resetTimer(),
                                                controller.transfer()
                                                )
                                ),
                                controller.sample4(),
                                controller.resetTimer(),
                                sub1,
                                controller.resetTimer(),
                                controller.pull(),
                                new ParallelAction(
                                        sample5,
                                        new SequentialAction(
                                                controller.resetTimer(),
                                                controller.transfer(),
                                                controller.resetTimer(),
                                                controller.sample5()
                                        )
                                ),
                                controller.resetTimer(),
                                sub2,
                                controller.resetTimer(),
                                controller.pull(),
                                new ParallelAction(
                                        sample6,
                                        new SequentialAction(
                                                controller.resetTimer(),
                                                controller.transfer(),
                                                controller.resetTimer(),
                                                controller.sample6()
                                        )
                                ),


                                park,
                                controller.teleopPrep()






                        )

                )
        );
        robot.stop();
    }
}