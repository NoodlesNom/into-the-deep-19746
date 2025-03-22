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
import com.acmerobotics.roadrunner.SleepAction;
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
@Autonomous(name = "6 SPEC + EXTRA TRIP", group = "Autonomous")


public class spec6_extra extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;
    ElapsedTime generaltimer = new ElapsedTime();
    boolean debug = true;

    private boolean failed = true;
    private boolean genericboolean = false;
    public static double blockx = 0;
    public static int blocky = 2;

    private StickyButton intakeposup = new StickyButton();
    private StickyButton intakeposdown = new StickyButton();
    private StickyButton intakeposleft = new StickyButton();
    private StickyButton switchblock = new StickyButton();
    private StickyButton intakeposright = new StickyButton();
    ElapsedTime loopTimer = new ElapsedTime();
    Deadline logging = new Deadline(100, TimeUnit.MILLISECONDS);

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
                        debugMsg+="\nlift height: "+ robot.mLift.getLiftTicks();
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
                robot.mDeposit.setClawPos(3);
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
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                return false;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal());
                return false;
            }
        }
        public Action pivotUp() {
            return new PivotUp();
        }

        public class DiffyIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(80,-90);
                return false;
            }
        }
        public Action diffyIntake() {
            return new DiffyIntake();
        }
        public class DiffyPlace implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(30,70);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECSLAM.getVal());
                robot.mLift.setTargetPos(2, timer.seconds());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlace() {
            return new DiffyPlace();
        }
        public class DiffyPlaceFirst implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(90,90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOCLEAR.getVal());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlaceFirst() {
            return new DiffyPlaceFirst();
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
                if ((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow()){
                    if (!detected){
                        generaltimer.reset();
                    }
                    detected = true;
                    BotLog.logD("deteted", detected+"");
                }
                if (detected){
                    robot.mIntake.pwmenable();

                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());

                    if (generaltimer.seconds() > 0.2) {
                        robot.mIntake.pwmenable();

                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                        return false;
                    } else if (generaltimer.seconds() > 0.1) {

                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else{
                        robot.mIntake.setIntakeOpenLoop(1);
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                    }
                }else {
                    if (generaltimer.seconds() > 1.5) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        return false;

                    } else if (generaltimer.seconds() > 1.4) {
                        robot.mIntake.pwmenable();
                        robot.mIntake.setOutputLimits(-1, 1);
                        robot.mIntake.setClawPos(1);
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (generaltimer.seconds() > 0.6) {

                        robot.mIntake.setExtendoTicks((int) (((blocky + 5)/0.033)), timer.seconds());
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(1);
                        }
                    } else if (generaltimer.seconds() > 0.4) {
                        //robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());



                    } else if (generaltimer.seconds() > 0.15) {
                        robot.mIntake.pwmdisable();
                    } else if (robot.mIntake.closeEnough() && genericboolean) {
                        if (!reject) {
                            robot.mIntake.setIntakeOpenLoop(0.6);
                            robot.mIntake.setExtendoOpenLoop(0);
                        }

                        stopresetting = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());

                    } else if (!genericboolean && robot.mIntake.getTargetExtendoIdx() != Intake.EXTEND_POS.INTAKING.getVal()) {
                        robot.mIntake.setExtendoTicks((int) (((blocky)/0.033)), timer.seconds());

                        genericboolean = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                        robot.mIntake.setOutputLimits(-1, 1);

                    }
                    if (!stopresetting) {
                        generaltimer.reset();
                    }
                }
                return true;
            }
        }
        public Action pull() {
            return new Pull();
        }


        public class DiffyPlaceLast implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(90,90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECPUSH.getVal());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlaceLast() {
            return new DiffyPlaceLast();
        }

        public class subPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoTicks((int) ((blocky /0.033)), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                return false;
            }
        }

        public Action subPrepareInstant() {
            return new subPrepareInstant();
        }

        public class intakePrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                return false;
            }
        }

        public Action intakePrepareInstant() {
            return new intakePrepareInstant();
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
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action liftClearInstant() {
            return new LiftClearInstant();
        }

        public class RobotReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                //robot.mIntake.setExtendoOpenLoop(-0.4);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                robot.mDeposit.setDiffyPos(0,0);

                return false;
            }
        }

        public Action robotReset() {
            return new RobotReset();
        }

        public class Intakeing implements Action {

            private boolean intaken = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setOutputLimits(-1,0.8);
                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1.2||(intaken&&generaltimer.seconds()>0.1)){
                            robot.mIntake.setIntakeOpenLoop(0);

                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            robot.mIntake.setOutputLimits(-1,1);
                            return false;
                        }else if  (intaken){
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                            robot.mIntake.setClawPos(1);
                        }else if (generaltimer.seconds()>0){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setClawPos(0);
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                        }

                    }else{
                        generaltimer.reset();
                    }
                    if (((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))||robot.mIntake.detectedYellow())&&!intaken){
                        intaken = true;
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                }
                return true;
            }
        }


        public Action intakeing() {
            return new Intakeing();
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
                    if (generaltimer.seconds()>0.3){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TALLAUTOSAMPLE.getVal(), timer.seconds());
                    }else if (generaltimer.seconds()>0.2){
                        robot.mIntake.setClawPos(0);
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }else if (generaltimer.seconds()>0.1){
                        robot.mDeposit.setClawPos(1);
                        robot.mIntake.setIntakeOpenLoop(0);
                    }else{
                        robot.mIntake.setExtendoOpenLoop(-0.7);
                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.TALLAUTOSAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    generaltimer.reset();
                    robot.mIntake.setIntakeOpenLoop(-1);
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
                }

                if (generaltimer.seconds()>0.5&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TALLAUTOSAMPLE.getVal()){
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
        public class IntakeingSample implements Action {

            private boolean intaken = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setOutputLimits(-1,1);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFER.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.9||(intaken&&generaltimer.seconds()>0.1)){
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setClawPos(1);
                            return false;
                        }else if (generaltimer.seconds()>0.8||(intaken)){
                            robot.mIntake.setClawPos(1);

                        }else if (generaltimer.seconds()>0){
                            robot.mIntake.setIntakeOpenLoop(1);
                            robot.mIntake.setClawPos(0);
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),900, new double[] {1,2,3,4,4,4,3,2,1,1});
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKINGTALL.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                        }
                    }else{
                        generaltimer.reset();

                    }
                    if ((robot.mIntake.detectedRed()||robot.mIntake.detectedYellow()||robot.mIntake.detectedBlue())&&!intaken){
                        intaken = true;
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                }
                return true;
            }
        }


        public Action intakeingSample() {
            return new IntakeingSample();
        }

        public class Sample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (robot.mLift.getLiftTicks()>550||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (!genericboolean&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.AUTOSAMPLE.getVal()&&robot.mDeposit.getPivotPos() != Deposit.PIVOT_POS.TRANSFER.getVal()&&!robot.mLift.closeEnough()) {
                        //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLENOSLAM.getVal(), 500, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                        robot.mDeposit.setDiffyPos(-50,-90);
                        generaltimer.reset();
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    }
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>0.2){
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30,-90);
                            genericboolean = true;
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(),1200, new double[] {1,2,3,4,4,4,3,2,1,1});
                            return false;
                        }else if (generaltimer.seconds()>0.15){
                            robot.mDeposit.setClawPos(2);
                            //robot.mIntake.setIntakeOpenLoop(1);
                            //robot.mIntake.setExtendoPos(Intake.EXTEND_POS.INTAKING.getVal(), timer.seconds());
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                            robot.mIntake.setClawPos(0);
                        }
                    }else{
                        generaltimer.reset();
                    }
                }else{
                    generaltimer.reset();
                    if (robot.mLift.getLiftTicks()>250){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSAMPLENOSLAM.getVal(), 600, new double[] {1,2,3,4,4,4,3,2,1,1});

                    }
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSAMPLE.getVal(), timer.seconds());
                }
                return true;
            }
        }

        public Action sample() {
            return new Sample();
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


        public class ClearWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (generaltimer.seconds()>1){
                    generaltimer.reset();
                }else if (generaltimer.seconds()>0.4){
                    robot.mDeposit.setDiffyPos(-20, 70);
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0, timer.seconds());
                    //robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSPECANGLED.getVal(), timer.seconds());
                    return false;
                }else if (generaltimer.seconds()>0.1) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSPECANGLED.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                }else{
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSPECANGLED.getVal(), timer.seconds());
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.SHOOTLOW.getVal());
                    //robot.mIntake.setExtendoPos(0, timer.seconds());
                }
                return true;
            }
        }

        public Action clearWall() {
            return new ClearWall();
        }


        public class ClearWallLast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (generaltimer.seconds()>1){
                    generaltimer.reset();
                }else if (generaltimer.seconds()>0.3){
                    robot.mDeposit.setDiffyPos(0, 90);
                    //robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSPECANGLED.getVal(), timer.seconds());
                    return false;
                }else if (generaltimer.seconds()>0.1) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TALLAUTOSAMPLE.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mIntake.setIntakeOpenLoop(0);
                }else{
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TALLAUTOSAMPLE.getVal(), timer.seconds());
                    //robot.mIntake.setIntakeOpenLoop(-0.8);
                    //robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                }
                return true;
            }
        }

        public Action clearWallLast() {
            return new ClearWall();
        }

        public class IntakeReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                if (generaltimer.seconds()>0.1){
                    //robot.mIntake.setExtendoOpenLoop(0);
                    //robot.mIntake.rezero();
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 900 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mDeposit.setClawPos(3);
                    robot.mDeposit.setDiffyPos(80,-90);
                    return false;
                }else{
                    //robot.mIntake.setExtendoOpenLoop(-0.5);
                }
                return true;            }
        }

        public Action intakeReset() {
            return new IntakeReset();
        }


        public class IntakeResetLast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                if (generaltimer.seconds()>0.1){
                    //robot.mIntake.setExtendoOpenLoop(0);
                    //robot.mIntake.rezero();
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 900 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mDeposit.setClawPos(3);
                    robot.mDeposit.setDiffyPos(30,-90);
                    return false;
                }else{
                    //robot.mIntake.setExtendoOpenLoop(-0.5);
                }
                return true;            }
        }

        public Action intakeResetLast() {
            return new IntakeResetLast();
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
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftUpInstant() {
            return new LiftUpInstant();
        }

        public class SpecPlace implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                robot.mDeposit.setDiffyPos(-40, 67);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TALLSPEC.getVal(), 600, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return false;
            }
        }

        public Action specPlace() {
            return new SpecPlace();
        }

        public class PivotShoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action pivotShoot() {
            return new PivotShoot();
        }
        public class PivotShootLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.SHOOTLOW.getVal());
                return false;
            }
        }

        public Action pivotShootLow() {
            return new PivotShootLow();
        }

        public class PivotShootLaunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.SHOOTLOW.getVal());
                robot.mIntake.setClawPos(0);
                robot.mIntake.setIntakeOpenLoop(-1);
                return false;
            }
        }

        public Action pivotShootLaunch() {
            return new PivotShootLaunch();
        }

        public class ExtendoPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                return false;
            }
        }

        public Action extendoPrepareInstant() {
            return new ExtendoPrepareInstant();
        }
        public class ExtendoInInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(1);
                robot.mDeposit.setClawPos(0);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                robot.mDeposit.setDiffyPos(80,-90);
                robot.mIntake.setExtendoTicks((int) (2/0.033), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.SHOOTLOW.getVal());
                return false;
            }
        }

        public Action extendoInInstant() {
            return new ExtendoInInstant();
        }

        public class ExtendoIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //robot.mIntake.setClawPos(0);
                //robot.mDeposit.setClawPos(0);
                //robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                //robot.mDeposit.setDiffyPos(80,-90);
                robot.mIntake.setExtendoPos(0, timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                return false;
            }
        }

        public Action extendoIn() {
            return new ExtendoIn();
        }


        public class Shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                if (!robot.mIntake.closeEnoughAuto()){
                    generaltimer.reset();
                }
                if (generaltimer.seconds()>0.4) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);
                    //robot.mLift.setTargetPos(1, timer.seconds());
                    robot.mDeposit.setDiffyPos(80,-90);
                    return false;
                }else  if (generaltimer.seconds()>0.1) {
                    robot.mIntake.setIntakeOpenLoop(-0.7);
                }else if   (robot.mIntake.closeEnoughAuto()){

                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoOpenLoop(-0.6);
                    robot.mDeposit.setDiffyPos(-80,90);
                }else {
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    robot.mLift.setTargetPos(2, timer.seconds());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());

                }


                return true;
            }
        }

        public Action shoot() {
            return new Shoot();
        }
        public class ShootLast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.SHOOTLOW.getVal());
                if (!robot.mIntake.closeEnoughAuto()){
                    generaltimer.reset();
                }
                if (generaltimer.seconds()>0.4) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);

                    return false;
                }else  if (generaltimer.seconds()>0.1) {
                    robot.mIntake.setIntakeOpenLoop(-0.7);
                }else if   (robot.mIntake.closeEnoughAuto()){

                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoOpenLoop(-0.6);
                }else {
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setIntakeOpenLoop(-0.7);
                    robot.mIntake.setExtendoTicks((int) (2/0.033), timer.seconds());

                }


                return true;
            }
        }

        public Action shootLast() {
            return new ShootLast();
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
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        robotController controller = new robotController();
        Pose2d initialPose = new Pose2d((10-7.25), 7.25, Math.toRadians(90));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        PinpointDrive drive = new PinpointDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;



        Action sub1 = drive.actionBuilder(new Pose2d(45,7, Math.toRadians(90)))
                .setReversed(false)
                //.setTangent(Math.toRadians(angle))
                //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))

                .splineToLinearHeading(new Pose2d(15, 30.7, Math.toRadians(107)), Math.toRadians(106),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                //.splineToSplineHeading(new Pose2d(15, 30.7, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))


                .stopAndAdd(new SequentialAction(
                        controller.diffyPlace(),
                        new SleepAction(0.1),
                        controller.openClaw(),
                        controller.resetTimer()
                ))
                .build();

        Action human1 = drive.actionBuilder(new Pose2d(15,30.7, Math.toRadians(107)))
                .setReversed(true)
                //.setTangent(Math.toRadians(angle-180))
                //.splineToLinearHeading(new Pose2d(36-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToLinearHeading(new Pose2d(36, 10 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                .splineToSplineHeading(new Pose2d(36, 8 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 95))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                //.waitSeconds(0.05)

                .build();

        Action sub2 = drive.actionBuilder(new Pose2d(36,8, Math.toRadians(90)))
                .setReversed(false)
                //.setTangent(Math.toRadians(angle))
                //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))

                .splineToLinearHeading(new Pose2d(13, 30.7, Math.toRadians(107)), Math.toRadians(106),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                //.splineToSplineHeading(new Pose2d(15, 30.7, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))


                .stopAndAdd(new SequentialAction(
                        controller.diffyPlace(),
                        new SleepAction(0.1),
                        controller.openClaw(),
                        controller.resetTimer()
                ))
                .build();

        Action human2 = drive.actionBuilder(new Pose2d(13,30.7, Math.toRadians(107)))
                .setReversed(true)
                //.setTangent(Math.toRadians(angle-180))
                //.splineToLinearHeading(new Pose2d(36-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToLinearHeading(new Pose2d(36, 10 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                .splineToSplineHeading(new Pose2d(36, 8 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 95))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                //.waitSeconds(0.05)

                .build();

        Action sub3 = drive.actionBuilder(new Pose2d(36,8, Math.toRadians(90)))
                .setReversed(false)
                //.setTangent(Math.toRadians(angle))
                //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))

                .splineToLinearHeading(new Pose2d(11, 30.7, Math.toRadians(107)), Math.toRadians(106),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                //.splineToSplineHeading(new Pose2d(15, 30.7, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))


                .stopAndAdd(new SequentialAction(
                        controller.diffyPlace(),
                        new SleepAction(0.1),
                        controller.openClaw(),
                        controller.resetTimer()
                ))
                .build();

        Action human3 = drive.actionBuilder(new Pose2d(11,30.7, Math.toRadians(107)))
                .setReversed(true)
                //.setTangent(Math.toRadians(angle-180))
                //.splineToLinearHeading(new Pose2d(36-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToLinearHeading(new Pose2d(36, 10 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                .splineToSplineHeading(new Pose2d(36, 8 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 95))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                //.waitSeconds(0.05)

                .build();
        Action sub4 = drive.actionBuilder(new Pose2d(36,8, Math.toRadians(90)))
                .setReversed(false)
                //.setTangent(Math.toRadians(angle))
                //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))

                .splineToLinearHeading(new Pose2d(9, 30.7, Math.toRadians(107)), Math.toRadians(106),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                //.splineToSplineHeading(new Pose2d(15, 30.7, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))


                .stopAndAdd(new SequentialAction(
                        controller.diffyPlace(),
                        new SleepAction(0.1),
                        controller.openClaw(),
                        controller.resetTimer()
                ))
                .build();

        Action human4 = drive.actionBuilder(new Pose2d(9,30.7, Math.toRadians(107)))
                .setReversed(true)
                //.setTangent(Math.toRadians(angle-180))
                //.splineToLinearHeading(new Pose2d(36-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .splineToLinearHeading(new Pose2d(36, 10 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                .splineToSplineHeading(new Pose2d(36, 8 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 95))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.liftClearInstant()
                ))
                //.waitSeconds(0.05)

                .build();
        Action sub5 = drive.actionBuilder(new Pose2d(36,8, Math.toRadians(90)))
                .setReversed(false)
                //.setTangent(Math.toRadians(angle))
                //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))

                .splineToLinearHeading(new Pose2d(8, 30.7, Math.toRadians(107)), Math.toRadians(106),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))
                //.splineToSplineHeading(new Pose2d(15, 30.7, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 95))


                .stopAndAdd(new SequentialAction(
                        controller.diffyPlace(),
                        new SleepAction(0.1),
                        controller.openClaw(),
                        controller.resetTimer()
                ))
                .build();

        Action human5 = drive.actionBuilder(new Pose2d(8,30.7, Math.toRadians(107)))
                .afterTime(0.5, controller.extendoPrepareInstant())
                .afterDisp(Math.sqrt((Math.pow(16-17, 2)+Math.pow(30.7-7, 2)))-3, new SequentialAction(
                        controller.resetTimer(),
                        controller.intakeingSample()
                ))
                .strafeToLinearHeading(new Vector2d(13, 7), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 90))
                .stopAndAdd(new SequentialAction(
                        controller.resetTimer(),
                        controller.intakeingSample()
                ))
                .build();
        Action sample = drive.actionBuilder(new Pose2d(13,7, Math.toRadians(0)))
                .setReversed(true)
                .splineTo(new Vector2d(-39, 8), Math.toRadians(180),new TranslationalVelConstraint(70 ), new ProfileAccelConstraint(-45, 95))
                .afterDisp(14, controller.openClaw())
                .splineTo(new Vector2d(-54, 6), Math.toRadians(220),new TranslationalVelConstraint(70 ), new ProfileAccelConstraint(-45, 95))

                .build();
        Action park = drive.actionBuilder(new Pose2d(8,30.7, Math.toRadians(107)))
                .setReversed(false)
                .afterTime(0.5, controller.robotReset())
                .strafeToLinearHeading(new Vector2d(46, 8), Math.toRadians(0),new TranslationalVelConstraint(70 ), new ProfileAccelConstraint(-80, 90))
                .waitSeconds(10)
                .build();










        //.stopAndAdd()




        // actions that need to happen on init; for instance, a claw tightening.


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
            telemetry.addLine("USE DPAD TO ADJUST INTAKE POS BY INCHES");
            if (intakeposup.getState()){
                blocky +=1;
            }else if (intakeposdown.getState()&& blocky >0){
                blocky -=1;
            }else if (intakeposleft.getState()&& blockx >-6){
                blockx -=0.5;
            }else if (intakeposright.getState()&& blockx <6){
                blockx +=0.5;
            }

            telemetry.addLine("CURR INTAKE POS: "+ (blockx) + ", " + blocky);


            telemetry.update();


        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        robot.mIntake.setExtendoPos(0, timer.seconds());

        blocky-=2;
        Action preload;
        if (blockx-initialPose.position.x<-3) {
            preload = drive.actionBuilder(initialPose)
                    .stopAndAdd(controller.specPlace())
                    .afterTime(0.75, controller.subPrepareInstant())
                    .setTangent(Math.toRadians(135))
                    .afterDisp(Math.sqrt(Math.pow(blockx-initialPose.position.x,2)+Math.pow(41.5-initialPose.position.y,2)),
                            new SequentialAction(
                                    controller.resetTimer(),
                                    controller.pull()
                            ))
                    .splineToLinearHeading(new Pose2d(blockx-1, 24, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .splineToLinearHeading(new Pose2d(blockx, 41.5, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .stopAndAdd(new SequentialAction(
                            controller.openClaw(),
                            controller.diffyPlace(),
                            controller.resetTimer()
                    ))
                    .build();
        } else if (blockx-initialPose.position.x>3) {
            preload = drive.actionBuilder(initialPose)
                    .stopAndAdd(controller.specPlace())
                    .afterTime(0.75, controller.subPrepareInstant())
                    .setTangent(Math.toRadians(45))
                    .afterDisp(Math.sqrt(Math.pow(blockx-initialPose.position.x,2)+Math.pow(41.5-initialPose.position.y,2)),
                            new SequentialAction(
                                    controller.resetTimer(),
                                    controller.pull()
                            ))
                    .splineToLinearHeading(new Pose2d(blockx, 24, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .splineToLinearHeading(new Pose2d(blockx, 41.5, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .stopAndAdd(new SequentialAction(
                            controller.openClaw(),
                            controller.diffyPlace(),
                            controller.resetTimer()
                    ))
                    .build();
        }else {
            preload = drive.actionBuilder(initialPose)
                    .stopAndAdd(controller.specPlace())
                    .afterTime(0.75, controller.subPrepareInstant())
                    .afterDisp(Math.sqrt(Math.pow(blockx-initialPose.position.x,2)+Math.pow(41.5-initialPose.position.y,2)),
                            new SequentialAction(
                                    controller.resetTimer(),
                                    controller.pull()
                            ))
                    .splineToLinearHeading(new Pose2d(blockx, 24, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .splineToLinearHeading(new Pose2d(blockx, 41.5, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-30, 85))
                    .stopAndAdd(new SequentialAction(
                            controller.openClaw(),
                            controller.diffyPlace(),
                            controller.resetTimer()
                    ))
                    .build();
        }
        Action shoot1 = drive.actionBuilder(new Pose2d(blockx, 41.5, Math.toRadians(90)))
                .setReversed(true)
                .afterTime(0.5,new SequentialAction(
                        controller.liftUpInstant(),
                        controller.pivotShoot()
                ))
                .afterDisp(62, new SequentialAction(
                        controller.resetTimer(),
                        controller.shoot()
                ))
                .splineToLinearHeading(new Pose2d(57.5,17, Math.toRadians(90)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))

                .build();
        Action intake1 = drive.actionBuilder(new Pose2d(57.5, 17, Math.toRadians(90)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(58,22), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .build();

        Action shoot2 = drive.actionBuilder(new Pose2d(58, 22, Math.toRadians(90)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(61,17), Math.toRadians(76),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .build();
        Action intake2 = drive.actionBuilder(new Pose2d(61, 17, Math.toRadians(76)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(63,23), Math.toRadians(76),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .build();
        Action shoot3 = drive.actionBuilder(new Pose2d(63, 23, Math.toRadians(76)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(48,17), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .build();
        Action intake3 = drive.actionBuilder(new Pose2d(48, 17, Math.toRadians(90)))
                .setReversed(false)
                .stopAndAdd(controller.pivotDown())

                .strafeToLinearHeading(new Vector2d(48,23), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .build();
        Action wall = drive.actionBuilder(new Pose2d(48, 23, Math.toRadians(90)))
                .setReversed(false)
                .stopAndAdd(controller.extendoInInstant())
                .strafeToLinearHeading(new Vector2d(45,7), Math.toRadians(90),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 95))
                .stopAndAdd(new SequentialAction(
                        controller.closeClaw(),
                        controller.pivotShootLow()
                ))
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(
                        controller.pivotShootLow(),
                        controller.resetTimer(),
                        controller.liftClearInstant(),
                        controller.pivotShootLow()

                ))
                //.waitSeconds(0.05)
                .build();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        controller.updateRobot(),
                        new SequentialAction(
                                preload,
                                controller.pull(),
                                shoot1,
                                controller.resetTimer(),
                                new ParallelAction(
                                        intake1,

                                        controller.intakeing()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        shoot2,
                                        controller.shoot()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        intake2,
                                        controller.intakeing()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        shoot3,
                                        controller.shoot()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        intake3,
                                        controller.intakeing()
                                ),
                                controller.resetTimer(),

                                wall,
                                controller.resetTimer(),
                                new ParallelAction(
                                        sub1,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human1,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub2,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human2,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub3,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human3,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub4,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        human4,
                                        controller.intakeReset()
                                ),
                                controller.resetTimer(),

                                new ParallelAction(
                                        sub5,
                                        controller.clearWall()
                                ),
                                controller.resetTimer(),
                                park

                        )

                )
        );
        robot.stop();
    }
}