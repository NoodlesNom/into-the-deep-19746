package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.autonomous.gf.OldAutoMaster.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "BSTEM🗣️️🔥💯TELEOP", group = "opMode")
public class BSTEMTELE extends OpMode {
    private List<Action> runningActions = new ArrayList<>();

    private Robot robot;
    public static int rejectingTime = 500;
    private Deadline rejecting = new Deadline(rejectingTime, TimeUnit.MILLISECONDS);

    private boolean reject = true;
    private boolean autoSpec=false;
    private int angle = 108;
    private boolean ResetAutoSpec=false;
    public List<LynxModule> allHubs;
    public static double firstwait = 0.3;

    private FtcDashboard dash = FtcDashboard.getInstance();

    public static int specplacingpivottime =800;
    public class robotController {


        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawToggleHits=0;
                return false;
            }
        }

        public Action openClaw() {
            return new robotController.OpenClaw();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawToggleHits=1;
                return false;
            }
        }

        public Action closeClaw() {
            return new robotController.CloseClaw();
        }

        public class PivotDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                return false;
            }
        }

        public Action pivotDown() {
            return new robotController.PivotDown();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal());
                return false;
            }
        }
        public Action pivotUp() {
            return new robotController.PivotUp();
        }

        public class DiffyIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(80,-90);
                return false;
            }
        }
        public Action diffyIntake() {
            return new robotController.DiffyIntake();
        }

        public class DiffyPlace implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setDiffyPos(20,70);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECSLAM.getVal());
                robot.mLift.setTargetPos(2, timer.seconds());
                BotLog.logD("BSDbg", "END OF PLACING TRAJECTORY AHHHHHHHHHHH");
                return false;
            }
        }
        public Action diffyPlace() {
            return new robotController.DiffyPlace();
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
            return new robotController.DiffyPlaceLast();
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
            return new robotController.intakePrepareInstant();
        }
        public class DiffyRelease implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                return false;
            }
        }
        public Action diffyRelease() {
            return new robotController.DiffyRelease();
        }

        public class LiftDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftDown() {
            return new robotController.LiftDown();
        }

        public class LiftIntakeInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftIntakeInstant() {
            return new robotController.LiftIntakeInstant();
        }


        public class LiftClearInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSPECANGLED.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action liftClearInstant() {
            return new robotController.LiftClearInstant();
        }

        public class RobotReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                robot.mIntake.setExtendoOpenLoop(-0.4);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                robot.mDeposit.setDiffyPos(0,0);

                return false;
            }
        }

        public Action robotReset() {
            return new robotController.RobotReset();
        }

        public class Intakeing implements Action {

            private boolean intaken = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setOutputLimits(-1,0.8);
                if (robot.mLift.closeEnough()||robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()){
                    if (robot.mDeposit.servoDone()||robot.mDeposit.getPivotPos() == Deposit.PIVOT_POS.TRANSFER.getVal()){
                        if (generaltimer.seconds()>1||(intaken&&generaltimer.seconds()>0.1)){
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
                    if (((robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED")))&&!intaken){
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
            return new robotController.Intakeing();
        }

        public class LiftPlaceInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(1, timer.seconds());
                return false;
            }
        }

        public Action liftPlaceInstant() {
            return new robotController.LiftPlaceInstant();
        }


        public class ClearWall implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (generaltimer.seconds()>1){
                    generaltimer.reset();
                }else if (generaltimer.seconds()>0.3){
                    robot.mDeposit.setDiffyPos(-20, 70);
                    //robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSPECANGLED.getVal(), timer.seconds());
                    return false;
                }else if (generaltimer.seconds()>0.1) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.AUTOSPECANGLED.getVal(), 800 , new double[] {1,2,2,2,2,2,2,1,1,1});
                    robot.mIntake.setIntakeOpenLoop(0);
                }else{
                    robot.mLift.setTargetPos(Lift.LIFT_POS.AUTOSPECANGLED.getVal(), timer.seconds());
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                }
                return true;
            }
        }

        public Action clearWall() {
            return new robotController.ClearWall();
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
                return true;
            }
        }

        public Action intakeReset() {
            return new robotController.IntakeReset();
        }


        public class LiftUpInstant implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(2, timer.seconds());
                return false;
            }
        }

        public Action liftUpInstant() {
            return new robotController.LiftUpInstant();
        }

        public class SpecPlace implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                robot.mDeposit.setDiffyPos(-40, 67);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal(), 600, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                return false;
            }
        }

        public Action specPlace() {
            return new robotController.SpecPlace();
        }

        public class PivotShoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action pivotShoot() {
            return new robotController.PivotShoot();
        }

        public class ExtendoPrepareInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.AUTOINTAKEPREPARE.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                return false;
            }
        }

        public Action extendoPrepareInstant() {
            return new robotController.ExtendoPrepareInstant();
        }
        public class ExtendoInInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setClawPos(0);
                robot.mDeposit.setClawPos(0);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());
                robot.mDeposit.setDiffyPos(80,-90);
                robot.mIntake.setExtendoPos(0, timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                return false;
            }
        }

        public Action extendoInInstant() {
            return new robotController.ExtendoInInstant();
        }


        public class Shoot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                if (!robot.mIntake.closeEnoughAuto()){
                    generaltimer.reset();
                }
                if (generaltimer.seconds()>0.4) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);

                    return false;
                }else  if (generaltimer.seconds()>0.1) {
                    robot.mIntake.setIntakeOpenLoop(-0.9);
                }else if   (robot.mIntake.closeEnoughAuto()){

                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setIntakeOpenLoop(0);
                }else {
                    robot.mIntake.setClawPos(1);
                    robot.mIntake.setIntakeOpenLoop(-0.8);
                }

                return true;
            }
        }

        public Action shoot() {
            return new robotController.Shoot();
        }
        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                generaltimer.reset();
                return false;
            }
        }

        public Action resetTimer() {
            return new robotController.ResetTimer();
        }
        public class ShootLast implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                if (!robot.mIntake.closeEnoughAuto()){
                    generaltimer.reset();
                }
                if (generaltimer.seconds()>0.3) {
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);

                    return false;
                }else  if   (robot.mIntake.closeEnoughAuto()){
                    robot.mIntake.setIntakeOpenLoop(-0.7);
                    robot.mIntake.setClawPos(0);
                }

                return true;
            }
        }

        public Action shootLast() {
            return new robotController.ShootLast();
        }

        public class LiftDownInstant implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return false;
            }
        }

        public Action liftDownInstant() {
            return new robotController.LiftDownInstant();
        }
    }
    robotController controller;


    public static double[] specplacingprofile = new double[]{1,2,2,2,2,2,2,1,1,1};
    public static double intakewait = 0.15;
    public static double secondwait = 0.07;
    public static int intakepivaottime = 250;
    public static int transferextendopos = 140;
    private boolean stalledintaking = false;

    private ElapsedTime timer;

    private boolean autoclear = false;
    private ElapsedTime defenseTimer = new ElapsedTime();
    private ElapsedTime straightenTimer = new ElapsedTime();

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;
    private double  outputRate = 0.25;
    private boolean liftRezeroing = false;
    private boolean intakeRezeroing = false;
    private Deadline logging = new Deadline((long)(outputRate*1000), TimeUnit.MILLISECONDS);
    private double loopCnt = 0;

    private boolean specangled = false;
    private double  lastOutput = 0.0;
    private boolean enableTelem = false;

    private boolean specplacing = false;

    private boolean hangReady = false;
    private boolean hanging = false;

    private boolean samplesafe = false;
    private boolean lowsample = false;


    // private double intakeSpeed = 1425;

    private boolean straighten = false;

    private boolean samplemode = false;

    private StickyButton angledbutton = new StickyButton();
    private StickyButton autoSpecButton = new StickyButton();
    private StickyButton mode = new StickyButton();

    private StickyButton intakeStepsButton = new StickyButton();

    private StickyButton safemode = new StickyButton();
    private StickyButton lowmode = new StickyButton();

    private StickyButton rejectToggle = new StickyButton();
    private StickyButton clawButton = new StickyButton();


    private double prevForwardPwr = 0;
    private int intakeStateHits = 0;
    private int clawToggleHits = 2;
    private ElapsedTime spectimer = new ElapsedTime();
    private ElapsedTime hangtimer = new ElapsedTime();

    private ElapsedTime transfertimer = new ElapsedTime();
    private ElapsedTime rezero1 = new ElapsedTime();
    private ElapsedTime rezero2 = new ElapsedTime();
    private ElapsedTime generaltimer = new ElapsedTime();
    private ElapsedTime sampletimer = new ElapsedTime();

    private ElapsedTime shoottimer = new ElapsedTime();
    private int pivotToggleHits = 2;
    private boolean firstTeleopLoop = false;
    private boolean enableCurrentReporting = true;
    private boolean firstStateLoop = true;

    private double offset = 0;

    private boolean transferready = false;

    // private Deadline imuTimer = new Deadline(500, TimeUnit.MILLISECONDS);

    private enum teleState{
        IDLE,
        SPECINTAKE,
        SPECPLACE,
        INTAKING,
        TRANSFER,
        SAMPLE,
        SHOOT,
        HANG,
        NOTHING,
        REZERO,
        SAFTEY,
        AUTOSPECPLACE,
        AUTOSPECINTAKE
    }

    private teleState teleFSM = teleState.IDLE;
    private teleState prevtelestate = teleState.IDLE;
    Action sub;
    Action human;

    public void init() {
        telemetry.addLine("Creating robot");
        telemetry.update();
        robot = new Robot(this);
        if(!firstTeleopLoop){
            controller = new robotController();
            sub = robot.mDrive.drive.actionBuilder(new Pose2d(39,6.5, Math.toRadians(90)))
                    .setReversed(false)
                    //.setTangent(Math.toRadians(angle))
                    //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                    .splineToLinearHeading(new Pose2d(16, 29.75, Math.toRadians(angle-10)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 85))
                    .splineToSplineHeading(new Pose2d(15, 30.25, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 85))


                    .stopAndAdd(new SequentialAction(
                            controller.diffyPlace(),
                            new SleepAction(0.2),
                            controller.openClaw(),
                            controller.resetTimer()
                    ))
                    .build();
            human = robot.mDrive.drive.actionBuilder(new Pose2d(15, 30.25, Math.toRadians(angle)))
                    .setReversed(true)
                    //.setTangent(Math.toRadians(angle-180))
                    //.splineToLinearHeading(new Pose2d(39-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                    .splineToLinearHeading(new Pose2d(39, 10 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                    .splineToSplineHeading(new Pose2d(39, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 85))

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

        }
        firstTeleopLoop = true;
        //PhotonCore.start(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        // Let's start with bulk caching off until we get through all the init.
        for (LynxModule hub : allHubs)
        {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }





        telemetry.addLine("Initing robot");
        telemetry.update();
        robot.teleopInit();





        timer = new ElapsedTime();
        timer.reset();

        defenseTimer = new ElapsedTime();
        defenseTimer.reset();

        telemetry.addLine("Init finished");
        telemetry.update();
        robot.mDeposit.setLiveLed(team);


        // We're through init, lets go back to bulk caching and clear caches
        for (LynxModule hub : allHubs)
        {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }
    }

    public void loop()
    {
        clawButton.update(gamepad1.a||gamepad2.b);
        rejectToggle.update(gamepad1.start);
        autoSpecButton.update(gamepad1.dpad_right);

        if (rejectToggle.getState()){
            reject = !reject;
        }

        double startTime = 0.0;
        if (debugLoopTime)
        {
            startTime = loopTimer.milliseconds();
        }

        // Lets clear caches at the top of the loop to get fresh data to work with
        for (LynxModule hub : allHubs)
        {
            hub.clearBulkCache();
        }

        if (firstTeleopLoop)
        {
            // Don't power the hang during the init cycle to avoid movement
            robot.mIntake.setExtendoPos(0, timer.seconds());
            // Set the target to 1.1 then back to 1.0 to make sure we latch it in
            firstTeleopLoop = false;

            if(enableCurrentReporting) {
                robot.setEnableCurrentReporting(true);
            }
            loopTimer.reset();
            startTime = loopTimer.milliseconds();
            logging.reset();
        }

        switch (teleFSM){
            case IDLE:{
                robot.mDeposit.setLed(team);
                if (prevtelestate == teleState.SAMPLE) {
                    clawToggleHits = 0;
                    //if last state was sample wait before coming back down
                    if (sampletimer.seconds() > 0.6) {
                        if (samplemode) {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                            robot.mDeposit.setDiffyPos(30, -90);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        } else {
                            robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                            robot.mDeposit.setDiffyPos(0, 0);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                        }
                        robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    }
                }else if (samplemode) {
                    //if sample mode go to transfer pos
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                    robot.mDeposit.setDiffyPos(30, -90);
                    clawToggleHits = 0;
                }else{
                    clawToggleHits = 1;
                }
                //open claw; 1 = close, 2 = open

                if (prevtelestate!= teleState.INTAKING) {
                    //dont reset the bot if last state was intaking
                    if (prevtelestate != teleState.SAMPLE) {
                        //if idle+samplemode, prep for transfer
                        if (samplemode) {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mDeposit.setDiffyPos(30, -90);
                        } else if (robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal() || robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.SPECANGLED.getVal()) {
                            //leave transfer pos
                            if (robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.TRANSFERPREP.getVal()) {
                                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECANGLED.getVal(), timer.seconds());
                            }
                            if (robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.SPECANGLED.getVal() && robot.mLift.closeEnough()) {
                                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                            }
                            if (robot.mLift.getLiftTicks() > 240) {
                                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                            }
                        } else {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                            robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                        }
                    }


                    if (prevtelestate == teleState.TRANSFER) {
                        //wait for lift to leave before raising pivot and diffy
                        if (transfertimer.seconds() > 0.3) {
                            robot.mDeposit.setDiffyPos(0, 0);
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setExtendoPos(0, timer.seconds());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());

                        }
                        //idk if this is redundant
                        //it works so im not removing it
                        //bite me
                    } else if (prevtelestate == teleState.SAMPLE) {
                        if (sampletimer.seconds() > 0.6) {
                            if (samplemode) {
                                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[]{1, 2, 3, 4, 4, 4, 3, 2, 1, 1});
                                robot.mDeposit.setDiffyPos(30, -90);
                                robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                                robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                            } else {
                                robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                                robot.mDeposit.setDiffyPos(0, 0);
                                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                            }
                            robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                            robot.mIntake.setIntakeOpenLoop(0);
                            robot.mIntake.setExtendoPos(0, timer.seconds());
                        }
                    } else {
                        //reset intake
                        robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        if (samplemode) {
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        } else {
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                        }


                    }
                }
                break;

            }
            case SPECINTAKE:{
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                robot.mDeposit.setDiffyPos(70,-90);
                if (samplemode&&(prevtelestate== teleState.IDLE||prevtelestate== teleState.TRANSFER)){
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), 1200, specplacingprofile);
                }else {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal(), specplacingpivottime, specplacingprofile);
                }
                samplemode = false;
                if (gamepad1.a){
                    clawToggleHits = 1;
                    autoclear = true;
                }
                if (autoclear){
                    if (spectimer.seconds()>0.15){
                        robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        gamepad1.rumble(200);
                        gamepad2.rumble(200);
                        shoottimer.reset();
                        autoclear = false;
                        specangled = false;
                        prevtelestate = teleFSM;
                        teleFSM = teleState.SPECPLACE;
                        specplacing = true;
                        spectimer.reset();

                        robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
                    }
                }else {
                    spectimer.reset();
                }

                break;
            }
            case SPECPLACE:{
                samplemode = false;
                if (shoottimer.seconds()>0.8){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                }else if (shoottimer.seconds()>0.5){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(-0.9);

                }else{
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                    robot.mIntake.setClawPos(0);
                }
                if (spectimer.seconds()>0.5){
                    if (specangled) {
                        robot.mDeposit.setDiffyPos(90, 0);
                        robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());

                    }else{
                        if (specplacing) {
                            robot.mDeposit.setDiffyPos(-40, 70);
                        }
                        robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                    }

                }
                if (spectimer.seconds()>0.4) {
                    if (specangled) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECPUSH.getVal());
                    }else{
                        if (specplacing) {
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal(), specplacingpivottime , specplacingprofile);

                        }else{
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECANGLED.getVal());
                        }
                    }

                }
                break;

            }
            case INTAKING:{

                if (robot.mIntake.detectedYellow()||(robot.mIntake.detectedBlue()&&team.name().equals("BLUE"))||(robot.mIntake.detectedRed()&&team.name().equals("RED"))){
                    gamepad1.rumble(100);
                }
                if (samplemode) {
                    clawToggleHits=2;
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[] {2,3,3,3,3,3,3,2,1,1});

                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                    robot.mDeposit.setDiffyPos(30, -90);
                }
                if (gamepad1.right_trigger>0.2&& robot.mIntake.getExtendoPosition()<615) {
                    robot.mIntake.setExtendoOpenLoop(gamepad1.right_trigger*0.7);
                }else if (gamepad1.left_trigger>0.2&& robot.mIntake.getExtendoPosition()>20){
                    robot.mIntake.setExtendoOpenLoop(-gamepad1.left_trigger*0.7);
                }else {
                    if (robot.mIntake.getExtendoPosition()>615){
                        robot.mIntake.setExtendoOpenLoop(-0.3);
                    }else {
                        robot.mIntake.setExtendoOpenLoop(0);
                    }
                }
                if (gamepad1.right_bumper){
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setIntakeOpenLoop(1);

                }else if(gamepad1.left_bumper){
                    robot.mIntake.setClawPos(0);
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    stalledintaking = false;
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    robot.mIntake.setIntakeOpenLoop(-0.6);
                }else if(gamepad1.y){
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    stalledintaking = false;
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.BLOCKCLEAR.getVal());
                    robot.mIntake.setIntakeOpenLoop(0.7);
                }else {
                    stalledintaking = false;
                    robot.mIntake.setIntakeOpenLoop(0);
                }
                if (gamepad1.x){
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                }
                if (gamepad1.b){
                    straightenTimer.reset();
                    straighten = true;
                    robot.mIntake.setClawPos(1);
                    stalledintaking = false;
                    robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
                    robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
                    if (samplemode){
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                    }else{
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }
                    robot.mIntake.setIntakeOpenLoop(0);
                    prevtelestate = teleFSM;
                    teleFSM = teleState.IDLE;
                }

                if((robot.mIntake.detectedBlue()&&team.name().equals("RED"))||(robot.mIntake.detectedRed()&&team.name().equals("BLUE"))){
                    rejecting.reset();
                }
                if (!rejecting.hasExpired()&&reject){
                    robot.mIntake.setIntakeOpenLoop(-0.6);
                }


                break;
            }
            case TRANSFER:{
                robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                robot.mDeposit.setDiffyPos(30, -90);
                if (robot.mIntake.closeEnoughAuto()&&robot.mLift.closeEnough()){
                    if (transfertimer.seconds()>0.4){
                        robot.mLift.setTargetPos(Lift.LIFT_POS.SAMPLE.getVal(), timer.seconds());
                    }else if (transfertimer.seconds()>0.3){
                        robot.mIntake.setIntakeOpenLoop(-0.8);
                    }
                    else if (transfertimer.seconds()>0.2){
                        robot.mIntake.setClawPos(0);
                    }else if (transfertimer.seconds()>0.1){
                        clawToggleHits = 1;
                        robot.mIntake.setIntakeOpenLoop(0);
                    }
                    else{
                        robot.mIntake.setExtendoOpenLoop(-0.75);
//                        robot.mIntake.setIntakeOpenLoop(-1);
                        robot.mIntake.setClawPos(1);
                    }
                }else if (robot.mLift.getLiftTargetPos() != Lift.LIFT_POS.SAMPLE.getVal()){
                    transfertimer.reset();
                }
                if (transfertimer.seconds()>0.7&&robot.mLift.getLiftTargetPos() == Lift.LIFT_POS.SAMPLE.getVal()){
                    //robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0,timer.seconds());
                    prevtelestate = teleFSM;
                    teleFSM = teleState.SAMPLE;
                    transfertimer.reset();
                }
                break;
            }
            case SHOOT:{
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                if (shoottimer.seconds()>1){
                    robot.mLift.setTargetPos(0, timer.seconds());
                    prevtelestate = teleFSM;
                    teleFSM = teleState.IDLE;
                }else if (shoottimer.seconds()>0.9){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                }else if (shoottimer.seconds()>0.6){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(-0.9);

                }else if (shoottimer.seconds()>0.3){
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());
                    robot.mIntake.setClawPos(0);

                }
                break;
            }
            case REZERO:{
                gamepad1.rumble(100);
                gamepad2.rumble(100);
                robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                if (liftRezeroing) {
                    if (rezero1.seconds() > 0.7) {
                        robot.mLift.zerofinish(timer.seconds());
                        robot.mLift.setTargetPos(0,timer.seconds());
                    } else if (rezero1.seconds() > 0.2) {
                        robot.mLift.rezero();
                    } else {
                        robot.mLift.setOpenLoop(-0.7);
                    }
                }

                if (intakeRezeroing) {
                    if (rezero2.seconds() > 0.7) {
                        robot.mIntake.zerofinish(timer.seconds());
                        robot.mIntake.setExtendoPos(0,timer.seconds());
                    } else if (rezero2.seconds() > 0.2) {
                        robot.mIntake.rezero();
                    } else {
                        robot.mIntake.setExtendoOpenLoop(-0.5);
                    }
                }
                if (rezero1.seconds()>0.7&& rezero2.seconds()>0.7) {
                    prevtelestate = teleFSM;
                    teleFSM = teleState.IDLE;
                    intakeRezeroing = false;
                    liftRezeroing = false;
                }
                break;
            }
            case AUTOSPECPLACE:{
                robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                if (runningActions.isEmpty()){
                    runningActions.add(
                            new SequentialAction(
                                    controller.resetTimer(),
                                    new ParallelAction(
                                            sub,
                                            controller.clearWall()
                                    )
                            ));
                    human = robot.mDrive.drive.actionBuilder(new Pose2d(15, 30.25+offset, Math.toRadians(angle)))
                            .setReversed(true)
                            //.setTangent(Math.toRadians(angle-180))
                           // .splineToLinearHeading(new Pose2d(39+offset-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                            .splineToLinearHeading(new Pose2d(39, 10+offset ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                            .splineToSplineHeading(new Pose2d(39, 6.5+offset ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 85))

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
                    prevtelestate=teleFSM;
                    teleFSM = teleState.AUTOSPECINTAKE;
                    offset-=0.21;
                }
                break;
            }
            case AUTOSPECINTAKE:{
                robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                if (runningActions.isEmpty()){
                    runningActions.add(
                            new SequentialAction(
                                    controller.resetTimer(),
                                    new ParallelAction(
                                            human,
                                            controller.intakeReset()
                                    )
                            ));
                    sub = robot.mDrive.drive.actionBuilder(new Pose2d(39,6.5+offset, Math.toRadians(90)))
                            .setReversed(false)
                            //.setTangent(Math.toRadians(angle))
                           // .splineToLinearHeading(new Pose2d(16+offset+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                            .splineToLinearHeading(new Pose2d(16, 29.75+offset, Math.toRadians(angle-10)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 85))
                            .splineToSplineHeading(new Pose2d(15, 30.25+offset, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 85))


                            .stopAndAdd(new SequentialAction(
                                    controller.diffyPlace(),
                                    new SleepAction(0.2),
                                    controller.openClaw(),
                                    controller.resetTimer()
                            ))
                            .build();
                    prevtelestate=teleFSM;
                    teleFSM = teleState.AUTOSPECPLACE;
                }
                break;
            }
            case SAFTEY:{
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECANGLED.getVal());
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(),timer.seconds());
                break;
            }
            case SAMPLE:{
                if (transfertimer.seconds()>0.3){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0, timer.seconds());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());

                }
                if (lowsample){
                    robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SAMPLE.getVal(), 500, new double[] {2,3,3,3,3,3,3,2,1,1});
                    robot.mDeposit.setDiffyPos(-60, -90);
                }else if (samplesafe){
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SAMPLESAFE.getVal(), timer.seconds());
                    if (robot.mLift.getLiftTicks()>500) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SAMPLE.getVal(), 500, new double[] {2,3,3,3,3,3,3,2,1,1});

                        if (robot.mLift.closeEnough()){
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }else{
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

                        }
                        robot.mDeposit.setDiffyPos(-60, -90);
                    }else{
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal(), 600, new double[] {2,3,3,3,3,3,3,2,1,1});

                        robot.mDeposit.setDiffyPos(0, -90);
                    }
                }else {
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SAMPLE.getVal(), timer.seconds());
                    if (robot.mLift.getLiftTicks()>700) {
                        if (robot.mLift.closeEnough()){
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }else{
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                        }
                    }
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SAMPLE.getVal(), 700, new double[] {2,3,3,3,3,3,3,2,1,1});

                    robot.mDeposit.setDiffyPos(-60, -90);

                }


                if (gamepad1.a){
                    prevtelestate = teleFSM;
                    teleFSM = teleState.IDLE;
                    sampletimer.reset();
                    gamepad1.rumble(200);
                    gamepad2.rumble(200);
                }
                break;
            }
            case HANG:{

                robot.mDeposit.setDiffyPos(130,0);
                clawToggleHits = 1;
                robot.mIntake.setIntakeOpenLoop(0);
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.HANG.getVal(), 1200, new double[] {2,3,3,3,3,3,3,2,1,1});
                if (!hangReady) {
                    if (robot.mIntake.closeEnough()) {
                        if(hangtimer.seconds()>0.5) {
                            robot.mLift.setTargetPos(Lift.LIFT_POS.HANG.getVal(), timer.seconds());
                            hangReady = true;
                        }
                    } else {
                        hangtimer.reset();
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                    }
                }else {
                    if (gamepad1.dpad_up) {
                        robot.mDrive.setWinchPos(1);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                    } else if (gamepad1.dpad_down) {
                        robot.mDrive.setWinchPos(2);
                        hanging=true;

                    }
                    if (!hanging){
                        hangtimer.reset();
                    }
                    if (hanging&&hangtimer.seconds()>2.5){
                        robot.mIntake.setExtendoTicks(300, timer.seconds());
                    }
                }

                //-110 diffy pitch
                break;
            }

        }
        mode.update(gamepad2.left_bumper);
        if (mode.getState()){
            samplemode = !samplemode;
        }

        if (gamepad2.b && !specplacing&&clawButton.getState()){
            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            prevtelestate = teleFSM;
            teleFSM = teleState.SPECINTAKE;
            autoclear = false;
            clawToggleHits = 0;
        }
        if (gamepad2.y){
            shoottimer.reset();
            specangled = false;
            prevtelestate = teleFSM;
            teleFSM = teleState.SPECPLACE;
            specplacing = true;
            spectimer.reset();

            robot.mLift.setTargetPos(Lift.LIFT_POS.SPECCLEAR.getVal(), timer.seconds());
        }
        if (gamepad1.right_trigger>0.2&&teleFSM!= teleState.INTAKING){

            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKEPREP.getVal());
            prevtelestate = teleFSM;
            teleFSM = teleState.INTAKING;
        }

        if ((gamepad1.a&&teleFSM== teleState.INTAKING&&samplemode)){
            robot.mIntake.setClawPos(1);
            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 750, new double[] {2,3,3,3,3,3,3,2,1,1});
            prevtelestate = teleFSM;
            teleFSM = teleState.TRANSFER;
            straighten=true;
            straightenTimer.reset();
            transferready = false;
            robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
            generaltimer.reset();
            transfertimer.reset();
            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
            clawToggleHits = 2;

        }
        if (gamepad2.x){
            prevtelestate = teleFSM;
            teleFSM = teleState.IDLE;
        }
        if (gamepad2.touchpad){
            prevtelestate = teleFSM;
            teleFSM = teleState.REZERO;
            liftRezeroing = true;
            rezero1.reset();
        }
        if (gamepad1.touchpad){
            prevtelestate = teleFSM;
            teleFSM = teleState.REZERO;
            intakeRezeroing = true;
            rezero2.reset();
        }
        if (gamepad2.dpad_right){
            prevtelestate = teleFSM;
            teleFSM = teleState.SAFTEY;
        }
        if (gamepad2.a){
            prevtelestate = teleFSM;
            teleFSM = teleState.SHOOT;
            shoottimer.reset();
            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
        }
        if (gamepad2.dpad_up){
            prevtelestate = teleFSM;
            teleFSM = teleState.SAMPLE;
        }
        safemode.update(gamepad2.dpad_left);
        lowmode.update(gamepad2.dpad_down);
        if (safemode.getState()){
            samplesafe = !samplesafe;
        }
        if (lowmode.getState()){
            lowsample = !lowsample;
        }
        angledbutton.update(gamepad2.right_bumper);
        if (angledbutton.getState()){
            specangled = !specangled;
        }
        if (gamepad1.b||(gamepad2.b&&teleFSM!= teleState.INTAKING)){
            stalledintaking = false;
            straightenTimer.reset();
            straighten = true;
            robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
            if (samplemode){
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
            }else{
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
            }
            robot.mIntake.setIntakeOpenLoop(0);
        }

        if (straighten){
            transfertimer.reset();
            robot.mIntake.setClawPos(1);
            if (straightenTimer.seconds()>0.6){
                robot.mIntake.setIntakeOpenLoop(0);
                straighten = false;
            }else if (straightenTimer.seconds()>0.1){
                robot.mIntake.setIntakeOpenLoop(-1);
            }
        }



        // DRIVETRAIN
        double driveInput = -gamepad1.left_stick_y;
        double strafeInput = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;
        if (Math.abs(robot.mLift.mPeriodicIO.demand)+Math.abs(robot.mIntake.getPowerDemandSum())>2.0||teleFSM == teleState.TRANSFER){
            driveInput*=0.7;
            strafeInput*=0.7;
            turnInput*=0.7;
        }

        if (driveInput<prevForwardPwr && teleFSM == teleState.SAMPLE){
            driveInput = prevForwardPwr-Math.min(prevForwardPwr-driveInput, 0.1);
        }
        prevForwardPwr = driveInput;

        if (teleFSM!=teleState.AUTOSPECINTAKE&&teleFSM!=teleState.AUTOSPECPLACE) {
            runningActions.clear();
            if (ResetAutoSpec){
                ResetAutoSpec=false;
                offset=0;
                sub = robot.mDrive.drive.actionBuilder(new Pose2d(39,6.5, Math.toRadians(90)))
                        .setReversed(false)
                        //.setTangent(Math.toRadians(angle))
                        //.splineToLinearHeading(new Pose2d(16+Math.sin(Math.toRadians(angle-90))*10, 38-10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(angle)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))

                        .splineToLinearHeading(new Pose2d(16, 29.75, Math.toRadians(angle-10)), Math.toRadians(angle),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 85))
                        .splineToSplineHeading(new Pose2d(15, 30.25, Math.toRadians(angle)), Math.toRadians(0),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-50, 85))


                        .stopAndAdd(new SequentialAction(
                                controller.diffyPlace(),
                                new SleepAction(0.2),
                                controller.openClaw(),
                                controller.resetTimer()
                        ))
                        .build();
                human = robot.mDrive.drive.actionBuilder(new Pose2d(15, 30.25, Math.toRadians(angle)))
                        .setReversed(true)
                        //.setTangent(Math.toRadians(angle-180))
                        //.splineToLinearHeading(new Pose2d(39-Math.sin(Math.toRadians(angle-90))*10, 6.5+10*Math.cos(Math.toRadians(angle-90)), Math.toRadians(90)), Math.toRadians(angle-180),new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40, 85))
                        .splineToLinearHeading(new Pose2d(39, 10 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45, 85))
                        .splineToSplineHeading(new Pose2d(39, 6.5 ,Math.toRadians(90)), Math.toRadians(270),  new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-45, 85))
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

            }
            robot.mDrive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveInput, strafeInput), turnInput));
        }else{
            if (!gamepad1.atRest()){
                prevtelestate = teleFSM;
                teleFSM = teleState.NOTHING;

            }
            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                if (teleFSM!=teleState.AUTOSPECINTAKE&&teleFSM!=teleState.AUTOSPECPLACE){
                    break;
                }
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }

            runningActions = newActions;
            dash.sendTelemetryPacket(packet);
        }





        // Just in case we getting ready to move to STOWED, shut intake down




        if (clawButton.getState()&&!(teleFSM== teleState.INTAKING&&samplemode)&&teleFSM!= teleState.SPECINTAKE&&teleFSM!= teleState.TRANSFER){
            clawToggleHits++;
            if (specplacing){
                specplacing= false;
                robot.mDeposit.setDiffyPos(0,90);
                robot.mDeposit.setLed(team);
                gamepad2.rumble(200);
            }
        }
        if (teleFSM == teleState.TRANSFER){
            if (clawToggleHits %2==0){
                robot.mDeposit.setClawPos(2);
            }else{
                robot.mDeposit.setClawPos(1);
            }
        }else {
            robot.mDeposit.setClawPos(clawToggleHits % 2);
        }

        if (gamepad2.share){
            robot.mIntake.setExtendoPos(0, timer.seconds());

            hangtimer.reset();
            robot.mDrive.setWinchPos(1);
            prevtelestate = teleFSM;
            teleFSM= teleState.HANG;
        }

        if (autoSpecButton.getState()&&robot.mDeposit.getPivotPos()==3){
            clawToggleHits=1;
            robot.mDrive.drive.pose= new Pose2d(39,6.5,Math.toRadians(90));
            //robot.mDrive.drive.pinpoint.setPosition(new Pose2d(39,6,Math.toRadians(90)));
            //robot.mDrive.drive.pinpoint.setPositionRR(new Pose2d(39,6,Math.toRadians(90)));
            runningActions.add(
                    new SequentialAction(
                            controller.resetTimer(),
                            new ParallelAction(
                                    sub,
                                    controller.clearWall()
                            )
            ));
            ResetAutoSpec=true;
            prevtelestate=teleFSM;
            teleFSM=teleState.AUTOSPECINTAKE;
        }





        // update robot
        robot.update(timer.seconds());

        //BotLog.logD("Lift :: ", String.format("%s", robot.mLift.getTelem(timer.seconds())));
        //BotLog.logD("Hang :: ", String.format("%s", robot.mHang.getTelem(timer.seconds())));

        //if ((timer.seconds() > (lastOutput + outputRate)) && enableTelem)
        //{
        //    BotLog.logD("robot :: ", String.format("%s", robot.getTelem(timer.seconds())));
        //    // BotLog.logD("lift :: ", String.format("%s", robot.mLift.getTelem(timer.seconds())));
        //    lastOutput = timer.seconds();
        //}

        boolean debug = true;
        if(debug) {
            if(logging.hasExpired())
            {
                logging.reset();
                String debugMsg = robot.getTelem(time);
                debugMsg += String.format("loopTime = %4.0f\n", loopTimer.milliseconds());
                debugMsg += String.format("tele State = %s\n", teleFSM.name());
                BotLog.logD("BSDbg",debugMsg);
            }
            loopTimer.reset();
        }

        // This adds a lot of loop time, don't call this lightly
        // if(imuTimer.hasExpired()) {
        //     robot.mDrive.logIMUs();
        //     imuTimer.reset();
        // }

        //telemetry.addLine(robot.getTelem(timer.seconds()));



        /*
        if ((timer.seconds() > (lastOutput + outputRate)))
        {
            //BotLog.logD("Odo", "%s",robot.mDrive.getPose());
            //BotLog.logD("intakeA", "%5.2f", robot.mIntake.getMotorCurrent());
            lastOutput = timer.seconds();
        }
        */

        // Disable in teleop for now

//        if (useAprilTags)
//        {
//            List<AprilTagDetection> currentDetections = robot.mVision.getAprilTags();
//            telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//            // Step through the list of detections and display info for each one.
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                } else {
//                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                }
//            }
//
//            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//            telemetry.addLine("RBE = Range, Bearing & Elevation");
//        }


        if (debugLoopTime)
        {
            BotLog.logD("lt_teleop :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }

//        telemetry.addData("fl current", robot.mDrive.drive.mPeriodicIO.flcurrent);
//        telemetry.addData("fr current", robot.mDrive.drive.mPeriodicIO.frcurrent);
//        telemetry.addData("bl current", robot.mDrive.drive.mPeriodicIO.blcurrent);
//        telemetry.addData("br current", robot.mDrive.drive.mPeriodicIO.brcurrent);
//        telemetry.addData("intake current", robot.mIntake.getIntakeCurrent());
//        telemetry.addData("intake pos", robot.mIntake.getPivotEncoderPos());
//        telemetry.addData("extendo pos", robot.mIntake.getExtendoPosition());
//        telemetry.addData("extendo target", robot.mIntake.getTargetExtendoPosition());
//        telemetry.addData("lift current", robot.mLift.getLiftCurrent());
//        telemetry.addData("lift pos", robot.mLift.getLiftTicks());
//        telemetry.addData("lift target", robot.mLift.getLiftTargetTicks());
//        telemetry.addData("left winch pos: ", robot.mDrive.leftwinchadjusted);
//        telemetry.addData("right winch pos: ", robot.mDrive.rightwinchadjusted);


        telemetry.update();
    }
}
