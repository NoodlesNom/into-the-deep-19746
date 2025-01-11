package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.autonomous.gf.OldAutoMaster.team;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.List;
@Config
@TeleOp(name = "BSTEM teleop", group = "opMode")
public class BSTEMTELE extends OpMode {

    private Robot robot;
    public List<LynxModule> allHubs;
    public static double firstwait = 0.3;
    public static double intakewait = 0.15;
    public static double secondwait = 0.07;
    public static int intakepivaottime = 250;
    public static int transferextendopos = 140;
    private boolean stalledintaking = false;

    private ElapsedTime timer;

    private boolean autoclear = false;
    private ElapsedTime defenseTimer = new ElapsedTime();

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;
    private double  outputRate = 0.5;

    private boolean specangled = false;
    private double  lastOutput = 0.0;
    private boolean enableTelem = false;

    private boolean specplacing = false;

    private boolean hanging;

    private boolean samplesafe = false;


    // private double intakeSpeed = 1425;


    private boolean samplemode = false;

    private StickyButton angledbutton = new StickyButton();
    private StickyButton mode = new StickyButton();

    private StickyButton intakeStepsButton = new StickyButton();

    private StickyButton safemode = new StickyButton();

    private StickyButton pivotButton = new StickyButton();
    private StickyButton clawButton = new StickyButton();


    private int intakeStateHits = 0;
    private int clawToggleHits = 2;
    private ElapsedTime spectimer = new ElapsedTime();

    private ElapsedTime transfertimer = new ElapsedTime();
    private ElapsedTime generaltimer = new ElapsedTime();
    private ElapsedTime sampletimer = new ElapsedTime();

    private ElapsedTime shoottimer = new ElapsedTime();
    private int pivotToggleHits = 2;
    private boolean firstTeleopLoop = true;
    private boolean enableCurrentReporting = true;
    private boolean firstStateLoop = true;

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
        NOTHING
    }

    private teleState teleFSM = teleState.IDLE;
    private teleState prevtelestate = teleState.IDLE;

    public void init() {

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



        telemetry.addLine("Creating robot");
        telemetry.update();
        robot = new Robot(this);

        telemetry.addLine("Initing robot");
        telemetry.update();
        robot.teleopInit();
        robot.mDeposit.setLed(team);





        timer = new ElapsedTime();
        timer.reset();

        defenseTimer = new ElapsedTime();
        defenseTimer.reset();

        telemetry.addLine("Init finished");
        telemetry.update();

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
        }

        switch (teleFSM){
            case IDLE:{
                robot.mDeposit.setLed(team);
                if (prevtelestate != teleState.SAMPLE) {
                    if (samplemode){
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 500);
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                        robot.mDeposit.setDiffyPos(25, -90);
                    } else if (robot.mLift.getLiftTargetPos()==Lift.LIFT_POS.TRANSFERPREP.getVal()||robot.mLift.getLiftTargetPos()==Lift.LIFT_POS.SPECANGLED.getVal()) {
                        if (robot.mLift.getLiftTargetPos()==Lift.LIFT_POS.TRANSFERPREP.getVal()){
                            robot.mLift.setTargetPos(Lift.LIFT_POS.SPECANGLED.getVal(), timer.seconds());
                        }
                        if (robot.mLift.getLiftTargetPos()==Lift.LIFT_POS.SPECANGLED.getVal()&&robot.mLift.closeEnough()){
                            robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                        }
                        if (robot.mLift.getLiftTicks()>240){
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                        }
                    } else {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                        robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                    }
                }


                if(prevtelestate == teleState.TRANSFER){
                    if (transfertimer.seconds()>0.3){
                        robot.mDeposit.setDiffyPos(0,0);
                        robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());

                    }
                }else if(prevtelestate == teleState.SAMPLE){
                    if (sampletimer.seconds()>0.5){
                        if (samplemode){
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
                            robot.mDeposit.setDiffyPos(25, -90);
                            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        }else {
                            robot.mLift.setTargetPos(Lift.LIFT_POS.DOWN.getVal(), timer.seconds());
                            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                            robot.mDeposit.setDiffyPos(0, 0);
                            robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                        }
                        robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                        robot.mIntake.setIntakeOpenLoop(0);
                        robot.mIntake.setExtendoPos(0, timer.seconds());


                    }
                }else{
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0, timer.seconds());
                    if (samplemode){
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                    }else {
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                    }


                }
                break;

            }
            case SPECINTAKE:{
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                robot.mDeposit.setDiffyPos(70,-90);
                robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECINTAKE.getVal());

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
                if (shoottimer.seconds()>0.6){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                }else if (shoottimer.seconds()>0.3){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(-1);
                }else{
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                }
                if (spectimer.seconds()>0.5){
                    if (specangled) {
                        if (specplacing) {
                            robot.mDeposit.setDiffyPos(-85, 90);
                        }
                        robot.mLift.setTargetPos(Lift.LIFT_POS.SPECANGLED.getVal(), timer.seconds());
                    }else{
                        if (specplacing) {
                            robot.mDeposit.setDiffyPos(-50, 70);
                        }
                        robot.mLift.setTargetPos(Lift.LIFT_POS.SPECIMEN_PLACE.getVal(), timer.seconds());
                    }

                }
                if (spectimer.seconds()>0.4) {
                    if (specangled) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPECANGLED.getVal());
                    }else{
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SPEC.getVal());
                    }

                }
                break;

            }
            case INTAKING:{
                if (samplemode) {
                    robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal(), 400);
                    robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
                    robot.mDeposit.setDiffyPos(25, -90);
                }
                if (gamepad1.right_trigger>0.2&& robot.mIntake.getExtendoPosition()<450) {
                    robot.mIntake.setExtendoOpenLoop(gamepad1.right_trigger*0.7);
                }else if (gamepad1.left_trigger>0.2&& robot.mIntake.getExtendoPosition()>20){
                    robot.mIntake.setExtendoOpenLoop(-gamepad1.left_trigger*0.7);
                }else {
                    robot.mIntake.setExtendoOpenLoop(0);
                }
                if (gamepad1.right_bumper){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
                    if (robot.mIntake.getIntakeCurrent()<6.7&&!stalledintaking) {
                        robot.mIntake.setIntakeOpenLoop(0.8);
                    }else{
                        robot.mIntake.setIntakeOpenLoop(0);
                        stalledintaking= true;
                    }

                }else if(gamepad1.left_bumper){
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    stalledintaking = false;
                    robot.mIntake.setIntakeOpenLoop(-0.5);
                }else {
                    stalledintaking = false;
                    robot.mIntake.setIntakeOpenLoop(0);
                }
                if (gamepad1.x){
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                }
                if (gamepad1.b){
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
                    teleFSM = teleState.NOTHING;
                }


                break;
            }
            case TRANSFER:{
                robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                robot.mDeposit.setDiffyPos(25, -90);
                if (robot.mLift.closeEnough()&&robot.mIntake.closeEnough()&&!transferready){
                    robot.mIntake.setExtendoOpenLoop(-0.3);
                    robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                    if (transfertimer.seconds()>firstwait){
                        robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());
                        transferready = true;
                        robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
                        transfertimer.reset();
                        robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFER.getVal(), timer.seconds());

                    }else if (transfertimer.seconds()>intakewait){
                        robot.mIntake.setIntakeOpenLoop(0.6);
                    }
                }else if (transferready){
                    if (robot.mLift.getLiftTicks()<135){
                        clawToggleHits = 1;
                        if (transfertimer.seconds()>secondwait) {
                            prevtelestate = teleFSM;
                            teleFSM = teleState.SAMPLE;

                            robot.mIntake.setIntakeOpenLoop(0);
                            transfertimer.reset();
                            robot.mIntake.setGatePos(Intake.GATE_POS.OPEN.getVal());
                            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.TRANSFER.getVal(), timer.seconds());
                        }
                    }else{
                        transfertimer.reset();
                    }

                } else {
                    transfertimer.reset();
                    clawToggleHits = 2;
                }


                break;
            }
            case SHOOT:{
                robot.mLift.setTargetPos(Lift.LIFT_POS.SPECINTAKE.getVal(), timer.seconds());
                if (shoottimer.seconds()>1){
                    robot.mLift.setTargetPos(0, timer.seconds());
                    prevtelestate = teleFSM;
                    teleFSM = teleState.IDLE;
                }else if (shoottimer.seconds()>0.8){
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
                }else if (shoottimer.seconds()>0.5){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(-1);
                }else if (shoottimer.seconds()>0.3){
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.LAUNCH.getVal());

                }
                break;
            }
            case SAMPLE:{
                if (transfertimer.seconds()>0.3){
                    robot.mIntake.setGatePos(Intake.GATE_POS.CATCH.getVal());
                    robot.mIntake.setIntakeOpenLoop(0);
                    robot.mIntake.setExtendoPos(0, timer.seconds());
                    robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());

                }
                if (samplesafe){
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SAMPLESAFE.getVal(), timer.seconds());
                    if (robot.mLift.getLiftTicks()>730) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SAMPLE.getVal());
                        if (robot.mLift.closeEnough()){
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            gamepad1.rumble(400);
                        }else{
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            gamepad1.rumble(100);
                        }
                        robot.mDeposit.setDiffyPos(-60, 0);
                    }else{
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                        robot.mDeposit.setDiffyPos(0, -90);
                    }
                }else {
                    robot.mLift.setTargetPos(Lift.LIFT_POS.SAMPLE.getVal(), timer.seconds());
                    if (robot.mLift.getLiftTicks()>650) {
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.SAMPLE.getVal());
                        if (robot.mLift.closeEnough()){
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            gamepad1.rumble(400);
                        }else{
                            robot.mDeposit.setLed(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            gamepad1.rumble(100);
                        }
                        robot.mDeposit.setDiffyPos(-60, 0);
                    }else{
                        robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
                        robot.mDeposit.setDiffyPos(0, -90);
                    }
                }


                if (gamepad1.a){
                    prevtelestate = teleFSM;
                    teleFSM = teleState.IDLE;
                    sampletimer.reset();
                    gamepad1.rumble(200);
                    gamepad2.rumble(200);
                }
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
            clawToggleHits = 1;
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
        if (gamepad1.right_trigger>0.2&&teleFSM!=teleState.INTAKING){
            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
            prevtelestate = teleFSM;
            teleFSM = teleState.INTAKING;
        }

        if (gamepad2.a||(gamepad1.a&&teleFSM == teleState.INTAKING&&!samplemode)){
            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
            robot.mLift.setTargetPos(Lift.LIFT_POS.TRANSFERPREP.getVal(), timer.seconds());
            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.TRANSFER.getVal());
            prevtelestate = teleFSM;
            teleFSM = teleState.TRANSFER;
            transferready = false;
            robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
            generaltimer.reset();
            transfertimer.reset();
            robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRANSFER.getVal());
            clawToggleHits = 2;
            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
        }
        if (gamepad2.x){
            prevtelestate = teleFSM;
            teleFSM = teleState.IDLE;
        }
        if (gamepad2.dpad_down){
            prevtelestate = teleFSM;
            teleFSM = teleState.SHOOT;
            shoottimer.reset();
            robot.mDeposit.setPivotPos(Deposit.PIVOT_POS.IDLE.getVal());
        }
        if (gamepad2.dpad_up||gamepad2.dpad_left){
            prevtelestate = teleFSM;
            teleFSM = teleState.SAMPLE;
        }
        safemode.update(gamepad2.dpad_left);
        if (safemode.getState()){
            samplesafe = !samplesafe;
        }
        angledbutton.update(gamepad2.right_bumper);
        if (angledbutton.getState()){
            specangled = !specangled;
        }
        if (gamepad1.b){
            stalledintaking = false;
            robot.mIntake.setGatePos(Intake.GATE_POS.CLAMP.getVal());
            robot.mIntake.setExtendoPos(Intake.EXTEND_POS.STOWED.getVal(), timer.seconds());
            if (samplemode){
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.TRAP.getVal());
            }else{
                robot.mIntake.setPivotPos(Intake.PIVOT_POS.IDLE.getVal());
            }
            robot.mIntake.setIntakeOpenLoop(0);
        }


        // DRIVETRAIN
        double driveInput = -gamepad1.left_stick_y;
        double strafeInput = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;
        if (false){
            driveInput*=0.8;
            strafeInput*=0.8;
            turnInput*=0.8;
        }
        robot.mDrive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveInput, strafeInput), turnInput));






        // Just in case we getting ready to move to STOWED, shut intake down




        if (clawButton.getState()&&teleFSM!=teleState.INTAKING){
            clawToggleHits++;
            if (specplacing){
                specplacing= false;
                robot.mDeposit.setDiffyPos(0,90);
                robot.mDeposit.setLed(team);
                gamepad2.rumble(200);
            }
        }

        robot.mDeposit.setClawPos(clawToggleHits%2);





        // update robot
        robot.update(timer.seconds());

        //BotLog.logD("Lift :: ", String.format("%s", robot.mLift.getTelem(timer.seconds())));
        //BotLog.logD("Hang :: ", String.format("%s", robot.mHang.getTelem(timer.seconds())));

        if ((timer.seconds() > (lastOutput + outputRate)) && enableTelem)
        {
            BotLog.logD("robot :: ", String.format("%s", robot.getTelem(timer.seconds())));
            // BotLog.logD("lift :: ", String.format("%s", robot.mLift.getTelem(timer.seconds())));
            lastOutput = timer.seconds();
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

        telemetry.addData("fl current", robot.mDrive.drive.mPeriodicIO.flcurrent);
        telemetry.addData("fr current", robot.mDrive.drive.mPeriodicIO.frcurrent);
        telemetry.addData("bl current", robot.mDrive.drive.mPeriodicIO.blcurrent);
        telemetry.addData("br current", robot.mDrive.drive.mPeriodicIO.brcurrent);
        telemetry.addData("intake current", robot.mIntake.getIntakeCurrent());
        telemetry.addData("extendo current", robot.mIntake.getExtendoCurrent());
        telemetry.addData("extendo pos", robot.mIntake.getExtendoPosition());
        telemetry.addData("extendo target", robot.mIntake.getTargetExtendoPosition());
        telemetry.addData("lift current", robot.mLift.getLiftCurrent());
        telemetry.addData("lift pos", robot.mLift.getLiftTicks());
        telemetry.addData("lift target", robot.mLift.getLiftTargetTicks());


        telemetry.update();
    }
}
