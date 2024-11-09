package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.List;

@TeleOp(name = "bum teleop", group = "opMode")
public class BSTEMTeleop extends OpMode {

    private Robot robot;
    public List<LynxModule> allHubs;

    private ElapsedTime timer;
    private ElapsedTime defenseTimer = new ElapsedTime();

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;
    private double  outputRate = 0.5;
    private double  lastOutput = 0.0;
    private boolean enableTelem = false;

    private boolean hanging;



    // private double intakeSpeed = 1425;



    private StickyButton verticalIncrease = new StickyButton();
    private StickyButton verticalDecrease = new StickyButton();

    private StickyButton intakeStepsButton = new StickyButton();

    private StickyButton intakeBackButton = new StickyButton();

    private StickyButton pivotButton = new StickyButton();
    private StickyButton clawButton = new StickyButton();


    private int liftpos = 0;
    private int intakeStateHits = 0;
    private int clawToggleHits = 1;
    private int pivotToggleHits = 2;
    private boolean firstTeleopLoop = true;
    private boolean enableCurrentReporting = true;
    private boolean firstStateLoop = true;

    // private Deadline imuTimer = new Deadline(500, TimeUnit.MILLISECONDS);

    private enum liftState {
        IDLE,
        CLOSE_GATE,
        ELEVATE,
        PIVOT,
        DROP_FRONT,
        DROP_BACK,
        BACK_CONTROL,
        HORZ_RETURN,
        PIVOT_RETURN,
        LIFT_RETURN
    }

    private liftState liftFSM = liftState.IDLE;

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
            // Set the target to 1.1 then back to 1.0 to make sure we latch it in
            firstTeleopLoop = false;

            if(enableCurrentReporting) {
                robot.setEnableCurrentReporting(true);
            }
        }



        // DRIVETRAIN
        double driveInput = -gamepad1.left_stick_y;
        double strafeInput = -gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;
        robot.mDrive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveInput, strafeInput), turnInput));






        // Just in case we getting ready to move to STOWED, shut intake down


        // LIFT
        verticalIncrease.update(gamepad1.right_bumper);
        verticalDecrease.update(gamepad1.left_bumper);

        if (verticalDecrease.getState()){
            liftpos--;
        }
        if (verticalIncrease.getState()){
            liftpos++;
        }
        telemetry.addData("life index", liftpos);

        liftpos = Range.clip(liftpos,0,3);
        robot.mLift.setTargetPos(liftpos, timer.seconds());

        // CLAW
        intakeStepsButton.update(gamepad1.a);
        intakeBackButton.update(gamepad1.b);
        clawButton.update(gamepad1.x);
        pivotButton.update(gamepad1.y);
        clawToggleHits += clawButton.getState() ? 1 : 0;
        pivotToggleHits += pivotButton.getState() ? 1 : 0;
        telemetry.addData("claw state", intakeStateHits%10);
        intakeStateHits += intakeStepsButton.getState() ? 1 : 0;
        if (intakeStateHits>0) {
            if (intakeStateHits% 10 == 2 || intakeStateHits% 10 == 0) {
                intakeStateHits -= intakeBackButton.getState() ? 2 : 0;
            } else {
                intakeStateHits -= intakeBackButton.getState() ? 1 : 0;

            }
        }
        if (intakeBackButton.getState()||intakeStepsButton.getState()){
            firstStateLoop = true;
        }
        switch (intakeStateHits % 10) {
            case 0: {
                robot.mLift.setClawPos(clawToggleHits%2);
                robot.mLift.setPivotPos(pivotToggleHits%2);
                break;

            }
            case 1: {
                clawToggleHits=2;
                pivotToggleHits=2;
                robot.mLift.setClawPos(0);
                intakeStateHits++;
                break;
            }
            case 2: {
                robot.mLift.setPivotPos(1);
                robot.mLift.setClawPos(0);
                //automatedTimerCanceller.reset();
                break;
            }
            case 3: {
                robot.mLift.setClawPos(1);
                robot.mLift.setPivotPos(1);
                break;
            }
            case 4: {
                if (firstStateLoop){
                    liftpos=0;
                    firstStateLoop = false;
                }

                robot.mLift.setPivotPos(0);
                break;
            }
            case 5: {
                if (firstStateLoop){
                    liftpos=3;
                    firstStateLoop = false;
                }
                robot.mLift.setPivotPos(0);

                break;
            }
            case 6: {
                robot.mLift.setPivotPos(1);
                robot.mLift.setClawPos(1);


                break;
            }
            case 7: {
                robot.mLift.setClawPos(0);
                robot.mLift.setPivotPos(1);
                break;
            }
            case 8: {
                if (firstStateLoop){
                    liftpos=3;
                    firstStateLoop = false;
                }
                robot.mLift.setPivotPos(0);
                robot.mLift.setClawPos(0);
                break;
            }
            case 9: {
                if (firstStateLoop){
                    liftpos=0;
                    firstStateLoop = false;
                }
                clawToggleHits=2;
                pivotToggleHits=2;
                intakeStateHits++;
                break;
            }
            default: {
                break;
            }
        }
        // HANG
        if (gamepad1.dpad_up){
            robot.mHang.setScrewOpenLoop(1);

        }else if(gamepad1.dpad_down) {
            robot.mHang.setScrewOpenLoop(-1);
            hanging = true;
        }else if (hanging){
            robot.mHang.setScrewOpenLoop(-0.05);
        }else{
            robot.mHang.stop();
        }


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



        telemetry.update();
    }
}
