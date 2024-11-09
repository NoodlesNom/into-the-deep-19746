//package org.firstinspires.ftc.teamcode.autonomous;//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.autonomous.vision.VisionLibrary;
//import org.firstinspires.ftc.teamcode.autonomous.vision.SignalSleevePosition;
//import org.firstinspires.ftc.teamcode.util.buttons.StickyButton;
//
//@Photon(maximumParallelCommands = 6)
//
//public abstract class BaseAuto extends LinearOpMode {
//    Robot robot=null;
//    public boolean useThreads = false;
//
//    private StickyButton awayStickyButton = new StickyButton();
//    private StickyButton nearStickyButton = new StickyButton();
//    private StickyButton leftStickyButton = new StickyButton();
//    private StickyButton rightStickyButton = new StickyButton();
//    private StickyButton goStickyButton = new StickyButton();
//
//    public double xModifier = 0.0;
//    public double yModifier = 0.0;
//    public double startDelay = 0.0;
//    long prevTime = 0;
//
//    public ElapsedTime runtime = new ElapsedTime();
//    public ElapsedTime parkTimer = new ElapsedTime();
//    public boolean emergencyPark = false;
//    private boolean motorsOff = false;
//    public boolean attemptRecovery = true;
//    private boolean loggingEnabled = false;
//
//    private SignalSleevePosition signalSleevePosition = SignalSleevePosition.UNKNOWN;
//
//    @Override
//    public void runOpMode() {
//        PhotonCore.photon.maximumParallelCommands();
//
//
//        telemetry.addLine("Creating Robot class");
//        telemetry.update();
//        robot = new Robot(this);
//
//        // Just in case soemthing weird happened in init
//
//        telemetry.addLine("Creating Vision Library");
//        telemetry.update();
//        VisionLibrary visionLibrary = new VisionLibrary(this);
//
//        if(useThreads) {
//            telemetry.addLine("Starting threads");
//            telemetry.update();
//        }
//
//        telemetry.addLine("InitAuto for components");
//        telemetry.update();
//        robot.autoInit();
//
//        showTelemetry();
//
//        //assignUltraSonic(robot);
//        alignRobot();
//        addDelay();
//        confirmRecovery();
//        enterPoleCorrection();
//
//        if (isStopRequested()) return;
//
//
//        telemetry.addLine("Building paths");
//        telemetry.update();
//        buildPaths(robot);
//
//        telemetry.addLine("Initializing Vision Library");
//        telemetry.update();
//        visionLibrary.init();
//
//        SignalSleevePosition signalSleevePositionTemp;
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            // Never forget the last valid thing we saw.
//            signalSleevePositionTemp = visionLibrary.getSignalSleevePosition();
//            if (signalSleevePositionTemp != SignalSleevePosition.UNKNOWN) {
//                signalSleevePosition =  signalSleevePositionTemp;
//            }
//
//            showTelemetry();
//        }
//
//        runtime.reset();
//        parkTimer.reset();          // DO NOT mess with this timer ever!!!
//
//        visionLibrary.stopVision();
//        if (isStopRequested()) return;
//
//        if (signalSleevePosition == SignalSleevePosition.UNKNOWN) {
//            signalSleevePosition =  SignalSleevePosition.TWO;
//        }
//
//        if(startDelay > 0.0) {
//            CheckWait(startDelay*1000);
//        }
//        runMain(robot, signalSleevePosition);
//
//        if(useThreads) {
//            robot.stop();
//        }
//    }
//
//    private void confirmRecovery() {
//        goStickyButton.update(gamepad1.a);
//        while ((goStickyButton.getState()==false) && !opModeIsActive() && !isStopRequested()) {
//            if (isStopRequested()) return;
//            awayStickyButton.update(gamepad1.dpad_up);
//            attemptRecovery = awayStickyButton.getState() ? !attemptRecovery : attemptRecovery;
//            nearStickyButton.update(gamepad1.dpad_down);
//            attemptRecovery = nearStickyButton.getState() ? !attemptRecovery : attemptRecovery;
//
//            telemetry.addLine("Press up/down dpad to disable auto-recovery.");
//            telemetry.addData("Auto-recovery: ", "%s", attemptRecovery);
//            telemetry.addLine("");
//            telemetry.addLine("Press <X> to continue...");
//
//            telemetry.update();
//
//            goStickyButton.update(gamepad1.a);
//        }
//    }
//
//    private void addDelay() {
//        startDelay = 0.0;
//        goStickyButton.update(gamepad1.a);
//        while ((goStickyButton.getState()==false) && !opModeIsActive() && !isStopRequested()) {
//            if (isStopRequested()) return;
//            awayStickyButton.update(gamepad1.dpad_up);
//            startDelay += awayStickyButton.getState() ? 0.25 : 0.0;
//            nearStickyButton.update(gamepad1.dpad_down);
//            startDelay -= nearStickyButton.getState() ? 0.25 : 0.0;
//            startDelay = Math.max(startDelay,0.0);
//
//            telemetry.addLine("Add delay.");
//            telemetry.addData("startDelay: ", "%.2f seconds", startDelay);
//            telemetry.addLine("");
//            telemetry.addLine("Press <X> to continue...");
//
//            telemetry.update();
//
//            goStickyButton.update(gamepad1.a);
//        }
//    }
//
//
//    private void enterPoleCorrection() {
//        goStickyButton.update(gamepad1.a);
//        while ((goStickyButton.getState()==false) && !opModeIsActive() && !isStopRequested()) {
//            if (isStopRequested()) return;
//            awayStickyButton.update(gamepad1.dpad_up);
//            yModifier += awayStickyButton.getState() ? 0.5 : 0.0;
//            nearStickyButton.update(gamepad1.dpad_down);
//            yModifier -= nearStickyButton.getState() ? 0.5 : 0.0;
//
//            leftStickyButton.update(gamepad1.dpad_left);
//            xModifier -= leftStickyButton.getState() ? 0.5 : 0.0;
//            rightStickyButton.update(gamepad1.dpad_right);
//            xModifier += rightStickyButton.getState() ? 0.5 : 0.0;
//
//            telemetry.addLine("Configure X/Y offsets.");
//            telemetry.addData("XModifier: ", "%.1f (%s)", xModifier, (xModifier<0)?"Left":((xModifier>0)?"Right":"None"));
//            telemetry.addData("YModifier: ", "%.1f (%s)", yModifier, (yModifier<0)?"Farther":((yModifier>0)?"Closer":"None"));
//            telemetry.addLine("");
//            telemetry.addLine("Press <X> to continue...");
//
//            telemetry.update();
//
//            goStickyButton.update(gamepad1.a);
//        }
//    }
//
//    private void alignRobot() {
//        // Align robot with Ultra sonics if we have them
//        if (robot.otherSideUltra != null) {
//
//            goStickyButton.update(gamepad1.a);
//            while ((goStickyButton.getState() == false) && !opModeIsActive() && !isStopRequested()) {
//                if (isStopRequested()) return;
//
//                // Reset and enable the ultraSonic
//                robot.otherSideUltra.reset();
//                robot.otherSideUltra.enableUltra = true;
//
//                // Capture for 105ms to get a new update
//                double now = runtime.seconds();
//                while (runtime.seconds() < now + .105) {
//                    robot.otherSideUltra.captureDistance();
//                }
//
//                // Display
//                telemetry.addLine("Align robot");
//                telemetry.addData("UltraSonic: ", "%.1f (target:727.0)", robot.otherSideUltra.maxS);
//                telemetry.addLine("");
//                telemetry.addLine("Press <X> to continue...");
//
//                telemetry.update();
//
//                goStickyButton.update(gamepad1.a);
//            }
//            // Disable and reset
//            robot.otherSideUltra.enableUltra = false;
//            robot.otherSideUltra.reset();
//        }
//    }
//
//
//    public void showTelemetry() {
//        //telemetry.addData("clawToggleHits: ", clawToggleHits);
//        double cycleTime = System.currentTimeMillis() - prevTime;
//        telemetry.addData("Loop Speed", cycleTime != 0 ? 1000 / cycleTime : Double.NaN);
//        telemetry.addData("Claw Goal", robot.claw.getCurrentGoal());
//        telemetry.addData("Lift Mode", robot.mLift.getHeight());
//        telemetry.addData("Lift Goal", robot.mLift.getGoal());
//        telemetry.addData("Act Height: ", robot.mLift.getLiftEncoderTicks());
//        telemetry.addData("Tgt Height: ", robot.mLift.getTgtPos());
//        telemetry.addData("Lift pwr: ", robot.mLift.pwr);
//        telemetry.addData("Min pwr: ", robot.mLift.pid.getOutputMin());
//        telemetry.addData("Max pwr: ", robot.mLift.pid.getOutputMax());
//        telemetry.addData("IMU: ", robot.drive.imuCH.getCalibrationStatus());
//        telemetry.addData("Recovery: ", attemptRecovery);
//
//        telemetry.addLine("");
//        telemetry.addData("Status", "Waiting...");
//        telemetry.addData("Signal Pos", signalSleevePosition);
//
//        telemetry.update();
//        prevTime = System.currentTimeMillis();
//
//        /*
//        if(loggingEnabled) {
//            robot.logger.logD("CheckWait", String.format(" Time: %.2f, CG: %s, LM: %s, LG:%s, AH: %.0f TH: %.0f LP: %.2f V:%.2f, Min: %.2f Max: %.2f",
//                    runtime.seconds(),
//                    robot.claw.getCurrentGoal(),
//                    robot.mLift.getHeight(),
//                    robot.mLift.getGoal(),
//                    robot.mLift.getLiftEncoderTicks(),
//                    robot.mLift.getTgtPos(),
//                    robot.mLift.pwr,
//                    robot.drive.batteryVoltageSensor.getVoltage(),
//                    robot.mLift.pid.getOutputMin(),
//                    robot.mLift.pid.getOutputMax()));
//        }
//        */
//
//    }
//
//    public abstract void buildPaths(Robot robot);
//    public abstract void runMain(Robot robot, SignalSleevePosition signalSleevePosition);
//    public void assignUltraSonic(Robot robot) {};
//    public double getP(){ return(3); }
//    public boolean useDeadWheels(){ return(true); }
//
//
////    public Boolean CloseEnough(Robot robot, Pose2d tgtPose, double errX, double errY, double errHeading ) {
////        // Are we there yet?
////        if( tgtPose!= null ) {
////            Pose2d  errPose = tgtPose.minus(robot.drive.getLocalizer().getPoseEstimate());
////
////            if((errX > 0.0) && (Math.abs(errPose.getX()) > errX)) {
////                return(false);
////            }
////            if((errY > 0.0) && (Math.abs(errPose.getY()) > errY)) {
////                return(false);
////            }
////            if((errHeading > 0.0) && (Math.abs(errPose.getHeading()) > errHeading)) {
////                return(false);
////            }
////        }
////        return(true);
////    }
//
//    // This routine will manage drive and component updates and keep time
//    // It is meant to fulfill the roles of the FollowTrajectory(), sleep(), and threading
//    // If threading is enabled, component update is skipped
//    // This routine will monitor the drive for idle for a min of minMS and max of maxMS
//    // We return the drive status on exit in case the drive was not done by our timeout -- this
//    // might be considered an error and we might consider shutting auto down.
//    // If the drive is idle, the minMS can be thought of as a sleep that updates components.
//    public boolean CheckWait(double minMS) { return(CheckWait(minMS, minMS)); }
//    public boolean CheckWait(double minMS, double maxMS) { return(CheckWait(minMS, minMS,false)); }
//    public boolean CheckWait(double minMS, boolean holdPosition) { return(CheckWait(minMS, minMS, holdPosition)); }
//    public boolean CheckWait(double minMS, double maxMS, boolean holdPosition) {  return(CheckWait( minMS,  maxMS,  holdPosition, null, 0.0, 0.0, 0.0)); }
//    public boolean CheckWait(double minMS, double maxMS, boolean holdPosition, Pose2d tgtPose, double errX, double errY, double errHeading ) {
//        boolean checkDrive = true;
//        boolean updateComponents = !useThreads;
//        NanoClock localClock = NanoClock.system();
//        double now = localClock.seconds();
//
//        if(emergencyPark) {
//            if(!motorsOff) {
//                robot.drive.setMotorPowers(0, 0, 0, 0);
//                motorsOff = true;
//            }
//            return(robot.drive.isBusy());
//        }
//
//        // Convert to seconds
//        minMS /= 1000;
//        maxMS /= 1000;
//
//        minMS += now;
//        if (maxMS > 0) {
//            maxMS += now;
//        } else {
//            maxMS = Double.POSITIVE_INFINITY;
//        }
//
//        while (opModeIsActive()) {
//            double cycleTime = System.currentTimeMillis() - prevTime;
//            telemetry.addData("Loop Speed", cycleTime != 0 ? 1000 / cycleTime : Double.NaN);
//            telemetry.update();
//            prevTime = System.currentTimeMillis();
//            // Get the time
//            now = localClock.seconds();
//
//           // showTelemetry();
//
//            // Master stop
//            if (!opModeIsActive()) {
//                if(!motorsOff) {
//                    robot.drive.setMotorPowers(0, 0, 0, 0);
//                    motorsOff = true;
//                }
//                return(robot.drive.isBusy());
//            }
//
//            // Update the drive
//            robot.drive.update();
//            if(holdPosition && !robot.drive.isBusy()) {
//                robot.drive.updateHold();
//            }
//            // Motors are likely powered now
//            motorsOff = false;
//
//            // Update the components
//            if (updateComponents) {
//                robot.updateComponents();
//            }
//
//            // Check timer expiration, bail if too long
//            if (maxMS < now) {
//                // Let's not leave the drive running...
//                if(!motorsOff) {
//                    robot.drive.setMotorPowers(0, 0, 0, 0);
//                    motorsOff = true;
//                }
//                return(robot.drive.isBusy());
//            }
//
//            // Make sure to wait for the minimum time
//            if (minMS > now) {
//                continue;
//            }
//
//            // Drive still running? Wait for it.
//            if (robot.drive.isBusy()) {
//                continue;
//            }
//
//            // In the hold position case,
//            // Keep correcting until we reach the target or the max time expires
//            if( holdPosition && (tgtPose!= null)) {
//                if( ! CloseEnough(robot, tgtPose, errX, errY, errHeading )) {
//                    continue;
//                }
//            }
//
//            // No reason to be here (past the minMS timer, drive is idle)
//            // Let's not leave the drive running...
//            if(!motorsOff) {
//                robot.drive.setMotorPowers(0, 0, 0, 0);
//                motorsOff = true;
//            }
//            return (robot.drive.isBusy());
//        }
//        // Let's not leave the drive running...
//        if(!motorsOff) {
//            robot.drive.setMotorPowers(0, 0, 0, 0);
//            motorsOff = true;
//        }
//        return (robot.drive.isBusy());
//    }
///*
//    public void ColorCorrect(Pose2d tgtPose, double kPt, double kPh, double minTO, double d) { ColorCorrect(tgtPose, kPt, kPh, minTO,d, true); }
//    public void ColorCorrect(Pose2d tgtPose, double kPt, double kPh, double minTO, double d,Boolean ColorCorrectEnable) { ColorCorrect(tgtPose, kPt, kPh, minTO,d, true, 0.5); }
//    public void ColorCorrect(Pose2d tgtPose, double kPt, double kPh, double minTO, double d, Boolean ColorCorrectEnable, double errY) {
//        Pose2d currentPose = robot.drive.getPoseEstimate();
//        Pose2d currentPose2 ;
//        Pose2d newPose = currentPose;
//        Pose2d newPose2 ;
//        double deltaS = 0.0 ;
//        double timeMS = 0.0 ;
//        double myShift = 0.0 ;
//        double old_kPt = robot.drive.TRANSLATIONAL_PID.kP ;
//        double old_kPh = robot.drive.HEADING_PID.kP ;
//        boolean goodMeasure = false;
//        double  headingDelta = (AngleUnit.DEGREES.normalize(Math.toDegrees(currentPose.getHeading()-tgtPose.getHeading())));
//
//        robot.logger.logD("CurTgtCC:", String.format(" Time: %.2f, Cur:%s, Tgt:%s", runtime.seconds(),currentPose,tgtPose));
//        // If we're in trouble, bail out
//        if (emergencyPark)
//        {
//            return;
//        }
//
//        // If more than 10 degrees from our target or not against the wall, we need to take some corrective actin
//        if((Math.abs(headingDelta) > 10.0) || (currentPose.getX() > -60.0))
//        {
//            if(attemptRecovery) {
//                // Drive to a safe spot
//                if ((Math.abs(currentPose.getY()) > 20) || (currentPose.getX() > -48)) {
//                    robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(currentPose)
//                        .back(10)
//                        .build());
//                    CheckWait(0);
//                }
//                robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
//                        .lineToSplineHeading(new Pose2d(-39, tgtPose.getY(), tgtPose.getHeading()), BMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                BMecanumDrive.getAccelerationConstraint(60))
//                        .build());
//                CheckWait(0);        // FollowTrajectory
//
//                // Now record where we are
//                currentPose2 = robot.drive.getPoseEstimate();
//                headingDelta = (AngleUnit.DEGREES.normalize(Math.toDegrees(currentPose2.getHeading() - tgtPose.getHeading())));
//
//                // If we are pointed the right way, relocalize with the ultra sonic
//                if (Math.abs(headingDelta) < 3.0) {
//                    double myY = 0.0;
//                    double ultraDist = 0.0;
//
//                    if(robot.ultra != null) {
//                        // Take an ultra sonic reading
//                        robot.ultra.reset();
//                        robot.ultra.enableUltra = true;
//                        CheckWait(105, false);
//                        // Compute the distance from the wall to our robot center
//                        ultraDist = ((robot.ultra.maxS / 25.4) + 6.81);
//                        robot.ultra.enableUltra = false;
//                        robot.logger.logD("SmartCone US:", String.format(" Time: %.2f, CP:%s, CP2:%s, NP:%s, now:%s, tgt:%s, :%3.1f",
//                                runtime.seconds(), currentPose, currentPose2, newPose, robot.drive.getPoseEstimate(), tgtPose, ultraDist));
//
//                        if (ultraDist < 71.0) {
//                            // Make sure we're on our half
//                            myY = d * (71.0 - ultraDist);
//                            //myShift = d * ((59.0 - 6.81) - (robot.ultra.maxS / 25.4));
//                            newPose = new Pose2d(currentPose2.getX(), myY, currentPose2.getHeading());
//                            robot.logger.logD("SmartCone US:", String.format(" Time: %.2f, CP:%s, CP2:%s, NP:%s, now:%s, tgt:%s, myY:%3.1f",
//                                    runtime.seconds(), currentPose, currentPose2, newPose, robot.drive.getPoseEstimate(), tgtPose, myY));
//                            // Assuming we are reasonably close to the expected Y, use the ultra sonic data
//                            if (Math.abs(myY - tgtPose.getY()) < 5.0) {
//                                robot.drive.setPoseEstimate(newPose);
//                            }
//                        }
//                    }
//                } else {
//                    robot.logger.logD("SmartCone NO US:", String.format(" Time: %.2f, CP:%s, CP2:%s, NP:%s, now:%s, tgt:%s",
//                            runtime.seconds(), currentPose, currentPose2, newPose, robot.drive.getPoseEstimate(), tgtPose));
//                }
//                // Try to re-approach the conestack
//                robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
//                        .lineToSplineHeading(tgtPose, BMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                BMecanumDrive.getAccelerationConstraint(60))
//                        .build());
//                CheckWait(0);        // FollowTrajectory
//
//                // Logging
//                robot.logger.logD("SmartCone:", String.format(" Time: %.2f, CP:%s, CP2:%s, NP:%s, now:%s, tgt:%s",
//                        runtime.seconds(), currentPose, currentPose2, newPose, robot.drive.getPoseEstimate(), tgtPose));
//
//                // Update the currentPose and headingDelta
//                currentPose = robot.drive.getPoseEstimate();
//                headingDelta = (AngleUnit.DEGREES.normalize(Math.toDegrees(currentPose.getHeading() - tgtPose.getHeading())));
//            }
//
//            // If we are still in pretty bad shape, emergency park
//            if((Math.abs(headingDelta) > 10.0) || (currentPose.getX() > -60.0)) {
//                emergencyPark=true;
//                return;
//            }
//        }
//
//        // If correction enabled
//        if(false) {
////            // Take a color measurement
////            goodMeasure = robot.color5.measure();
////
////            // Use it or try UltraSonic
////            if (goodMeasure) {
////                myShift = robot.color5.shiftedAmount;
////            } else {
////                if(robot.ultra != null) {
////                    // We are only here if we are < 10 degrees of twist and close to the wall
////                    robot.ultra.reset();
////                    robot.ultra.enableUltra = true;
////                    CheckWait(105, false);
////                    myShift = d * ((59.0 - 6.81) - (robot.ultra.maxS / 25.4));
////                    robot.ultra.enableUltra = false;
////                }
////            }
////
////            // Reject wild readings
////            if ((Math.abs(myShift) >= 0.0) && (Math.abs(myShift) < 5.0)) {
////
////                // Compute our current location, update localization and determine how much time we need to correct
////                newPose = new Pose2d(tgtPose.getX(), tgtPose.getY() + myShift, currentPose.getHeading());
////                robot.drive.setPoseEstimate(newPose);
////                deltaS = newPose.getY() - tgtPose.getY();
////                timeMS = Math.abs(deltaS) * 500.0;
////
////                // logging
////                robot.logger.logD("ColorInfo:", String.format(" Time: %.2f, lY:%2.2f, eY:%2.2f, s:%2.2f, d:%2.2f, m:%s, %s,%s,%s,%s,%s",
////                        runtime.seconds(),robot.drive.getPoseEstimate().getY(),tgtPose.getY(),myShift,deltaS,goodMeasure,
////                        robot.color5.L2OnTape,robot.color5.L1OnTape,robot.color5.COnTape,robot.color5.R1OnTape,robot.color5.R2OnTape));
////
////                robot.logger.logD("CurTgt-2:", String.format(" Time: %.2f, Cur:%s, Tgt:%s", runtime.seconds(),robot.drive.getPoseEstimate(),tgtPose));
////
////                // If we needed some time and we're not pretty close already, do a correction cycle
////                if ((timeMS > 0.0) && !CloseEnough(robot, tgtPose, 0.0, errY, 2.0)) {
////                    robot.drive.TRANSLATIONAL_PID.kP = kPt;
////                    robot.drive.HEADING_PID.kP = kPh;
////                    CheckWait(0.0, timeMS, true, tgtPose, 0.0, errY, 2.0);
////                }
////
////                // Grab a cone
////                robot.claw.setClawServoPosition(ClawPosition.CLOSED);
////
////                robot.logger.logD("CurTgt-1:", String.format(" Time: %.2f, Cur:%s, Tgt:%s", runtime.seconds(),robot.drive.getPoseEstimate(),tgtPose));
////
////                // Wait for claw to close
////                CheckWait(minTO, true);
////
////
////                // Get the new pose after our shift
////                currentPose2 = robot.drive.getPoseEstimate();
////                // Preserve the Y and heading
////                // But figure out how much 'X' motion we had in our motion and adjust the original X value on function entry  based on that.
////                newPose2 = new Pose2d(currentPose.getX() - (tgtPose.getX() - currentPose2.getX()), currentPose2.getY(), currentPose2.getHeading());
////                robot.drive.setPoseEstimate(newPose2);
////
////                robot.drive.TRANSLATIONAL_PID.kP = old_kPt;
////                robot.drive.HEADING_PID.kP = old_kPh;
////
////                // Logging
////                robot.logger.logD("XInfo:", String.format(" Time: %.2f, CP:%.2f, NP:%.2f, CP2%.2f, NP2%.2f, TP:%.2f",
////                        runtime.seconds(), currentPose.getX(), newPose.getX(), currentPose2.getX(), newPose2.getX(), tgtPose.getX()));
////                robot.logger.logD("Yes Correction:", String.format(" Time: %.2f, %s, %s", runtime.seconds(), robot.drive.getPoseEstimate(), tgtPose));
////                robot.logger.logD("Cone Stack Heading:", String.format(" Time: %.2f, %s, %s", runtime.seconds(), Math.toDegrees(AngleUnit.RADIANS.normalize(robot.drive.getRawExternalHeading()-robot.drive.startingCH)), tgtPose));
////                robot.logger.logD("CurTgtDone:", String.format(" Time: %.2f, Cur:%s, Tgt:%s", runtime.seconds(),robot.drive.getPoseEstimate(),tgtPose));
////                return;
////            } else {
////                emergencyPark = true;
////                return;
////            }
//        }
//        // This falls through if there is no correction.
//        robot.logger.logD("No Correction:", String.format(" Time: %.2f, %s, %s", runtime.seconds(), robot.drive.getPoseEstimate(), tgtPose));
//        robot.logger.logD("Cone Stack Heading:", String.format(" Time: %.2f, %s, %s", runtime.seconds(), Math.toDegrees(AngleUnit.RADIANS.normalize(robot.drive.getRawExternalHeading()-robot.drive.startingCH)), tgtPose));
//        CheckWait(minTO, true);
//    }
//*/
//    /*
//    public void showPosition(String path, Pose2d expectedPose){
//        if(loggingEnabled) {
//            Pose2d drivePose = robot.drive.getPoseEstimate();
//            Pose2d locPose = robot.drive.getLocalizer().getPoseEstimate();
//            robot.logger.logD("showPosition", String.format(" Path: %11.11s, Time: %.2f, lX:%2.2f,eX:%2.2f,(%.2f),lY:%2.2f,eY:%2.2f,(%.2f),lh:%3.0f,eh:%3.0f,",
//                    path,
//                    runtime.seconds(),
//                    //drivePose.getX(),
//                    locPose.getX(),
//                    expectedPose.getX(),
//                    locPose.getX() - expectedPose.getX(),
//                    //drivePose.getY(),
//                    locPose.getY(),
//                    expectedPose.getY(),
//                    locPose.getY() - expectedPose.getY(),
//                    //Math.toDegrees(drivePose.getHeading()),
//                    Math.toDegrees(locPose.getHeading()),
//                    Math.toDegrees(expectedPose.getHeading()) ) );
//        }
//    }
//
//    public void showUltraInfo(String path, Pose2d expectedPose){
//        if(loggingEnabled) {
//            Pose2d locPose = robot.drive.getLocalizer().getPoseEstimate();
//            robot.logger.logD("UltraInfo:", String.format(" Path: %s, Time: %.2f, lY:%2.2f,eY:%2.2f,uY:%2.2f",
//                    path,
//                    runtime.seconds(),
//                    locPose.getY(),
//                    expectedPose.getY(),
//                    robot.ultraRight.maxS/25.4));
//        }
//    }
//    */
//}
