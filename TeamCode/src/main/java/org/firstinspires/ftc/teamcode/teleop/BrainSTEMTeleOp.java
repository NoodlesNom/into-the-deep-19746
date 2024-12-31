package org.firstinspires.ftc.teamcode.teleop;//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.autonomous.enums.LeftFlipPosition;
//import org.firstinspires.ftc.teamcode.autonomous.cancellers.TimerCanceller;
//import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
//import org.firstinspires.ftc.teamcode.austonomous.enums.RightFlipPosition;
//import org.firstinspires.ftc.teamcode.buttons.StickyButton;
//import org.firstinspires.ftc.teamcode.buttons.ToggleButton;
//import org.firstinspires.ftc.teamcode.Subsystems.BrainSTEMRobot;
//import org.firstinspires.ftc.teamcode.Subsystems.Claw;
//import org.firstinspires.ftc.teamcode.Subsystems.old.Lift;
//import org.firstinspires.ftc.teamcode.utils.SmoothDrive;
//
//@Photon
//@TeleOp
//public class BrainSTEMTeleOp extends LinearOpMode {
//    private StickyButton autoStackModeStickyButton = new StickyButton();
//
//    private StickyButton heightIncStickyButton = new StickyButton();
//    private StickyButton heightDecStickyButton = new StickyButton();
//
//    private StickyButton clawIncStickyButton = new StickyButton();
//    private StickyButton clawDecStickyButton = new StickyButton();
//    private StickyButton coneFlipStickyButton = new StickyButton();
//    private StickyButton groundJunctionStickyButton = new StickyButton();
//    private StickyButton groundReplaceStickyButton = new StickyButton();
//    private StickyButton clrConeStackIncStickyButton = new StickyButton();
//    private StickyButton leftFlipConeStickyButton = new StickyButton();
//    private StickyButton rightFlipConeStickyButton = new StickyButton();
//    private StickyButton beaconBooleanStickyButton = new StickyButton();
//    private ToggleButton lowToHighPoleToggleButton = new ToggleButton();
//
//
//    private StickyButton zeroLiftStickyButton = new StickyButton();
//    ////////////
//    //DRIVER 1//
//    ////////////
//    /*
//        NEW DRIVER 1 CONTROLS
//        Left Stick X:
//        Left Stick Y: Drive left side
//        Left Stick Button:
//        Right Stick X:
//        Right Stick Y: Drive right side
//        Right Stick Button:
//        D-pad Up:
//        D-pad Left:
//        D-pad Down:
//        D-pad Right:
//        Start:
//        X:
//        B:
//        Y:
//        A:
//        Left Bumper:
//        Left Trigger:
//        Right Bumper:
//        Right Trigger:
//     */
//    // private double drive;
//    // private double turn;
//
//    private ElapsedTime runtime = new ElapsedTime();
//    private TimerCanceller groundReplaceCanceller = new TimerCanceller(300);
//    private TimerCanceller automatedTimerCanceller = new TimerCanceller(750);
//
//    private boolean moveLiftUp;
//    private boolean moveLiftDown;
//    private boolean beacon = false;
//    private int boolean2 = 0;
//    // private int lowToHighToggle;
//
//    private int clawToggleHits = 0;
//    private int coneResetToggleHits = 0;
//    private int groundJunctionToggleHits = 0;
//    private int groundReplaceToggleHits = 0;
//    private int rightFlipConeToggleHits = 0;
//    private int leftFlipConeToggleHits = 0;
//    private int beaconBooleanToggleHits = 0;
//
//
//
//
//    private int heightToggleHits = 2;
//
//    private void mapControls(BrainSTEMRobot robot) {
//        autoStackModeStickyButton.update(gamepad1.dpad_down);
//
//        zeroLiftStickyButton.update(gamepad1.dpad_up);
//        clrConeStackIncStickyButton.update(gamepad1.y);
//
//      //  lowToHighToggle = lowToHighPoleToggleButton.update(gamepad1.dpad_left);
//        groundJunctionStickyButton.update(gamepad1.dpad_left);
//        groundJunctionToggleHits += groundJunctionStickyButton.getState() ? 1 : 0;
//
//        groundReplaceStickyButton.update(gamepad1.b);
//        groundReplaceToggleHits += groundReplaceStickyButton.getState() ? 1 : 0;
//
//        rightFlipConeStickyButton.update(gamepad1.x);
//        rightFlipConeToggleHits += rightFlipConeStickyButton.getState() ? 1 : 0;
//
//        leftFlipConeStickyButton.update(gamepad1.touchpad);
//        leftFlipConeToggleHits += leftFlipConeStickyButton.getState() ? 1 : 0;
////
//        beaconBooleanStickyButton.update(gamepad1.a);
//        beaconBooleanToggleHits += beaconBooleanStickyButton.getState() ? 1 : 0;
//
//        coneFlipStickyButton.update(gamepad1.dpad_right);
//        coneResetToggleHits += coneFlipStickyButton.getState() ? 1 : 0;
//
//        if (robot.claw.isSafeToChangeClawGoal()) {
//            clawIncStickyButton.update(gamepad1.right_bumper);
//            clawToggleHits += clawIncStickyButton.getState() ? 1 : 0;
//            if (clawIncStickyButton.getState()){
//                coneResetToggleHits = 0;
//                groundJunctionToggleHits = 0;
//            }
//            clawDecStickyButton.update(gamepad1.left_bumper);
//            if (clawDecStickyButton.getState()) {
//                coneResetToggleHits = 0;
//                groundJunctionToggleHits = 0;
//                if (clawToggleHits == 1) {
//                    clawToggleHits = 4;
////                } else if(clawToggleHits == 4) {
////                    clawToggleHits = 2;
//                } else if(clawToggleHits == 5) {
//                    clawToggleHits = 5;
//                } else {
//                    clawToggleHits = Math.max(0, clawToggleHits - 1);
//                }
//            }
//        }
//
////        if( robot.claw.isSafeToChangeClawGoal() && ((robot.claw.getCurrentGoal() == Claw.Goal.RETURN_MID) || (robot.claw.getCurrentGoal() == Claw.Goal.COLLECT_MID))) {
//        moveLiftUp = gamepad1.right_trigger > 0.5;
//        moveLiftDown = gamepad1.left_trigger > 0.5;
////        }
//
////        if((robot.lift.getGoal() == Lift.Goal.DOWN) || ( robot.claw.isSafeToChangeClawGoal() && ((robot.claw.getCurrentGoal() == Claw.Goal.RETURN_MID) || (robot.claw.getCurrentGoal() == Claw.Goal.COLLECT_MID)))) {
//        heightIncStickyButton.update(gamepad1.right_stick_button);
//        heightToggleHits += heightIncStickyButton.getState() ? 1 : 0;
//
//        heightDecStickyButton.update(gamepad1.left_stick_button);
//        heightToggleHits -= heightDecStickyButton.getState() ? 1 : 0;
//        Range.clip(heightToggleHits, 0, 2);
////        }
//    }
//
//    @Override
//    public void runOpMode() {
//
//
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//
//        robot.initTeleOp();
//        robot.lift.MAX_LIFT_UP_PWR = 1.0;
//
//        // Hardcode some init here
//        robot.claw.initClaw();
//        robot.claw.setClawServoPosition(ClawPosition.OPEN);
//        robot.lift.zeroLift();
//        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // This is a 'better' way to do the smooth drive where we push everything into a class
//        SmoothDrive sd = new SmoothDrive(gamepad1, gamepad2);
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            //Status to show if telemetry was initialized
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//        }
//
//        long prevTime = 0;
//
//        runtime.reset();
//        while (opModeIsActive()) {
//            mapControls(robot);
//
//            if (zeroLiftStickyButton.getState()) {
////                robot.drive.setMotorPowers(0, 0, 0, 0);
//                robot.lift.zeroLift();
//            }
//
////            // This is a 'better' way to do the smooth drive where we push everything into a class
//            sd.update(Math.toDegrees(robot.drive.getRawExternalHeading()));
//            robot.drive.setMotorPowers(sd.l_f_motor_power, sd.l_b_motor_power, sd.r_b_motor_power, sd.r_f_motor_power);
//
//            double driveInput = -gamepad1.left_stick_y;
//            double strafeInput = -gamepad1.left_stick_x;
//            double turnInput = -gamepad1.right_stick_x;
//            robot.drive.setWeightedDrivePower(
//                    new Pose2d(
//                            driveInput,
//                            strafeInput,
//                            turnInput
//                    )
//            );
//
//            // Check for the auto Stack Mode button press and take action
//            if (autoStackModeStickyButton.getState()) {
//                robot.lift.autoStackModeButtonPress();
//            }
//
//            if (clrConeStackIncStickyButton.getState()) {
//                robot.lift.clearConeStack += 2200;
//            }
////            if (moveLiftUp) {
////                robot.lift.setGoal(Lift.Goal.UP);
////                //robot.claw.setCurrentGoal(Claw.Goal.GO_TO_DEPOSIT);
////                clawToggleHits = 2;
////
////                telemetry.addLine("Running Motor: Front Left");
////            } else if (moveLiftDown) {
////                robot.lift.setGoal(Lift.Goal.DOWN);
////                telemetry.addLine("Running a: Rear Left");
////            }
////
////            if (lowToHighToggle){
////                robot.claw.inLowToHighMode = true;
////            }
////            else {
////                robot.claw.inLowToHighMode = false;
////            }
//
//            switch (clawToggleHits % 6) {
//                case 1: {
//                    robot.claw.setCurrentGoal(Claw.Goal.COLLECT_25);
//                    break;
//                }
//                case 2: {
//                    robot.lift.setGoal(Lift.Goal.UP);
//                    robot.claw.setCurrentGoal(Claw.Goal.GO_TO_DEPOSIT);
//                    //automatedTimerCanceller.reset();
//                    break;
//                }
//                case 3: {
//                    robot.claw.setCurrentGoal(Claw.Goal.OPEN_LOOP);
//                    robot.claw.setClawServoPosition(ClawPosition.RELEASE);
//                    if (beacon) {
//                        clawToggleHits = 4;
//                    }
//                    break;
//                }
//                case 4: {
//                        robot.claw.setCurrentGoal(Claw.Goal.DEPOSIT_TELEOP);
//                        if (robot.claw.lastGoalCollect25) {
//                            clawToggleHits = 0;
//                        } else {
//                            clawToggleHits = 5 ;
//                            automatedTimerCanceller.reset();
//                        }
//                    break;
//                }
//                case 5: {
//                    if (automatedTimerCanceller.isConditionMet()) {
//                        robot.lift.setGoal(Lift.Goal.DOWN);
//                        clawToggleHits = 0;
//                    }
//                    break;
//                }
//                default: {
//                    break;
//                }
//            }
//            switch (coneResetToggleHits % 4) {
//                case 1:
//                    robot.claw.setCurrentGoal(Claw.Goal.FIX);
//                    break;
//                case 2:
//                    robot.claw.disableFlipServo();
//                    clawToggleHits = 0;
//                    break;
//                case 3:
//                    robot.claw.setCurrentGoal(Claw.Goal.FIX_RESET);
//                    coneResetToggleHits = 0;
//                    clawToggleHits = 0;
//                default:
//                    break;
//            }
//            switch (groundJunctionToggleHits % 3) {
//                case 1:
//                    robot.claw.setCurrentGoal(Claw.Goal.IDLE_GOAL);
//                    robot.claw.setClawServoPosition(ClawPosition.CLOSED);
//                    robot.lift.setHeight(Lift.Height.CONE_2);
//                    robot.lift.setGoal(Lift.Goal.UP);
//                    break;
//                case 2:
//                    robot.claw.setClawServoPosition(ClawPosition.OPEN);
//                    robot.lift.setGoal(Lift.Goal.DOWN);
//                    groundJunctionToggleHits = 0;
//                    break;
//                default:
//                    break;
//            }
//            switch (groundReplaceToggleHits % 5) {
//                case 1:
//                    robot.claw.setCurrentGoal(Claw.Goal.IDLE_GOAL);
//                    robot.claw.setClawServoPosition(ClawPosition.CLOSED);
//                    robot.lift.clearConeStack = 10000;
//                    //groundReplaceCanceller.reset();
//                 //   if (groundReplaceCanceller.isConditionMet()) {
//                   // }
//                    break;
//                case 2:
//                    robot.lift.setHeight(Lift.Height.CLR_CONE_STACK);
//                    robot.lift.setGoal(Lift.Goal.UP);
//                    break;
//                case 3:
//                    robot.claw.setClawServoPosition(ClawPosition.OPEN);
//                    break;
//                case 4:
//                    robot.lift.setGoal(Lift.Goal.DOWN);
//                    groundReplaceToggleHits = 0;
//                    break;
//                default:
//                    break;
//            }
//            switch (rightFlipConeToggleHits % 2) {
//                case 0:
//                    robot.claw.setRightConeFlipServoPosition(RightFlipPosition.STORED);
//                    break;
//                case 1:
//                    robot.claw.setRightConeFlipServoPosition(RightFlipPosition.FLIP);
//                    break;
//            }
//
//            switch (leftFlipConeToggleHits % 2) {
//                case 0:
//                    robot.claw.setLeftConeFlipServoPosition(LeftFlipPosition.STORED);
//                    break;
//                case 1:
//                    robot.claw.setLeftConeFlipServoPosition(LeftFlipPsosition.FLIP);
//                    break;
//            }
//
//            switch (beaconBooleanToggleHits % 2) {
//                case 0:
//                    beacon = true;
//                    robot.claw.clawReleaseBeaconPos = 0.27;
//                    break;
//                case 1:
//                    beacon = false;
//                    robot.claw.clawReleaseBeaconPos = 0.25;
//                    break;
//
//            }
//
//            if (groundJunctionToggleHits == 0 && groundReplaceToggleHits == 0) {
//                switch (heightToggleHits % 3) {
//                    case 0:
//                        robot.lift.setHeight(Lift.Height.LOW);
//                        break;
//                    case 1:
//                        robot.lift.setHeight(Lift.Height.MED);
//                        break;
//                    case 2:
//                        robot.lift.setHeight(Lift.Height.HIGH);
//                        break;
//                }
//            }
//
//
//            robot.claw.updateComponent();
//            robot.lift.updateComponent();
//
//            telemetry.addData("Loop Time", System.currentTimeMillis() - prevTime);
//      //  //    telemetry.addData("Current Right Front Lift Motor", robot.lift.getCurrentAverage());
//    //        telemetry.addData("Current Alert Right Front Lift Motor", robot.lift.getCurrentAlertAverage());
//       //     telemetry.addData("Current Right Front Lift Motor", robot.drive.getCurrentAverage());
//
//          //  telemetry.addData("All motor draw", robot.drive.getCurrent() + robot.lift.getCurrent());
////            telemetry.addData("Drive Motor Encoders", robot.drive.getEncoderPositions());
////            telemetry.addData("Drive Motor Powers", robot.drive.getMotorPowers());
//            telemetry.addData("clawToggleHits: ", clawToggleHits);
//            telemetry.addData("Claw Goal", robot.claw.getCurrentGoal());
//            telemetry.addData("rightfliphtis", rightFlipConeToggleHits);
//
//            telemetry.addData("leftflipihits", leftFlipConeToggleHits);
//
//            telemetry.addData("Lift Goal", robot.lift.getGoal());
//            telemetry.addData("Lift Mode", robot.lift.getHeight());
////            telemetry.addData("Down Position", robot.lift.getDownPosition());
//           telemetry.addData("Act Height: ", robot.lift.getLiftEncoderTicks());
//           telemetry.addData("Tgt Height: ", robot.lift.getTgtPos());
////            telemetry.addData("Lift pwr: ", robot.lift.pwr);
////            telemetry.addData("Min pwr: ", robot.lift.pid.getOutputMin());
////            telemetry.addData("Max pwr: ", robot.lift.pid.getOutputMax());
//        //    telemetry.addData("zero sticky: ", zeroLiftStickyButton.getState());
//            telemetry.addData("Prev Claw Goal Collect_25: ", robot.claw.getPrevGoalCollect25());
//            telemetry.addData("timedservo: ", robot.claw.timedRightFlipServo.getTgtPos());
//            telemetry.addData("timedservo3: ", robot.claw.timedLeftFlipServo.getTgtPos());
//            telemetry.update();
//
//            prevTime = System.currentTimeMillis();
//
//            /*
//            robot.logger.logD("Teleop",String.format(" Time: %.3f, DSC2:%.3f, CG: %s, LM: %s, LG:%s, AH: %.0f TH: %.0f LP: %.2f V:%.2f, Min: %.2f Max: %.2f",
//                    runtime.seconds(),
//                    robot.claw.disableServoCanceller2.timeRemaining(),
//                    robot.claw.getCurrentGoal(),
//                    robot.lift.getHeight(),
//                    robot.lift.getGoal(),
//                    robot.lift.getLiftEncoderTicks(),
//                    robot.lift.getTgtPos(),
//                    robot.lift.pwr,
//                    robot.drive.batteryVoltageSensor.getVoltage(),
//                    robot.lift.pid.getOutputMin(),
//                    robot.lift.pid.getOutputMax()));
//
//             */
//        }
//    }
//}
