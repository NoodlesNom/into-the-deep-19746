package org.firstinspires.ftc.teamcode.util.tests;//package org.firstinspires.ftc.teamcode.tests;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
//import org.firstinspires.ftc.teamcode.autonomous.enums.FlipPosition;
//import org.firstinspires.ftc.teamcode.buttons.StickyButton;
//import org.firstinspires.ftc.teamcode.buttons.ToggleButton;
//import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
//
//@TeleOp
//public class ServoArmTest extends LinearOpMode {
//    private StickyButton backward = new StickyButton();
//    private ToggleButton armToggle = new ToggleButton();
//
//
//
//    ////////////
//    //DRIVER 1//
//    ////////////
//    /*
//        NEW DRIVER 1 CONTROLS
//        Left Stick X:
//        Left Stick Y: Drive left side
//        Left Stick Button: Press left button once to enable collector flip servos and move it to exactly horizontal, press again to disable flip servos
//        Right Stick X:
//        Right Stick Y: Drive right side
//        Right Stick Button:
//        D-pad Up:
//        D-pad Down:
//        D-pad Left:
//        D-pad Right:
//        Start:
//        X: Press X once to close the gate servo, extend the depositor out, and deposit the element.
//        Press X again to close the deposit servo, retract depositor, and open gate servo.
//        B:
//        Y:
//        A: Press A once to put collector gate servo in and click A again to put it out.
//        Left Bumper: Reverse collector when holding
//        Left Trigger: Move lift down
//        Right Bumper: Toggle collector on/off
//        Right Trigger: Move lift up
//     */
//    private boolean collectorFlipServoDeposit;
//    private boolean collectorFlipServoCollect;
//    private boolean collectorFlipServoMid;
//    private boolean togglePwm;
//    private boolean collectorClawServoClosed;
//    private boolean collectorClawServoRelease;
//    private static double position = 0.2;
//
//
//    private void mapControls() {
//        togglePwm = armToggle.update(gamepad1.y);
//        collectorFlipServoCollect = gamepad1.a;
//        collectorFlipServoDeposit = gamepad1.b;
//        collectorFlipServoMid = gamepad1.x;
//    }
//
//    @Override
//    public void runOpMode() {
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//
//        FlipPosition prevState = FlipPosition.INIT;
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            //Status to show if telemetry was initialized
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//        }
//
//
//        while (opModeIsActive()) {
//            mapControls();
//
//            if (collectorFlipServoDeposit && (prevState != FlipPosition.DEPOSIT)){
//                robot.claw.setFlipServoPosition(FlipPosition.DEPOSIT);
//                prevState = FlipPosition.DEPOSIT ;
//                telemetry.addData("Status", "Deposit");
//            }
//            if (collectorFlipServoCollect && (prevState != FlipPosition.COLLECT)){
//                robot.claw.setFlipServoPosition(FlipPosition.COLLECT);
//                prevState = FlipPosition.COLLECT ;
//                telemetry.addData("Status", "Collect");
//            }
//            if (collectorFlipServoMid && (prevState != FlipPosition.MID)){
//                robot.claw.setFlipServoPosition(FlipPosition.MID);
//                prevState = FlipPosition.MID ;
//                telemetry.addData("Status", "Mid");
//            }
////            if (togglePwm) {
////                robot.claw.disableFlipServo();
////            }
////            else {
////                robot.claw.enableFlipServo();
////            }
//
//
//            robot.claw.updateComponent();
//            telemetry.addData("Left", robot.claw.getLeftClawPosition());
//            telemetry.addData("Right", robot.claw.getRightClawPosition());
//            telemetry.update();
////            robot.collector.disableFlipServos();
////
////            //If the x value of the left stick, the y value of the left stick, or the x value of
////            //the right stick is greater than the THRESHOLD value, move the robot
////
////            if (collectorFlipServoOut) {
////                robot.collector.setFlipServosPosition(CollectorPosition.OUT);
////                telemetry.addData("test", "testting");
////                telemetry.update();
////            }
////            if (collectorFlipServoIn) {
////                robot.collector.setFlipServosPosition(CollectorPosition.IN);
////                telemetry.addData("test", "testt");
////                telemetry.update();
////            }
////            if (collectorFlipServoMiddle) {
////                robot.collector.setFlipServosPosition(CollectorPosition.TILTED_UP);
////                telemetry.addData("test", "testt");
////                telemetry.update();
//            }
//        }
//    }
