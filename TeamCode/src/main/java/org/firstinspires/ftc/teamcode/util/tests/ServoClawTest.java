package org.firstinspires.ftc.teamcode.util.tests;//package org.firstinspires.ftc.teamcode.tests;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.autonomous.enums.ClawPosition;
//import org.firstinspires.ftc.teamcode.buttons.StickyButton;
//import org.firstinspires.ftc.teamcode.Subsystems.BrainSTEMRobot;
//
//@Disabled
//@TeleOp
//public class ServoClawTest extends LinearOpMode {
//    private StickyButton backward = new StickyButton();
//    private StickyButton forward = new StickyButton();
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
//    private boolean collectorClawServoOut;
//    private boolean collectorClawServoIn;
//    private static double position = 0.2;
//
//
//    private void mapControls() {
//        collectorClawServoIn = gamepad1.a;
//        collectorClawServoOut = gamepad1.y;
//
//    }
//
//    @Override
//    public void runOpMode() {
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
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
//            if (collectorClawServoIn){
//                robot.claw.setClawServoPosition(ClawPosition.OPEN);
//                telemetry.addData("Status", "jssgsbsgbg");
//                telemetry.update();
//            }
//            if (collectorClawServoOut){
//                robot.claw.setClawServoPosition(ClawPosition.CLOSED);
//                telemetry.addData("Status", "jjjjjjjjjjj");
//                telemetry.update();
//            }
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
//        }
//    }
//}
