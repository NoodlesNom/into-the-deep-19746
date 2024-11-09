//package org.firstinspires.ftc.teamcode.util.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Subsystems.BrainSTEMRobot;
//
//@Disabled
//@TeleOp
//public class ServosTest extends LinearOpMode {
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
//    private boolean depositorFlipIn;
//    private boolean depositorFlipOut;
//    private boolean depGateOut;
//    private boolean depGateIn;
//    private boolean gateIn;
//    private boolean gateOut;
//    private boolean liftOut;
//    private boolean liftIn;
//
//
//    private void mapControls() {
//        depositorFlipIn = gamepad1.a;
//        depositorFlipOut = gamepad1.b;
//        depGateIn = gamepad1.x;
//        depGateOut = gamepad1.y;
//        gateOut = gamepad1.dpad_up;
//        gateIn = gamepad1.dpad_down;
//        liftIn = gamepad1.dpad_left;
//        liftOut = gamepad1.dpad_right;
//
//    }
//
//    @Override
//    public void runOpMode() {
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//
//        robot.initTeleOp();
//
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
//            if (depositorFlipOut) {
//                // PORTME robot.depositor.setFlipDepositorServoPosition(DepositorPosition.OUT);
//            }
//
//
//            if (depositorFlipIn) {
//                // PORTME robot.depositor.setFlipDepositorServoPosition(DepositorPosition.IN);
//            }
//
//            if (depGateIn) {
//                // PORTME robot.depositor.setDepositorGateServoPosition(DepositorPosition.IN);
//            }
//
//            if (depGateOut) {
//                // PORTME robot.depositor.setDepositorGateServoPosition(DepositorPosition.OUT);
//            }
//
//            if (liftIn) {
//                // PORTME robot.depositor.setExtendServoPosition(DepositorPosition.IN);
//            }
//
//            if (liftOut) {
//                // PORTME robot.depositor.setExtendServoPosition(DepositorPosition.OUT);
//            }
//
//            if (gateOut) {
//                // PORTME robot.collector.setGateServoPosition(CollectorPosition.IN);
//            }
//
//            if (gateIn) {
//                // PORTME robot.collector.setGateServoPosition(CollectorPosition.OUT);
//            }
//
//        }
//    }
//}
