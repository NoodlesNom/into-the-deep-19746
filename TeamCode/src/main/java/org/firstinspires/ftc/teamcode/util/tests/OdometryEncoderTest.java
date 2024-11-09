package org.firstinspires.ftc.teamcode.util.tests;//package org.firstinspires.ftc.teamcode.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.buttons.StickyButton;
//import org.firstinspires.ftc.teamcode.Subsystems.BrainSTEMRobot;
//
//@Disabled
//@TeleOp
//public class OdometryEncoderTest extends LinearOpMode {
//
//    private double MOTOR_TICK_COUNT2;
//    private double MOTOR_TICK_COUNT3;
//    private StickyButton goStickyButton = new StickyButton();
//
//
//    @Override
//    public void runOpMode() {
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//
//       // robot.initTeleOp();
//        robot.lift.ResetOdoEncoders();
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            //Status to show if telemetry was initialized
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//        }
//
//
//        while (opModeIsActive()) {
//
//            goStickyButton.update(gamepad1.a);
//            if ( goStickyButton.getState()) {
//                robot.lift.ResetOdoEncoders();
//            }
//
//            MOTOR_TICK_COUNT2 = robot.lift.getEncoderTicks();
//            MOTOR_TICK_COUNT3 = robot.lift.get2EncoderTicks();
//
//            telemetry.addData("flLiftAndParallelOdometry", MOTOR_TICK_COUNT2);
//            telemetry.addData("brLiftAndPerpendicularEncoder", MOTOR_TICK_COUNT3);
//
//            telemetry.update();
//        }
//    }
//}
