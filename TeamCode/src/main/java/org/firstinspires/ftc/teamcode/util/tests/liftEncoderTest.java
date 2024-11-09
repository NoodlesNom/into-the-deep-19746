package org.firstinspires.ftc.teamcode.util.tests;//package org.firstinspires.ftc.teamcode.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Subsystems.BrainSTEMRobot;
//
//@TeleOp
//@Disabled
//public class liftEncoderTest extends LinearOpMode {
//
//    private double MOTOR_TICK_COUNT;
//    private double MOTOR_TICK_COUNT2;
//
//    private double MOTOR_TICK_COUNT3;
//
//
//    @Override
//    public void runOpMode() {
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//
//       // robot.initTeleOp();
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            //Status to show if telemetry was initialized
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//
//        }
//
//
//        while (opModeIsActive()) {
//
//             MOTOR_TICK_COUNT = robot.lift.getLiftEncoderTicks();
//            MOTOR_TICK_COUNT2 = robot.lift.getEncoderTicks();
//
//            MOTOR_TICK_COUNT3 = robot.lift.get2EncoderTicks();
//
//            telemetry.addData("Deposit Mode", MOTOR_TICK_COUNT);
//            telemetry.addData("Deposit 2", MOTOR_TICK_COUNT2);
//
//            telemetry.addData("Deposit 3", robot.drive.getWheelPositions());
//
//            telemetry.update();
//        }
//    }
//}
