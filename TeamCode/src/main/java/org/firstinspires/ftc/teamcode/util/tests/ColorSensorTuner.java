package org.firstinspires.ftc.teamcode.util.tests;//package org.firstinspires.ftc.teamcode.tests;
//
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.buttons.StickyButton;
//import org.firstinspires.ftc.teamcode.components.BrainSTEMRobot;
////import org.firstinspires.ftc.teamcode.components.//color5;
//import org.firstinspires.ftc.teamcode.utils.BotLog;
//
//@TeleOp
//@Disabled
//public class ColorSensorTuner extends LinearOpMode {
//
//    private StickyButton sensorSampleButton = new StickyButton();
//    private boolean sensorSample = false;
//
//    //public //color5 //color5;
//
//    private ElapsedTime runtime = new ElapsedTime();
//    public BotLog logger = new BotLog();
//
//    private void mapControls(BrainSTEMRobot robot) {
//        sensorSampleButton.update(gamepad1.right_bumper);
//    }
//
//    @Override
//    public void runOpMode() {
//        PhotonCore.enable();
//
//        // Initialize a new robot object
//        BrainSTEMRobot robot = new BrainSTEMRobot(this);
//
//        //color5 = new //color5(hardwareMap);
//
//        robot.initTeleOp();
//        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        logger.LOGLEVEL = logger.LOGDEBUG;
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            //Status to show if telemetry was initialized
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//        }
//
//        double now=0.0;
//
//        runtime.reset();
//        while (opModeIsActive()) {
//            mapControls(robot);
//
//            /*
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
//            */
//
//            // If sampling is enabled
//            sensorSample = sensorSampleButton.getState() ? true : false;
//            if( sensorSample ) {
//                double myShift = 0.0;
//                //if(color5.measure()) {
//                //myShift = color5.shiftedAmount;
//                }
//
//                telemetry.addLine()
//                        .addData("r  ", "%.3f, %.3f, %.3f, %.3f, %.3f", //color5.colorsL2.red, //color5.colorsL1.red, //color5.colorsC.red, //color5.colorsR1.red, //color5.colorsR2.red)
//                        .addData("b  ", "%.3f, %.3f, %.3f, %.3f, %.3f", //color5.colorsL2.blue, //color5.colorsL1.blue, //color5.colorsC.blue, //color5.colorsR1.blue, //color5.colorsR2.blue)
//                        .addData("r/b", "%2.2f, %2.2f, %2.2f, %2.2f, %2.2f", //color5.colorsL2.red///color5.colorsL2.blue, //color5.colorsL1.red///color5.colorsL1.blue, //color5.colorsC.red///color5.colorsC.blue, //color5.colorsR1.red///color5.colorsR1.blue, //color5.colorsR2.red///color5.colorsR2.blue)
//                        .addData("b/r", "%2.2f, %2.2f, %2.2f, %2.2f, %2.2f", //color5.colorsL2.blue///color5.colorsL2.red, //color5.colorsL1.blue///color5.colorsL1.red, //color5.colorsC.blue///color5.colorsC.red, //color5.colorsR1.blue///color5.colorsR1.red, //color5.colorsR2.blue///color5.colorsR2.red)
//                        .addData("m  ", "%2.2f", myShift)
//                ;
//
//                telemetry.update();
//                logger.logD("//color5_Red  :", String.format("%.3f, %.3f, %.3f, %.3f, %.3f", //color5.colorsL2.red, //color5.colorsL1.red, //color5.colorsC.red, //color5.colorsR1.red, //color5.colorsR2.red));
//                logger.logD("//color5_Blu  :", String.format("%.3f, %.3f, %.3f, %.3f, %.3f", //color5.colorsL2.blue, //color5.colorsL1.blue, //color5.colorsC.blue, //color5.colorsR1.blue, //color5.colorsR2.blue));
//                logger.logD("//color5_RB   :", String.format("%2.2f, %2.2f, %2.2f, %2.2f, %2.2f", //color5.colorsL2.red///color5.colorsL2.blue, //color5.colorsL1.red///color5.colorsL1.blue, //color5.colorsC.red///color5.colorsC.blue, //color5.colorsR1.red///color5.colorsR1.blue, //color5.colorsR2.red///color5.colorsR2.blue));
//                logger.logD("//color5_BR   :", String.format("%2.2f, %2.2f, %2.2f, %2.2f, %2.2f", //color5.colorsL2.blue///color5.colorsL2.red, //color5.colorsL1.blue///color5.colorsL1.red, //color5.colorsC.blue///color5.colorsC.red, //color5.colorsR1.blue///color5.colorsR1.red, //color5.colorsR2.blue///color5.colorsR2.red));
//                logger.logD("//color5_isRed:", String.format("%s", //color5.isRed()?"true":"false"));
//                logger.logD("//color5_isBlu:", String.format("%s", //color5.isBlue()?"true":"false"));
//                logger.logD("//color5_isGra:", String.format("%s", //color5.isGray()?"true":"false"));
//                logger.logD("//color5_measu:", String.format("%2.2f", myShift));
//
//            }
//        }
//    }
//}
