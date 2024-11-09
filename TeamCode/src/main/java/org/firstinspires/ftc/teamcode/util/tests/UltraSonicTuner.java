//package org.firstinspires.ftc.teamcode.util.tests;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.BrainSTEMRobot;
//import org.firstinspires.ftc.teamcode.Subsystems.MBUltraSonic;
//import org.firstinspires.ftc.teamcode.util.buttons.ToggleButton;
//import org.firstinspires.ftc.teamcode.util.utils.BotLog;
//@Photon
//@TeleOp
//@Disabled
//public class UltraSonicTuner extends LinearOpMode {
//
//    private ToggleButton sampleSensorToggleButtonRight = new ToggleButton();
//    private ToggleButton sampleSensorToggleButtonLeft = new ToggleButton();
//    private boolean sampleSensorRight = false;
//    private boolean sampleSensorLeft = false;
//    private ElapsedTime runtime = new ElapsedTime();
//    public BotLog logger = new BotLog();
//    public MBUltraSonic ultraLeft;
//    public MBUltraSonic ultraRight;
//    public MBUltraSonic ultra;
//
//    private void mapControls(BrainSTEMRobot robot) {
//        sampleSensorRight = sampleSensorToggleButtonRight.update(gamepad1.right_bumper);
//        sampleSensorLeft = sampleSensorToggleButtonLeft.update(gamepad1.left_bumper);
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
//        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        logger.LOGLEVEL = logger.LOGDEBUG;
//
//        ultraRight = new MBUltraSonic(this.hardwareMap, "V33Right", "UltraSRight");
//        ultraRight.initAuto();
//        ultraRight.enableUltra = true;
//
//        ultraLeft = new MBUltraSonic(this.hardwareMap, "V33Left", "UltraSLeft");
//        ultraLeft.initAuto();
//        ultraLeft.enableUltra = true;
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            //Status to show if telemetry was initialized
//            telemetry.addData("Status", "Initialized");
//            telemetry.update();
//        }
//
//        double prevTime = 0;
//        double nextUltra = 0.0;
//        double nextReset = 0.105;
//        double now=0.0,nowLog=0.0,deltaLog=0.0;
//        double[] s = new double[10];
//        int sIdx = 0;
//        double distance = 0.0;
//        double max = 0.0;
//
//        runtime.reset();
//        while (opModeIsActive()) {
//            now = runtime.seconds();
//            mapControls(robot);
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
//            if( sampleSensorRight ) {
//                ultra = ultraRight;
//            } else if (sampleSensorLeft ) {
//                ultra = ultraLeft;
//            }
//            // If sampling is enabled
//            if( sampleSensorRight || sampleSensorLeft ) {
//                // Check the time to see if it's time to sample (every 250ms)
//                if (now > nextUltra) {
//                    // Set the next time
//                    nextUltra = now + 0.0;
//
//                    if(!ultra.enableUltra) {
//                        ultra.reset();
//                        ultra.enableUltra=true;
//                    }
//
//                    if(ultra.elapsed.seconds() > nextReset) {
//                        ultra.reset();
//                    }
//
//                    distance = ultra.captureDistance();
//
//                    // Update the max
//                    max = Math.max(max, distance);
//
//                    // If the max is what we are replacing, recompute max
//                    if (s[sIdx % s.length] == max) {
//                        // Replace element
//                        s[sIdx % s.length] = distance;
//
//                        // Start with a known value, then compute max
//                        max = distance;
//                        for (int i = 0; i < s.length; i++) {
//                            max = Math.max(max, s[i]);
//                        }
//                    } else {
//                        // Replace element
//                        s[sIdx % s.length] = distance;
//                    }
//
//                    // Increment index
//                    sIdx++;
//
//                    nowLog = runtime.seconds();
//                    logger.logD("UltraS:", String.format(" Name:%s, Time: %.4f, Loop: %.2f, PrevLog: %.3f, Vcc: %.4f, Vobs: %.4f, s: %.2f, max: %.2f, idx: %d, ultraMax: %.2f, ultraIdx: %d",
//                            ultra.name,
//                            nowLog,
//                            (now-prevTime)*1000.0,
//                            deltaLog,
//                            ultra.lastVcc,
//                            ultra.lastVobs,
//                            distance,
//                            max,
//                            sIdx,
//                            ultra.maxS,
//                            ultra.samplesSinceReset
//                            ));
//                    deltaLog = (runtime.seconds()-nowLog)*1000.0;
//                }
//            } else {
//                // Max should only be 0 after we erase
//                if(max != 0.0) {
//                    // Erase
//                    for (int i = 0; i < s.length; i++) {
//                        s[i] = 0;
//                    }
//                    max = 0.0;
//                }
//                if(ultra.enableUltra) {
//                    ultra.reset();
//                    ultra.enableUltra=false;
//                }
//            }
//            prevTime = now;
//
//            // telemetry.addData("Loop Time", (runtime.seconds() - prevTime) * 1000.0);
//            // telemetry.update();
//            // prevTime = runtime.seconds();
//        }
//    }
//}
