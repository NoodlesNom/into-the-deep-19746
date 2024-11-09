//package org.firstinspires.ftc.teamcode.util.tests;
//
///* From: https://github.com/ftc16626/Skystone-2019/blob/9857dd777e9febf84febc6f95e32e2ea916e4ff0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tools/IMUCalibration.java */
//
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
//import org.firstinspires.ftc.teamcode.util.AxisDirection;
//import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
//import org.firstinspires.ftc.teamcode.util.utils.BotLog;
//
//@Disabled
//@TeleOp(name = "IMU Test", group = "Tools")
//public class IMUTest extends OpMode {
//
//    boolean doubleIMUs = true;
//
//    private final BNO055IMU.Parameters parametersCH = new BNO055IMU.Parameters();
//    private BNO055IMU imuCH;
//
//    private final BNO055IMU.Parameters parametersEH = new BNO055IMU.Parameters();
//    private BNO055IMU imuEH;
//
//    //private final String fileNameCH = "noCalCH.json";
//    //private final String fileNameEH = "noCalCH.json";
//    private final String fileNameCH = "19746IMUCalibrationCH.json";
//    private final String fileNameEH = "19746IMUCalibrationEH.json";
//
//    public BotLog logger = new BotLog();
//
//    public Orientation startupHeadingCH;
//    public Orientation startupHeadingEH;
//    public double startupQCH = 0.0;
//    public double startupQEH = 0.0;
//    public double prevQCH = 0.0 ;
//    public double prevQEH = 0.0 ;
//    public double intQCH = 0.0 ;
//    public double intQEH = 0.0 ;
//    public double instQCH = 0.0 ;
//    public double instQEH = 0.0 ;
//    public Orientation instHeadingCH = new Orientation();
//    public Orientation instHeadingEH = new Orientation();
//    public Orientation prevHeadingCH = new Orientation();
//    public Orientation prevHeadingEH = new Orientation();
//    public Orientation intCH = new Orientation();
//    public Orientation intEH = new Orientation();
//    NanoClock localClock = NanoClock.system();
//    double updateTime = 0.0;
//
//    @Override
//    public void init() {
//        logger.LOGLEVEL = logger.LOGDEBUG;
//
//        imuCH = hardwareMap.get(BNO055IMU.class, "imuCH");
//        parametersCH.mode = BNO055IMU.SensorMode.IMU;
//        parametersCH.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parametersCH.accelUnit = AccelUnit.METERS_PERSEC_PERSEC;
//        parametersCH.calibrationDataFile = fileNameCH;
//
//        imuCH.initialize(parametersCH);
//        BNO055IMUUtil.remapZAxis(imuCH, AxisDirection.POS_Z);
//        //BNO055IMUUtil.remapZAxis(imuCH, AxisDirection.NEG_X);
//
//        if(doubleIMUs) {
//            imuEH = hardwareMap.get(BNO055IMU.class, "imuEH");
//            parametersEH.mode = BNO055IMU.SensorMode.IMU;
//            parametersEH.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//            parametersEH.accelUnit = AccelUnit.METERS_PERSEC_PERSEC;
//            parametersEH.calibrationDataFile = fileNameEH;
//
//            imuEH.initialize(parametersEH);
//            BNO055IMUUtil.remapZAxis(imuEH, AxisDirection.POS_Z);
//            //BNO055IMUUtil.remapZAxis(imuEH, AxisDirection.POS_X);
//        }
//
//        telemetry.addData("", "Waiting for gyro to calibrate");
//
//        while(!imuCH.isGyroCalibrated()) {}
//        if(doubleIMUs) {
//            while(!imuEH.isGyroCalibrated()) {}
//        }
//
//        double now = localClock.seconds();
//        while (localClock.seconds() < now + 2.0) { }
//
//        startupHeadingCH = imuCH.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        prevHeadingCH = imuCH.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        instHeadingCH = imuCH.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        intCH.firstAngle = 0;
//        intCH.secondAngle = 0;
//        intCH.thirdAngle = 0;
//
//        startupQCH = getQHeading(imuCH);
//        prevQCH = startupQCH;
//        intQCH = 0.0;
//
//        if(doubleIMUs) {
//            startupHeadingEH = imuEH.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            prevHeadingEH = imuEH.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            instHeadingEH = imuEH.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            intEH.firstAngle = 0;
//            intEH.secondAngle = 0;
//            intEH.thirdAngle = 0;
//
//            startupQEH = -getQHeading(imuEH);
//            prevQEH = startupQEH;
//            intQEH = 0.0;
//        }
//
//        telemetry.addData("", "Press the start button to begin testing IMU");
//    }
//
//    @Override
//    public void loop() {
//
//        telemetry.addData("Total CH", imuCH.getCalibrationStatus());
//        if(doubleIMUs) {
//            telemetry.addData("Total EH", imuEH.getCalibrationStatus());
//        }
//
//        //getRawExternalHeading();
//        if (localClock.seconds() > updateTime){
//            updateTime = localClock.seconds()+0.0;
//        }
//        updateCH(updateTime);
//        if(doubleIMUs) {
//            updateEH(updateTime);
//        }
//        //getExternalHeadingVelocity();
//        //getQHeading(imuCH);
//        //getQHeading(imuEH);
//    }
//
//    public double getQHeading(BNO055IMU imu)
//    {
//        Quaternion q = imu.getQuaternionOrientation();
//        //Orientation gyroOrien = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
//        q = q.normalized();
//
//        // This code was leveraged from here:
//        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
//
//        // May not need the next 3 lines
//        //double t=q.x*q.y + q.z*q.w;
//        double h,a,b;
//        //double sqy = q.y * q.y;
//        //double b;
//        double sqx = q.x * q.x;
//        double sqz = q.z * q.z;
//
//        // We only need 'b' here since our bot is only intending to rotate in one dimension
//        // We don't need to worry about singularities for Quaternion to Euler conversion
//        // Because of a single dimension of turning.
//        // May not need h and a
//        //h = Math.atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
//        //a = Math.asin(2 * t);
//        b = Math.atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
//
//        // This code is a way to compare our Quaternion vs Euler gyro readings
//        //logger.logD("Quaternion",String.format(" getQHeading: t:%f, h/Y:%f(%f), a/Z:%f(%f), b/X:%f(%f)",
//        //             t,
//        //             -(h/Math.PI)*180,
//        //             AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.secondAngle),
//        //             -(a/Math.PI)*180,
//        //             AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle),
//        //             -(b/Math.PI)*180,
//        //             AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.thirdAngle)));
//
//        //logger.logD("Quaternion",String.format(" getQHeading: b:%f",
//        //             -(b/Math.PI)*180));
//
//        if(imu == imuCH) {
//            instQCH = AngleUnit.RADIANS.normalize(b - startupQCH);
//        } else {
//            instQEH = AngleUnit.RADIANS.normalize(-b - startupQEH);
//        }
//
//        return (b);
//    }
//
//    /*
//    public double update() {
//        Orientation CH = imuCH.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//        Orientation EH = imuEH.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//        double QCH = -getQHeading(imuCH);
//        double QEH = getQHeading(imuEH);
//        double QCHChange = 0;
//        double QEHChange = 0;
//
//
//
//        intCH.firstAngle += AngleUnit.DEGREES.normalize( CH.firstAngle - prevHeadingCH.firstAngle);
//        intCH.secondAngle += AngleUnit.DEGREES.normalize( CH.secondAngle - prevHeadingCH.secondAngle);
//        intCH.thirdAngle += AngleUnit.DEGREES.normalize( CH.thirdAngle - prevHeadingCH.thirdAngle);
//        prevHeadingCH = CH ;
//
//        intEH.firstAngle += AngleUnit.DEGREES.normalize( EH.firstAngle - prevHeadingEH.firstAngle);
//        intEH.secondAngle += AngleUnit.DEGREES.normalize( EH.secondAngle - prevHeadingEH.secondAngle);
//        intEH.thirdAngle += AngleUnit.DEGREES.normalize( EH.thirdAngle - prevHeadingEH.thirdAngle);
//        prevHeadingEH = EH ;
//
//        QCHChange = AngleUnit.RADIANS.normalize(QCH - prevQCH);
//        //if(QCHChange < 0) {
//        //    QCHChange *= (2160.0/2166.7);
//        //} else {
//        //    QCHChange *= (2160.0/2166.3);
//        //}
//        intQCH += QCHChange ;
//        prevQCH = QCH;
//
//        QEHChange = AngleUnit.RADIANS.normalize(QEH - prevQEH);
//        //if(QEHChange < 0) {
//        //    QEHChange *= (2160.0/2146.3);
//        //} else {
//        //    QEHChange *= (2160.0/2147.6);
//        //}
//        intQEH += QEHChange ;
//        prevQEH = QEH;
//
//        String logLine =String.format("  %s\n  %s\n  QCH:%3.1f, QEH:%3.1f\n  QEH:%3.1f\n ",
//                intCH,
//                intEH,
//                Math.toDegrees(intQCH),
//                Math.toDegrees(intQEH),
//                Math.toDegrees((intQEH+intQCH)/2.0));
//        logger.logD("Integrating:", logLine);
//        telemetry.addData("Integrating:", "\n" + logLine + "\n");
//
//        return 0;
//    }
//    */
//
//    public double updateCH(double updateTime) {
//        Orientation CH = imuCH.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//        instHeadingCH = CH;
//        double QCH = getQHeading(imuCH);
//        double QCHChange = 0;
//
//        instHeadingCH.firstAngle = AngleUnit.DEGREES.normalize( CH.firstAngle - startupHeadingCH.firstAngle);
//        instHeadingCH.secondAngle = AngleUnit.DEGREES.normalize( CH.secondAngle - startupHeadingCH.secondAngle);
//        instHeadingCH.thirdAngle = AngleUnit.DEGREES.normalize( CH.thirdAngle - startupHeadingCH.thirdAngle);
//
//        intCH.firstAngle += AngleUnit.DEGREES.normalize( CH.firstAngle - prevHeadingCH.firstAngle);
//        intCH.secondAngle += AngleUnit.DEGREES.normalize( CH.secondAngle - prevHeadingCH.secondAngle);
//        intCH.thirdAngle += AngleUnit.DEGREES.normalize( CH.thirdAngle - prevHeadingCH.thirdAngle);
//        prevHeadingCH = CH ;
//
//        QCHChange = AngleUnit.RADIANS.normalize(QCH - prevQCH);
//        //if(QCHChange < 0) {
//        //    QCHChange *= (2160.0/2166.7);
//        //} else {
//        //    QCHChange *= (2160.0/2166.3);
//        //}
//        intQCH += QCHChange ;
//        prevQCH = QCH;
//
//        if(localClock.seconds() > updateTime) {
//            String logLine = String.format("  %s\n  %s\n  QIntCH:%3.1f, QInstCH:%3.1f\n ",
//                    "int: " + intCH,
//                    "inst:" + instHeadingCH,
//                    Math.toDegrees(intQCH),
//                    Math.toDegrees(instQCH));
//            logger.logD("HeadingCH:", logLine);
//            telemetry.addData("HeadingCH:", "\n" + logLine);
//        }
//
//        return 0;
//    }
//
//    public double updateEH(double updateTime) {
//        Orientation EH = imuEH.getAngularOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//        instHeadingEH = EH;
//        double QEH = -getQHeading(imuEH);
//        double QEHChange = 0;
//
//        instHeadingEH.firstAngle = AngleUnit.DEGREES.normalize( EH.firstAngle - startupHeadingEH.firstAngle);
//        instHeadingEH.secondAngle = AngleUnit.DEGREES.normalize( EH.secondAngle - startupHeadingEH.secondAngle);
//        instHeadingEH.thirdAngle = AngleUnit.DEGREES.normalize( EH.thirdAngle - startupHeadingEH.thirdAngle);
//
//        intEH.firstAngle += AngleUnit.DEGREES.normalize( EH.firstAngle - prevHeadingEH.firstAngle);
//        intEH.secondAngle += AngleUnit.DEGREES.normalize( EH.secondAngle - prevHeadingEH.secondAngle);
//        intEH.thirdAngle += AngleUnit.DEGREES.normalize( EH.thirdAngle - prevHeadingEH.thirdAngle);
//        prevHeadingEH = EH ;
//
//        QEHChange = AngleUnit.RADIANS.normalize(QEH - prevQEH);
//        //if(QEHChange < 0) {
//        //    QEHChange *= (2160.0/2146.3);
//        //} else {
//        //    QEHChange *= (2160.0/2147.6);
//        //}
//        intQEH += QEHChange ;
//        prevQEH = QEH;
//
//        if(localClock.seconds() > updateTime) {
//            String logLine = String.format("  %s\n  %s\n  QIntEH:%3.1f, QInstEH:%3.1f\n ",
//                    "int: " + intEH,
//                    "inst: " + instHeadingEH,
//                    Math.toDegrees(intQEH),
//                    Math.toDegrees(instQEH));
//            logger.logD("HeadingEH:", logLine);
//            telemetry.addData("HeadingEH:", "\n" + logLine);
//        }
//
//        return 0;
//    }
//
//    /*
//    public double getRawExternalHeading() {
//        Orientation CH = imuCH.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
//        Orientation EH = CH ;
//        double QCH = AngleUnit.RADIANS.normalize( getQHeading(imuCH) - startupQCH);
//        double QEH = AngleUnit.RADIANS.normalize( getQHeading(imuEH) - startupQEH);
//        if(doubleIMUs) {
//            EH = imuEH.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.RADIANS);
//        }
//
//        String logLine =String.format(" %s\n %s\n QCH:%3.1f, QEH:%3.1f ",
//                CH.toString(),
//                EH.toString(),
//                Math.toDegrees(QCH),
//                Math.toDegrees(QEH));
//        logger.logD("HeadingLog:", logLine);
//        telemetry.addData("Heading:", logLine + "\n");
//
//        return 0;
//    }
//
//    public Double getExternalHeadingVelocity() {
//        double CH = (double) imuCH.getAngularVelocity().zRotationRate;
//        double rate = CH ;
//        double EH = 0.0;
//        if(doubleIMUs) {
//            EH = (double) imuEH.getAngularVelocity().zRotationRate;
//            rate = AngleUnit.RADIANS.normalize((CH + EH) / 2.0);
//        }
//
//        String logLine = String.format(" CH:%3.1f, EH:%3.1f, H:%3.1f, delta: %3.1f",
//                Math.toDegrees(CH),
//                Math.toDegrees(EH),
//                Math.toDegrees(rate),
//                Math.toDegrees(AngleUnit.RADIANS.normalize(CH-EH)));
//        logger.logD("HeadingRateLog:", logLine);
//        telemetry.addData("HeadingRate:", logLine + "\n");
//
//        return rate;
//    }
//    */
//}
