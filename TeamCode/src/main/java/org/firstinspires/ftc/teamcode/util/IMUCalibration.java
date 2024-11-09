package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Disabled
@TeleOp(name = "IMU Calibration", group = "Tools")
public class IMUCalibration extends OpMode {

    private final BNO055IMU.Parameters parametersCH = new BNO055IMU.Parameters();
    private BNO055IMU imuCH;

    private final BNO055IMU.Parameters parametersEH = new BNO055IMU.Parameters();
    private BNO055IMU imuEH;

    private final String fileNameCH = "19746IMUCalibrationCH.json";
    private final String fileNameEH = "19746IMUCalibrationEH.json";

    @Override
    public void init() {
        imuCH = hardwareMap.get(BNO055IMU.class, "imuCH");
        parametersCH.mode = BNO055IMU.SensorMode.IMU;
        parametersCH.angleUnit = AngleUnit.RADIANS;
        parametersCH.accelUnit = AccelUnit.METERS_PERSEC_PERSEC;
        parametersCH.calibrationDataFile = null;

        imuCH.initialize(parametersCH);

        imuEH = hardwareMap.get(BNO055IMU.class, "imuEH");
        parametersEH.mode = BNO055IMU.SensorMode.IMU;
        parametersEH.angleUnit = AngleUnit.RADIANS;
        parametersEH.accelUnit = AccelUnit.METERS_PERSEC_PERSEC;
        parametersEH.calibrationDataFile = null;

        imuEH.initialize(parametersEH);

        telemetry.addData("", "Press the start button to begin calibration");
    }

    @Override
    public void loop() {
        if(!imuCH.isGyroCalibrated()) {
            telemetry.addData("Do this", "Leave the device still (CH gyro)\n"
                    + "n");
        } else if(!imuEH.isGyroCalibrated()) {
            telemetry.addData("Do this", "Leave the device still (EH gyro)\n"
                    + "n");
        } else if(!imuCH.isAccelerometerCalibrated()) {
            telemetry.addData("Do this", "Move the sensor in various positions. Start flat, rotating 45 degrees (CH Accel)\n");
        } else if(!imuEH.isAccelerometerCalibrated()) {
            telemetry.addData("Do this", "Move the sensor in various positions. Start flat, rotating 45 degrees (EH Accel)\n");
            //} else if(!imuCH.isMagnetometerCalibrated()) {
            //    telemetry.addData("Do this", "Move device in figure 8 pattern (CH Mag)\n");
            //} else if(!imuEH.isMagnetometerCalibrated()) {
            //    telemetry.addData("Do this", "Move device in figure 8 pattern (EH Mag)\n");
        } else {
            telemetry.addData("Do this", "DONE\n");
            finish();
            requestOpModeStop();
        }

        telemetry.addData("Total CH", imuCH.getCalibrationStatus());
        telemetry.addData("Total EH", imuEH.getCalibrationStatus());

        telemetry.addData("Accel Calibrated CH", imuCH.isAccelerometerCalibrated() ? "✔" : "");
        telemetry.addData("Accel Calibrated EH", imuEH.isAccelerometerCalibrated() ? "✔" : "");

        telemetry.addData("Gyro Calibrated CH", imuCH.isGyroCalibrated() ? "✔" : "");
        telemetry.addData("Gyro Calibrated EH", imuEH.isGyroCalibrated() ? "✔" : "");

        telemetry.addData("Mag Calibrated CH", imuCH.isMagnetometerCalibrated() ? "✔" : "NA");
        telemetry.addData("Mag Calibrated EH", imuEH.isMagnetometerCalibrated() ? "✔" : "NA");

        telemetry.addData("Sys Calibrated CH", imuCH.isSystemCalibrated() ? "✔" : "");
        telemetry.addData("Sys Calibrated EH", imuEH.isSystemCalibrated() ? "✔" : "");
    }

    private void finish() {
        BNO055IMU.CalibrationData calibrationDataCH = imuCH.readCalibrationData();
        BNO055IMU.CalibrationData calibrationDataEH = imuEH.readCalibrationData();
        File fileCH = AppUtil.getInstance().getSettingsFile(fileNameCH);
        File fileEH = AppUtil.getInstance().getSettingsFile(fileNameEH);
        AppUtil.getInstance().delete(fileCH);
        AppUtil.getInstance().delete(fileEH);
        ReadWriteFile.writeFile(fileCH, calibrationDataCH.serialize());
        ReadWriteFile.writeFile(fileEH, calibrationDataEH.serialize());
        telemetry.log().clear();
        telemetry.log().add("Saved to '%s'", fileNameCH);
        telemetry.log().add(calibrationDataCH.serialize());
        telemetry.log().add("Saved to '%s'", fileNameEH);
        telemetry.log().add(calibrationDataEH.serialize());
    }
}
