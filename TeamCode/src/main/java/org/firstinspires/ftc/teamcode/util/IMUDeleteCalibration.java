package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Disabled
@TeleOp(name = "IMU Delete Calibration", group = "Tools")
public class IMUDeleteCalibration extends OpMode {


    private final String fileNameCH = "19746IMUCalibrationCH.json";
    private final String fileNameEH = "19746IMUCalibrationEH.json";

    @Override
    public void init() {
        telemetry.addData("", "Press the start button to DELETE calibration");
        telemetry.addData("", "Press the start button to DELETE calibration");
        telemetry.addData("", "Press the start button to DELETE calibration");
        telemetry.addData("", "Press the start button to DELETE calibration");
        telemetry.addData("", "Press the start button to DELETE calibration");
    }

    @Override
    public void loop() {
        File fileCH = AppUtil.getInstance().getSettingsFile(fileNameCH);
        File fileEH = AppUtil.getInstance().getSettingsFile(fileNameEH);
        AppUtil.getInstance().delete(fileCH);
        AppUtil.getInstance().delete(fileEH);
        telemetry.addData("Calibration Deleted", "\n");
        requestOpModeStop();
    }
}
