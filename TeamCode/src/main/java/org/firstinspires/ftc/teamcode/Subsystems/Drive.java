package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrivePeriodic;
import org.firstinspires.ftc.teamcode.util.BotLog;

public class Drive extends Subsystem {

    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;

    public boolean hold = false;

    public MecanumDrivePeriodic drive;

    public Drive(HardwareMap map) {
        drive = new MecanumDrivePeriodic(map,new Pose2d(0,0,0));
    }

    public Drive(HardwareMap map, Pose2d initialPose) {
        drive = new MecanumDrivePeriodic(map, initialPose);
    }



//    @Override
//    public void autoInit() {
//        drive.initAuto();
//    }
//
//    @Override
//    public void teleopInit() {
//        drive.initTeleOp();
//    }

    @Override
    public void autoInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void update(double timestamp) {
        double startTime = 0.0;
        if (debugLoopTime)
        {
            startTime = loopTimer.milliseconds();
        }

        readPeriodicInputs(timestamp);




        writePeriodicOutputs();


        if (debugLoopTime)
        {
            BotLog.logD("lt_lift :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }

    }

    @Override
    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
        drive.updatePoseEstimate();
    }

    @Override
    public String getTelem(double time) {
        return "";
    }

    @Override
    public void readPeriodicInputs(double time) {
        // Read all the sensors

        drive.mPeriodicIO.estimate = drive.updatePoseEstimate();

        drive.mPeriodicIO.volts = drive.voltageSensor.getVoltage();

    }

    @Override
    public void writePeriodicOutputs() {
        drive.leftFront.setPower(drive.mPeriodicIO.lf_pwr*0.8979);
        drive.leftBack.setPower(drive.mPeriodicIO.lr_pwr);
        drive.rightBack.setPower(drive.mPeriodicIO.rr_pwr*0.8684);
        drive.rightFront.setPower(drive.mPeriodicIO.rf_pwr*0.8696);

    }
}
