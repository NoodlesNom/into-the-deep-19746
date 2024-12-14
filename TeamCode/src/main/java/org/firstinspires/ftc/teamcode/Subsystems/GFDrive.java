package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMath.inToCM;
import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement.followCurve;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.autonomous.gf.CurvePoint;
import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;
import org.firstinspires.ftc.teamcode.autonomous.gf.Path;
import org.firstinspires.ftc.teamcode.autonomous.gf.Pose2dWithCurvature;
import org.firstinspires.ftc.teamcode.autonomous.gf.Rotation2d;
import org.firstinspires.ftc.teamcode.autonomous.gf.Twist2d;
import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrivePeriodic;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.BotLog;

import java.util.ArrayList;

public class GFDrive extends Subsystem {

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private double lastTime = 0.0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean mIsDoneGF = false;
    private boolean debugLoopTime = false;
    private ArrayList<CurvePoint> currGFPath;

    private boolean GFReversed;

    public boolean hold = false;

    private drivestate state = drivestate.OpenLoop;

    public PinpointDrive localizer;

    public GFDrive(HardwareMap map) {
        localizer = new PinpointDrive(map,mPeriodicIO.initialpose);
    }

    public GFDrive(HardwareMap map, Pose2d initialPose) {
        mPeriodicIO.initialpose=initialPose;
        localizer = new PinpointDrive(map, mPeriodicIO.initialpose);
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



    public void update(double timestamp)
    {
        double startTime = 0.0;
        if (debugLoopTime)
        {
            startTime = loopTimer.milliseconds();
        }

        readPeriodicInputs(timestamp);

        if (state == drivestate.GF) {
            updateGF();
        }

        writePeriodicOutputs();

        if (debugLoopTime)
        {
            BotLog.logD("lt_drive :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }
    }

    @Override
    public void stop() {
        localizer.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
    }

    private void updateGF()
    {

        double xV = inToCM(mPeriodicIO.currentvels.linearVel.y);
        double yV = inToCM(mPeriodicIO.currentvels.linearVel.x);
        double rV = mPeriodicIO.currentvels.angVel;

        GFMovement.updateRobotVars(inToCM(mPeriodicIO.currentpose.position.y), inToCM(mPeriodicIO.currentpose.position.x), mPeriodicIO.currentpose.heading.toDouble(), xV, yV, rV);
        //BotLog.logD("pos :: ", String.format("x :: %.2f, y :: %.2f, rad :: %.2f", inToCM(getXPos()), inToCM(getYPos()), getHeadingRad()));
        mIsDoneGF = followCurve(currGFPath, GFReversed ? Math.toRadians(-90) : Math.toRadians(90));

        double drive = GFMovement.getMovement_y();
        double strafe = GFMovement.getMovement_x();
        double turn = GFMovement.getMovement_rad();
        if (!mIsDoneGF)
        {
            mPeriodicIO.lf_pwr = drive + strafe - turn;
            mPeriodicIO.lr_pwr = drive - strafe - turn;
            mPeriodicIO.rf_pwr = drive - strafe + turn;
            mPeriodicIO.rr_pwr = drive + strafe + turn;
        }
        else
        {
            mPeriodicIO.lf_pwr = 0;
            mPeriodicIO.lr_pwr = 0;
            mPeriodicIO.rf_pwr = 0;
            mPeriodicIO.rr_pwr = 0;
        }

    }

    @Override
    public String getTelem(double time) {
        return "";
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        if (lastTime != timestamp)
        {
            localizer.updatePoseEstimate();
            mPeriodicIO.currentpose = localizer.pinpoint.getPositionRR();
            mPeriodicIO.currentvels = localizer.pinpoint.getVelocityRR();
            mPeriodicIO.angularVelocity = localizer.pinpoint.getRobotAngularVelocity(AngleUnit.DEGREES);
            lastTime = timestamp;
        }

    }

    public synchronized void setOpenLoop(double lf, double lr, double rf, double rr) {
        state = drivestate.OpenLoop;
        mPeriodicIO.lf_pwr = lf;
        mPeriodicIO.lr_pwr = lr;
        mPeriodicIO.rf_pwr = rf;
        mPeriodicIO.rr_pwr = rr;
    }

    public void setWantGFPath(ArrayList<CurvePoint> path, boolean rev)
    {
        state = drivestate.GF;

        currGFPath = path;
        GFReversed = rev;

        GFMovement.initForMove();
        GFMovement.initCurve();
        GFMovement.updateRobotVars(mPeriodicIO.currentpose.position.x, mPeriodicIO.currentpose.position.y, mPeriodicIO.currentpose.heading.toDouble(), 0, 0, 0);
    }
    public void setWantGFPath(Path path)
    {
        setWantGFPath(path.getPath(), path.getReversed());
    }
    @Override
    public void writePeriodicOutputs() {
        if (mPeriodicIO.lf_pwr != mPeriodicIO.last_lf_demand || mPeriodicIO.lr_pwr != mPeriodicIO.last_lr_demand || mPeriodicIO.rf_pwr != mPeriodicIO.last_lf_demand || mPeriodicIO.rr_pwr != mPeriodicIO.last_rr_demand)
        {
            localizer.leftBack.setPower(mPeriodicIO.lr_pwr);
            localizer.rightBack.setPower(mPeriodicIO.rr_pwr*0.8684);
            localizer.leftFront.setPower(mPeriodicIO.lf_pwr*0.8979);
            localizer.rightFront.setPower(mPeriodicIO.rf_pwr*0.8696);
            mPeriodicIO.last_lf_demand = mPeriodicIO.lf_pwr;
            mPeriodicIO.last_lr_demand = mPeriodicIO.lr_pwr;
            mPeriodicIO.last_rf_demand = mPeriodicIO.rf_pwr;
            mPeriodicIO.last_rr_demand = mPeriodicIO.rr_pwr;

        }

    }

    public static class PeriodicIO
    {
        // INPUTS
        public Pose2d initialpose = new Pose2d(0,0,0);
        public Pose2d prevpose = new Pose2d(0,0,0);
        public Pose2d currentpose = new Pose2d(0,0,0);
        public PoseVelocity2d currentvels = new PoseVelocity2d(new Vector2d(0,0),0);
        public AngularVelocity angularVelocity = new AngularVelocity();
        public org.firstinspires.ftc.teamcode.autonomous.gf.Pose2d error = org.firstinspires.ftc.teamcode.autonomous.gf.Pose2d.identity();

        // OUTPUTS
        public double last_lf_demand = -1; // stupid hack
        public double last_rf_demand = -1; // stupid hack v2
        public double last_lr_demand = -1; // stupid hack v3
        public double last_rr_demand = -1; // stupid hack v4
        public double lf_pwr=0;
        public double lr_pwr=0;
        public double rf_pwr=0;
        public double rr_pwr=0;
    }
    public enum drivestate
    {
        OpenLoop,
        GF;
    }
}
