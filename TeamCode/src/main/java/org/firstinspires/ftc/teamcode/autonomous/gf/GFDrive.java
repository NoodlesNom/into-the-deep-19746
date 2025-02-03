package org.firstinspires.ftc.teamcode.autonomous.gf;

import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMath.inToCM;
import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement.followCurve;
import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement.goToPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.BotLog;

import java.util.ArrayList;
@Config
public class GFDrive extends Subsystem {

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private double lastTime = 0.0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean mIsDoneGF = false;
    private boolean debugLoopTime = false;
    private ArrayList<CurvePoint> currGFPath;

    private double followangle;
    private boolean constant = false;
    private ArrayList<Double> currGFPos = new ArrayList<Double>();
    public static boolean canmove = true;
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

        if (state == drivestate.GFPATH|| state==drivestate.GFPOS) {
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
    public boolean isDoneWithGF()
    {

        if (!(state == drivestate.GFPATH || state == drivestate.GFPOS))
        {
            return true;
        }

        return mIsDoneGF;
    }

    private void updateGF()
    {

        double xV = (inToCM(mPeriodicIO.currentvels.linearVel.y));
        double yV = (inToCM(mPeriodicIO.currentvels.linearVel.x));
        double rV = mPeriodicIO.angularVelocity.zRotationRate;

        GFMovement.updateRobotVars(inToCM(mPeriodicIO.currentpose.position.x), inToCM(mPeriodicIO.currentpose.position.y), mPeriodicIO.currentpose.heading.toDouble(), xV, yV, rV);
        //BotLog.logD("pos :: ", String.format("x :: %.2f, y :: %.2f, rad :: %.2f", inToCM(getXPos()), inToCM(getYPos()), getHeadingRad()));
        if (state == drivestate.GFPATH) {
            if (GFReversed) {
                mIsDoneGF = followCurve(currGFPath, followangle-Math.toRadians(180), constant);
            }else{
                mIsDoneGF = followCurve(currGFPath, followangle, constant);

            }
        }else if (state == drivestate.GFPOS) {
            mIsDoneGF = goToPosition(currGFPos.get(0), currGFPos.get(1),currGFPos.get(2),currGFPos.get(3),currGFPos.get(4));
        }
        double drive = GFMovement.getMovement_y();
        double strafe = GFMovement.getMovement_x();
        double turn = GFMovement.getMovement_rad();


        mPeriodicIO.lf_pwr = drive + strafe - turn;
        mPeriodicIO.lr_pwr = drive - strafe - turn;
        mPeriodicIO.rf_pwr = drive - strafe + turn;
        mPeriodicIO.rr_pwr = drive + strafe + turn;
        if (isDoneWithGF()){
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
    public String getDemands() {
        return "";
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        if (lastTime != timestamp)
        {
            localizer.updatePoseEstimate();
            mPeriodicIO.currentpose = localizer.pinpoint.getPositionRR();
            mPeriodicIO.currentvels = localizer.pinpoint.getVelocityRR();
            mPeriodicIO.angularVelocity = localizer.pinpoint.getRobotAngularVelocity(AngleUnit.RADIANS);
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

    public void setWantGFPath(ArrayList<CurvePoint> path, boolean rev, double angle, boolean followconstant)
    {
        state = drivestate.GFPATH;
        constant = followconstant;
        currGFPath = path;
        GFReversed = rev;
        followangle = angle;
        GFMovement.initForMove();
        GFMovement.initCurve();
        GFMovement.updateRobotVars(inToCM(mPeriodicIO.currentpose.position.x), inToCM(mPeriodicIO.currentpose.position.y), mPeriodicIO.currentpose.heading.toDouble(), 0, 0, 0);
    }
    public void setWantGFPos(double targetX, double targetY, double point_angle, double movement_speed, double point_speed)
    {
        currGFPos.clear();
        state = drivestate.GFPOS;
        GFMovement.initForMove();
        GFMovement.updateRobotVars(inToCM(mPeriodicIO.currentpose.position.x), inToCM(mPeriodicIO.currentpose.position.y), mPeriodicIO.currentpose.heading.toDouble(), 0, 0, 0);
        currGFPos.add(inToCM(targetX));
        currGFPos.add(inToCM(targetY));
        currGFPos.add(point_angle);
        currGFPos.add(movement_speed);
        currGFPos.add(point_speed);
    }

    public void setWantGFPath(Path path)
    {
        setWantGFPath(path.getPath(), path.getReversed(), path.getAngle(), path.getConstant());
    }
    @Override
    public void writePeriodicOutputs() {
        if (mPeriodicIO.lf_pwr != mPeriodicIO.last_lf_demand || mPeriodicIO.lr_pwr != mPeriodicIO.last_lr_demand || mPeriodicIO.rf_pwr != mPeriodicIO.last_lf_demand || mPeriodicIO.rr_pwr != mPeriodicIO.last_rr_demand)
        {
            double maxPower = Math.max(Math.abs(mPeriodicIO.lf_pwr), Math.max(Math.abs(mPeriodicIO.lr_pwr),Math.max(Math.abs(mPeriodicIO.rf_pwr),Math.abs(mPeriodicIO.rr_pwr))));

            if (maxPower > 1)
            {
                mPeriodicIO.lf_pwr/=maxPower;
                mPeriodicIO.lr_pwr/=maxPower;
                mPeriodicIO.rf_pwr/=maxPower;
                mPeriodicIO.rr_pwr/=maxPower;
            }
            if (canmove) {
                localizer.leftBack.setPower(mPeriodicIO.lr_pwr);
                localizer.rightBack.setPower(mPeriodicIO.rr_pwr * 1);
                localizer.leftFront.setPower(mPeriodicIO.lf_pwr * 1);
                localizer.rightFront.setPower(mPeriodicIO.rf_pwr * 1);
                mPeriodicIO.last_lf_demand = mPeriodicIO.lf_pwr;
                mPeriodicIO.last_lr_demand = mPeriodicIO.lr_pwr;
                mPeriodicIO.last_rf_demand = mPeriodicIO.rf_pwr;
                mPeriodicIO.last_rr_demand = mPeriodicIO.rr_pwr;
            }

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
        GFPATH,
        GFPOS;
    }
}
