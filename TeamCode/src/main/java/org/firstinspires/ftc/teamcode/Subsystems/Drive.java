package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrivePeriodic;
import org.firstinspires.ftc.teamcode.util.BotLog;

public class Drive extends Subsystem {

    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;
    private AnalogInput leftwinchencoder;
    private AnalogInput rightwinchencoder;

    private CRServoImplEx winchL;
    private CRServoImplEx winchR;

    private double winchTarget = 55;


    private double leftdelta;
    private double rightdelta;
    public double leftwinchadjusted = 0;

    private double winchLdemand = -2;
    private double prevwinchLdemand = -2;
    private double winchRdemand = -2;
    private double prevwinchRdemand = -2;
    public double rightwinchadjusted = 0;

    private double leftwinchprevious = 0;
    private double rightwinchprevious = 0;

    private double leftwinchpos;
    private double rightwinchpos;

    public boolean hold = false;

    public MecanumDrivePeriodic drive;
    private double[] winchPos = new double[] {55, 700, -10, 740};

    public enum EXTEND_POS
    {
        //Constants with values
        IDLE(0),
        UP(1),
        DOWN(2),
        UPSAFTEY(3);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        EXTEND_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }
    public Drive(HardwareMap map) {
        drive = new MecanumDrivePeriodic(map,new Pose2d(0,0,0));
        leftwinchencoder = map.get(AnalogInput.class, "leftwinchencoder");
        rightwinchencoder = map.get(AnalogInput.class, "rightwinchencoder");
        winchL = map.get(CRServoImplEx.class, "hangL");
        winchR = map.get(CRServoImplEx.class, "hangR");
        winchL.setDirection(CRServoImplEx.Direction.REVERSE);
    }

    public Drive(HardwareMap map, Pose2d initialPose) {
        drive = new MecanumDrivePeriodic(map, initialPose);

        leftwinchencoder = map.get(AnalogInput.class, "leftwinchencoder");
        rightwinchencoder = map.get(AnalogInput.class, "rightwinchencoder");
        winchL = map.get(CRServoImplEx.class, "hangL");
        winchR = map.get(CRServoImplEx.class, "hangR");
        winchL.setDirection(CRServoImplEx.Direction.REVERSE);
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

        drive.mPeriodicIO.flcurrent = 0;//drive.leftFront.getCurrent(CurrentUnit.AMPS);
        drive.mPeriodicIO.frcurrent = 0;//drive.rightFront.getCurrent(CurrentUnit.AMPS);
        drive.mPeriodicIO.blcurrent = 0;//drive.leftBack.getCurrent(CurrentUnit.AMPS);
        drive.mPeriodicIO.brcurrent = 0;//drive.rightBack.getCurrent(CurrentUnit.AMPS);
        leftwinchpos = -(leftwinchencoder.getVoltage() / 3.3 * 360)+360;
        rightwinchpos = (rightwinchencoder.getVoltage() / 3.3 * 360);
        leftdelta = leftwinchpos - leftwinchprevious;

        if (leftdelta > 180) leftdelta -= 360;
        if (leftdelta < -180) leftdelta += 360;
        leftwinchadjusted += leftdelta;
        leftwinchprevious = leftwinchpos;

        rightdelta = rightwinchpos - rightwinchprevious;
        if (rightdelta > 180) rightdelta -= 360;
        if (rightdelta < -180) rightdelta += 360;
        rightwinchadjusted += rightdelta;
        rightwinchprevious = rightwinchpos;
        winchLdemand = Range.clip(-(winchTarget - leftwinchadjusted) / 150, -1, 1);
        winchRdemand = Range.clip(-(winchTarget - rightwinchadjusted) / 150, -1, 1);
    }

    public void setWinchPosTicks(double pos){
        winchTarget = Range.clip(pos, -20, 800);
    }
    public void setWinchPos(int pos){
        winchTarget = Range.clip(winchPos[pos], -20, 800);
    }
    @Override
    public String getDemands(){
        boolean debug = true;
        String output = "";
        if( debug ) {
            output =  "   fl.pwr  :: " + drive.mPeriodicIO.lf_pwr + "\n";
            output += "   fr.pwr  :: " + drive.mPeriodicIO.rf_pwr + "\n";
            output += "   bl.pwr  :: " + drive.mPeriodicIO.lr_pwr + "\n";
            output += "   br.pwr  :: " + drive.mPeriodicIO.rr_pwr + "\n";
//            output += "   fl.amps  :: " + drive.mPeriodicIO.flcurrent + "\n";
//            output += "   fr.amps  :: " + drive.mPeriodicIO.frcurrent + "\n";
//            output += "   bl.amps  :: " + drive.mPeriodicIO.blcurrent + "\n";
//            output += "   br.amps  :: " + drive.mPeriodicIO.brcurrent + "\n";
        }
        return output;
    }
    @Override
    public void writePeriodicOutputs() {

        drive.leftFront.setPower(drive.mPeriodicIO.lf_pwr*1);
        drive.leftBack.setPower(drive.mPeriodicIO.lr_pwr*1);
        drive.rightBack.setPower(drive.mPeriodicIO.rr_pwr*1);
        drive.rightFront.setPower(drive.mPeriodicIO.rf_pwr*1);

        if (Math.abs(winchLdemand-prevwinchLdemand)>0.05){
            prevwinchLdemand = winchLdemand;
            winchL.setPower(winchLdemand);
        }
        if (Math.abs(winchRdemand-prevwinchRdemand)>0.05){
            prevwinchRdemand = winchRdemand;
            winchR.setPower(winchRdemand);
        }

    }
}
