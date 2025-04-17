package org.firstinspires.ftc.teamcode.autonomous.pedroPathing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.BotLog;

public class DriveManualMotor extends Subsystem {

    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;
    private AnalogInput leftwinchencoder;
    private AnalogInput rightwinchencoder;
    private double lf_pwr=-2;
    private double lr_pwr=-2;
    private double rf_pwr=-2;
    private double rr_pwr=-2;
    private int pto_count = 100;

    public static boolean ptoEnabled = false;

    private CRServoImplEx winchL;
    private CRServoImplEx winchR;
    private ServoImplEx pto;

    private double winchTarget = 55;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;


    private double leftdelta;
    private double rightdelta;
    public double leftwinchadjusted = 0;

    public boolean ptoReady = false;

    private double winchLdemand = -2;

    private int pto_pos=-1;
    private int prev_pto_pos=-1;
    private double prevwinchLdemand = -2;
    private double winchRdemand = -2;
    private double prevwinchRdemand = -2;
    public double rightwinchadjusted = 0;

    private double leftwinchprevious = 0;
    private double rightwinchprevious = 0;

    private double leftwinchpos;
    private double rightwinchpos;

    public boolean hold = false;

    private double[] winchPos = new double[] {55, 750, 30};

    public enum HANG_POS
    {
        //Constants with values
        IDLE(0),
        UP(1),
        DOWN(2);
        //UPSAFTEY(3);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        HANG_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }

    private double[] ptoPos = new double[] {0.81, 0.38};

    public enum PTO_POS
    {
        //Constants with values
        DISENGAGED(0),
        ENGAGED(1);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        PTO_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }
    public DriveManualMotor(HardwareMap map) {
        leftwinchencoder = map.get(AnalogInput.class, "leftwinchencoder");
        rightwinchencoder = map.get(AnalogInput.class, "rightwinchencoder");
        winchL = map.get(CRServoImplEx.class, "hangL");
        winchR = map.get(CRServoImplEx.class, "hangR");
        pto = map.get(ServoImplEx.class, "PTO");
        winchL.setDirection(CRServoImplEx.Direction.REVERSE);
        leftFront = map.get(DcMotorEx.class, "fl");
        leftBack = map.get(DcMotorEx.class, "bl");
        rightBack = map.get(DcMotorEx.class, "br");
        rightFront = map.get(DcMotorEx.class, "fr");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    public DriveManualMotor(HardwareMap map, Pose2d initialPose) {

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
        disengagePto();
        setWinchPos(0);
    }

    @Override
    public void teleopInit() {
        disengagePto();

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
    }

    @Override
    public String getTelem(double time) {
        return "";
    }

    @Override
    public void readPeriodicInputs(double time) {
        pto_count++;
        // Read all the sensors
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
        winchTarget = pos;
    }

    public void engagePto(double vel){

        if (vel<2) {
            pto_pos = 1;
        }
        ptoEnabled=true;
    }

    public void disengagePto(){
        pto_pos = 0;
        ptoEnabled=false;

    }
    public void setDrivePower(double lf, double lr, double rf, double rr){
        lf_pwr = lf;
        lr_pwr = lr;
        rf_pwr = rf;
        rr_pwr = rr;
    }
    public void setWinchPos(int pos){
        winchTarget = winchPos[pos];
    }
    @Override
    public String getDemands(){
        boolean debug = true;
        String output = "";
        if( debug ) {
//            output += "   fl.amps  :: " + drive.mPeriodicIO.flcurrent + "\n";
//            output += "   fr.amps  :: " + drive.mPeriodicIO.frcurrent + "\n";
//            output += "   bl.amps  :: " + drive.mPeriodicIO.blcurrent + "\n";
//            output += "   br.amps  :: " + drive.mPeriodicIO.brcurrent + "\n";
        }
        return output;
    }
    @Override
    public void writePeriodicOutputs() {
        if (lf_pwr!=-2 && lr_pwr!=-2 && rf_pwr!=-2 && rr_pwr!=-2) {
            if (pto_count > 20) {
                if (ptoEnabled) {
                    ptoReady = true;
                    leftFront.setPower(lf_pwr * -100);
                    leftBack.setPower(lf_pwr * -100);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                } else {

                    leftFront.setPower(lf_pwr * 1);
                    leftBack.setPower(lr_pwr * 1);
                    rightBack.setPower(rr_pwr * 1);
                    rightFront.setPower(rf_pwr * 1);
                }
            } else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
        }

        if (Math.abs(winchLdemand-prevwinchLdemand)>0.05){
            prevwinchLdemand = winchLdemand;
            winchL.setPower(winchLdemand);
        }
        if (Math.abs(winchRdemand-prevwinchRdemand)>0.05){
            prevwinchRdemand = winchRdemand;
            winchR.setPower(winchRdemand);
        }
        if (pto_pos!=prev_pto_pos){
            prev_pto_pos = pto_pos;
            if (pto_pos!=0) {
                pto_count = 0;
                ptoReady= false;
            }
            pto.setPosition(ptoPos[pto_pos]);
        }

    }
}
