package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.TimedProfiledServo;
import org.firstinspires.ftc.teamcode.util.TimedServo;

import java.util.concurrent.TimeUnit;

public class Deposit extends Subsystem {

    //private Servo gate;

    private int servoTime= 1;
    private ServoImplEx pivotL;
    private TimedProfiledServo pivotLTimed;

    private RevBlinkinLedDriver led;
    private ServoImplEx pivotR;
    private TimedProfiledServo pivotRTimed;



    private ServoImplEx claw;

    //70 for spec intake
    //-85 for spec angled place
    //-65 for spec head on place
    //20 roll for spec angled place
    //-20 roll for spec head on place
    //-40 for sample place
    //90 roll for sample place

    private ServoImplEx diffyL;

    private ServoImplEx diffyR;
    // Control states


    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;

    // Hardware states
    public PeriodicIO mPeriodicIO;


    // private double[] pivotPositions = new double[]{.46, .78}; // TELEOP PIVOTS
    // private double[] pivotPositions = new double[]{.47, .78, .52}; // Regionals Gobilda Pivot
    //0.467 straight up
    private double[] pivotPositions = new double[]{0.915,0.67,0.4, 0, 0.467, 0.73, 0.775, 0.8, 0.284, 0.204,0.25,0.82,0.590}; // Double Wide Axon Pivot

    public enum PIVOT_POS
    {
        //Constants with values

        TRANSFER(0),
        SPEC(1),
        SAMPLE(2),
        SPECINTAKE(3),
        IDLE(4),
        SPECANGLED(5),
        TRANSFERPREPARE(6),
        TRANSFERPREPARE2(7),
        AUTOSPEC(8),
        AUTOSPECANGLED(9),
        AUTOSAMPLE(10),
        AUTOINIT(11),
        AUTOEND(12);


        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        PIVOT_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }

    private double[] clawPositions = new double[]{0.04, 0.3, 0.04, 0.1};

    public enum CLAW_POS
    {
        //Constants with values
        CLOSED(1),
        OPEN(0),
        TRANSFER(2),
        AUTOOPEN(3);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        CLAW_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }

    private Deadline PivotTimer = new Deadline(550, TimeUnit.MILLISECONDS);
    private Deadline ClawTimer = new Deadline(150, TimeUnit.MILLISECONDS);

//    private Deadline GateCloseTimer = new Deadline(150, TimeUnit.MILLISECONDS);
//    private Deadline GateOpenTimer = new Deadline(125, TimeUnit.MILLISECONDS);



//    public enum LiftGateState
//    {
//        CLOSED,
//        MOVING,
//        OPEN
//    }


    public enum PivotState
    {
        MOVING,
        READY
    }

//    public PivotState getPivotState()
//    {
//        if (true){
//            return
//        }
//    }



    public Deposit(HardwareMap map)
    {
        mPeriodicIO = new PeriodicIO();

        led = map.get(RevBlinkinLedDriver.class, "blinkin");

        pivotL = map.get(ServoImplEx.class, "pivotL");
        pivotL.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));

        pivotR = map.get(ServoImplEx.class, "pivotR");
        pivotR.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));
        pivotR.setDirection(ServoImplEx.Direction.REVERSE);

        pivotLTimed = new TimedProfiledServo(pivotL, new ElapsedTime());
        pivotLTimed.servo.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));

        pivotRTimed = new TimedProfiledServo(pivotR, new ElapsedTime());
        pivotRTimed.servo.setPwmRange(new PwmControl.PwmRange(500, PwmControl.PwmRange.usPulseUpperDefault));;

        claw = map.get(ServoImplEx.class, "claw");



        diffyL = map.get(ServoImplEx .class, "diffyL");
        diffyR = map.get(ServoImplEx .class, "diffyR");
        diffyR.setDirection(ServoImplEx.Direction.REVERSE);


    }

    @Override
    public void autoInit()
    {
        //zeroLift();
        setPivotPos(PIVOT_POS.AUTOINIT.getVal());

        setClawPos(1);

        setDiffyPos(-120,0);
        //setGatePos(GATE_POS.CLOSED.getVal()); // closed

    }

    @Override
    public void teleopInit()
    {
        setPivotPos(PIVOT_POS.IDLE.getVal());

        setClawPos(1);

        setDiffyPos(0,0);


    }

    @Override
    public void update(double timestamp)
    {
        double startTime = 0.0;
        if (debugLoopTime)
        {
            startTime = loopTimer.milliseconds();
        }

        // This is zero-time due to bulk reads.
        // We need to read inputs here for teleop fine-tune even if the PID is not in use.
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


    public void setPivotPos(int pos)
    {
        setPivotPos(pos, 1);
    }
    public void setPivotPos(int pos, int time)
    {
        setPivotPos(pos, time, new double[]{1});
    }
    public void setPivotPos(int pos, int time, double[] profile)
    {
        mPeriodicIO.pivotPos = pos;
        mPeriodicIO.profile = profile;
        setServoTime(time);
    }

    public void setLed(RevBlinkinLedDriver.BlinkinPattern pattern)
    {

        mPeriodicIO.ledstate = pattern;

    }
    public void setLiveLed(RevBlinkinLedDriver.BlinkinPattern pattern)
    {

        led.setPattern(pattern);

    }

    public void setClawPos(int pos)
    {
        mPeriodicIO.clawPos = pos;
    }

    public void setDiffyPos(int pitch, int roll)
    {
        mPeriodicIO.pitch = pitch;
        mPeriodicIO.roll = roll;
    }




    public void readPeriodicInputs(double time)
    {

    }

    public void writePeriodicOutputs()
    {
        writePivotOutputs();
        writeClawOutputs();
        writeDiffyOutputs();
        if (mPeriodicIO.ledstate!=mPeriodicIO.prevledstate){
            mPeriodicIO.prevledstate = mPeriodicIO.ledstate;
            led.setPattern(mPeriodicIO.ledstate);
        }
    }


    public void setServoTime(int speed){
        servoTime = speed;
    }

    public boolean servoDone(){
        return pivotRTimed.reachedTgt;
    }

    public int getPivotPos(){
        return mPeriodicIO.pivotPos;
    }
    public void writePivotOutputs()
    {
        if (mPeriodicIO.prevPivotPos != mPeriodicIO.pivotPos)
        {

            pivotRTimed.setProfile(mPeriodicIO.profile);
            pivotLTimed.setProfile(mPeriodicIO.profile);
            mPeriodicIO.prevprofile = mPeriodicIO.profile;

            pivotRTimed.setTimedPosition(pivotPositions[mPeriodicIO.pivotPos], servoTime);
            pivotLTimed.setTimedPosition(pivotPositions[mPeriodicIO.pivotPos]-0.02, servoTime);


            mPeriodicIO.prevPivotPos = mPeriodicIO.pivotPos;
        }

        pivotRTimed.update();
        pivotLTimed.update();
    }

    public void writeClawOutputs()
    {
        if (mPeriodicIO.prevClawPos != mPeriodicIO.clawPos)
        {

            claw.setPosition(clawPositions[mPeriodicIO.clawPos]);

            mPeriodicIO.prevClawPos = mPeriodicIO.clawPos;
        }
    }

    public void writeDiffyOutputs()
    {
        if (mPeriodicIO.prevPitch != mPeriodicIO.pitch|| mPeriodicIO.prevRoll != mPeriodicIO.roll)
        {
            diffyL.setPosition(0.5+(((mPeriodicIO.pitch+3)/340.0)+(mPeriodicIO.roll/320.0)));
            diffyR.setPosition(0.5+(((mPeriodicIO.pitch+3)/340.0)-(mPeriodicIO.roll/320.0)));
            mPeriodicIO.prevRoll = mPeriodicIO.roll;
            mPeriodicIO.prevPitch = mPeriodicIO.pitch;

        }
    }

    @Override
    public String getTelem(double time)
    {
        boolean debug = true;
        String output = "";
        if( debug ) {
            output =   " pivotL.tgt  :: " + pivotL.getPosition() + "\n";
            output +=  " pivotR.tgt  :: " + pivotR.getPosition() + "\n";
            output +=  "   claw.tgt  :: " +   claw.getPosition() + "\n";
            output +=  "  pitch.tgt  :: " + mPeriodicIO.pitch + "\n";
            output +=  "   roll.tgt  :: " + mPeriodicIO.roll + "\n";
            output +=  " diffyL.tgt  :: " + diffyL.getPosition() + "\n";
            output +=  " diffyR.tgt  :: " + diffyR.getPosition() + "\n";
        }
        return output;
    }

    public static class PeriodicIO {
        // INPUTS
        public RevBlinkinLedDriver.BlinkinPattern ledstate;
        public RevBlinkinLedDriver.BlinkinPattern prevledstate;
        public int pivotPos=-1;
        public double[] profile;
        public double[] prevprofile;
        public int clawPos=-1;
        public int prevPivotPos = -1;
        public int prevPitch = -200000;
        public int prevRoll = -200000;
        public int prevClawPos = -1;
        public int pitch = 0;
        public int roll = 0;
    }
}
