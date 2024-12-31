package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.old.LiftOld;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.TimedServo;

import java.util.concurrent.TimeUnit;

public class Deposit extends Subsystem {

    //private Servo gate;

    private int servoTime= 250;
    private ServoImplEx pivotL;
    private ServoImplEx pivotR;
    private TimedServo pivotLTimed;

    private AnalogInput pivotencoder;


    private ServoImplEx claw;

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
    private double[] pivotPositions = new double[]{0.08,0.25 ,0.58, 0.92}; // Double Wide Axon Pivot

    public enum PIVOT_POS
    {
        //Constants with values

        TRANSFER(0),
        SPEC(1),
        SAMPLE(2),
        SPECINTAKE(3);


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

    private double[] clawPositions = new double[]{0, 60.0/180};

    public enum CLAW_POS
    {
        //Constants with values
        CLOSED(1),
        OPEN(0);

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

    public PivotState getPivotState()
    {
        if (Math.abs(mPeriodicIO.pivotencoderpos-mPeriodicIO.pivotPos*355)<7){
            return PivotState.READY;
        }else{
            return PivotState.MOVING;
        }
    }



    public Deposit(HardwareMap map)
    {
        mPeriodicIO = new PeriodicIO();

        pivotL = map.get(ServoImplEx.class, "pivotL");
        pivotL.setDirection(ServoImplEx.Direction.REVERSE);
        pivotLTimed = new TimedServo(pivotL, new ElapsedTime());

        pivotR = map.get(ServoImplEx.class, "pivotR");

        
        claw = map.get(ServoImplEx.class, "claw");
        claw.setDirection(ServoImplEx.Direction.REVERSE);


        diffyL = map.get(ServoImplEx .class, "diffyL");
        diffyR = map.get(ServoImplEx .class, "diffyR");
        diffyR.setDirection(ServoImplEx.Direction.REVERSE);

        pivotencoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");


    }

    @Override
    public void autoInit()
    {
        //zeroLift();
        setPivotPos(2);

        setClawPos(1);

        setDiffyPos(0,0);
        //setGatePos(GATE_POS.CLOSED.getVal()); // closed

    }

    @Override
    public void teleopInit()
    {
        //zeroLift();
        setPivotPos(PIVOT_POS.TRANSFER.getVal());

        setClawPos(1);
        setDiffyPos(0,0);



        //setGatePos(GATE_POS.OPEN.getVal()); // closed

        PivotTimer.reset();
        PivotTimer.cancel();

        ClawTimer.reset();
        ClawTimer.cancel();

        //GateCloseTimer.reset();
        //GateCloseTimer.cancel();


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
        mPeriodicIO.pivotPos = pos;
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
        mPeriodicIO.pivotencoderpos = (pivotencoder.getVoltage() / 3.3 * 360);
    }

    public void writePeriodicOutputs()
    {
        writePivotOutputs();
        writeClawOutputs();
        writeDiffyOutputs();
    }


    public void setServoTime(int speed){
        servoTime = speed;
    }

    public void writePivotOutputs()
    {
        if (mPeriodicIO.prevPivotPos != mPeriodicIO.pivotPos)
        {

            pivotLTimed.setTimedPosition(pivotPositions[mPeriodicIO.pivotPos], servoTime);
            pivotR.setPosition(pivotLTimed.servo.getPosition());

            mPeriodicIO.prevPivotPos = mPeriodicIO.pivotPos;
        }

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
            diffyL.setPosition(0.5+((mPeriodicIO.pitch/340.0)+(mPeriodicIO.roll/320.0)));
            diffyR.setPosition(0.5+((mPeriodicIO.pitch/340.0)-(mPeriodicIO.roll/320.0)));
            mPeriodicIO.prevRoll = mPeriodicIO.roll;
            mPeriodicIO.prevPitch = mPeriodicIO.pitch;

        }
    }

    @Override
    public String getTelem(double time)
    {
        boolean debug = true;
        String output = "\n";
        if( debug ) {
        }

        return output;
    }

    public static class PeriodicIO {
        // INPUTS
        public int pivotPos;
        public int clawPos;
        public int prevPivotPos = -1;
        public int prevPitch = -200000;
        public int prevRoll = -200000;
        public int prevClawPos = -1;

        public double pivotencoderpos;
        public int pitch = 0;
        public int roll = 0;
    }
}
