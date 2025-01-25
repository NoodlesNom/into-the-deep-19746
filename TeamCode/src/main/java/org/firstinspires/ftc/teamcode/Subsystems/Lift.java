package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.MiniPID;
@Config
public class Lift extends Subsystem {

    // Hardware
    private DcMotorEx lift;

    // Control states
    private LiftControlState mLiftControlState;

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;

    // Hardware states
    public PeriodicIO mPeriodicIO;
    private double tgtTicks;
    private double pidTgtTicks;
    private double PIDTime = 0.025;
    //private double PIDTime = 0.05;    // "we need to look at the changes" -/@ coach todd
    private double nextPID;
    private int PIDCount = 0;
    private int PIDSkipCount = 0;
    private double prevPIDTime = 0.0;
    private MiniPID pid;

    // Measured: ±22" == 765
    // Motor: 145.1 / Rev
    // Spindle: 36mm * 3.1459 / 25.4 == 4.4526" / rev
    // Does it match?
    // 22 / 4.4526 = 4.94 rev total
    // 4.94 * 145.1 =  716
    // 765/22 = 34 ticks / inch

    // These are from PowerPlay (external encoder I think.)
    // private final double P = 0.00015;                 // proportional scaling
    // private final double I = 0.00003;                 // integral scaling
    // private final double D = 0.0009;                  // derivative scaling

    public static double P = 0.0075 / 1 ;
    public static double I = 0.0015 / 1;
    public static double D = 0.045 / 1;
    public static double F = 0;
    private double vF = F;
    public static double MAX_LIFT_PWR = 1;
    public static double MIN_LIFT_PWR = -1;

    // This are from Center Stage (no external encoder)
    //    private final double P = 0.0075 / 1.3;
    //    private final double I = 0.0015 / 1.3;
    //    private final double D = 0.045 / 1.3;
    //    private final double F = 0.0002631578947;
    //    public final double MAX_LIFT_PWR = 1.0;
    // Old Value public final double MIN_LIFT_PWR = -0.45;
    //    public final double MIN_LIFT_PWR = -0.25;

    // 80 ticks is one tier of the backboard.  These new heights are optimized  for teleop.
    // 758 is the hard max of the lift, but we should not use it as is puts strain on the string.
    // 720 gives us second set line bonus.
    // Old values private int[] liftPositions = new int[]{1, 300, 380, 460, 540, 620, 700, 720, 500, 758, 758, 758, 100, 60};
    //
    //                                      0  1    2    3    4    5    6    7    8    9    10   11   12   13  14   15   16,  17,  18,  19
    private int[] liftPositions = new int[]{1,130,220,270,760,1040,400,830, 330,330, 960};
    // private int[] liftPositions = new int[]{1, 300, 380, 475, 560, 635, 720, 758, 500, 758, 758, 758};
    //120 spec place normal, changed for mega
    public final double SAFE_HEIGHT = 200;

    public enum LIFT_POS
    {
        //Constants with values
        DOWN(0),
        SPECINTAKE(2),
        SPECIMEN_PLACE(1),
        SAMPLE(4),
        MAX(5),
        SPECANGLED(3),
        SPECCLEAR(6),
        SAMPLESAFE(7),
        TRANSFERPREP(8),
        TRANSFER(9),
        AUTOSAMPLE(10);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        LIFT_POS (int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }

    public enum LiftControlState
    {
        OPEN_LOOP,
        PID_CONTROL
    }

    public enum LiftState
    {
        CLOSE_ENOUGH,
        MOVING
    }

//    public enum LiftGateState
//    {
//        CLOSED,
//        MOVING,
//        OPEN
//    }



    public Lift(HardwareMap map)
    {
        mPeriodicIO = new PeriodicIO();

        lift = map.get(DcMotorEx.class, "lift");

        //gate = map.get(Servo.class, "gate");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new MiniPID(P, I, D, F);
        pid.reset();
        pid.setOutputLimits(MIN_LIFT_PWR, MAX_LIFT_PWR);
        pid.setOutputRampRate(0.35);
        vF = F;
        pid.setPID(P, I, D, vF);

        nextPID = 0;

        setOpenLoop(0);
    }

    @Override
    public void autoInit()
    {
        //zeroLift()
        //setGatePos(GATE_POS.CLOSED.getVal()); // closed

        pid.setOutputLimits(MIN_LIFT_PWR, MAX_LIFT_PWR);

    }

    @Override
    public void teleopInit()
    {

        pid.setOutputLimits(MIN_LIFT_PWR /*+ .05*/, MAX_LIFT_PWR);

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
        switch (mLiftControlState)
        {
            case OPEN_LOOP:
                break;
            case PID_CONTROL:
                updateLiftPID(timestamp);
                break;
        }
        writePeriodicOutputs();

        if (debugLoopTime)
        {
            BotLog.logD("lt_lift :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }
    }

    @Override
    public void stop()
    {
        setOpenLoop(0);
    }

    public void zeroLift()
    {
        setOpenLoop(-.4);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < 1)
        {
            // wait for 1 second
        }

        setOpenLoop(0);
        timer.reset();

        while (timer.seconds() < .25)
        {
            // wait for .25 second
        }

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public  void setTargetPos(int tgtPosArg, double timestamp)
    {
        tgtPosArg = Math.min(tgtPosArg,(liftPositions.length-1));
        double tgtPosArgTicks = liftPositions[tgtPosArg];

        mLiftControlState = LiftControlState.PID_CONTROL;
        mPeriodicIO.liftPos = tgtPosArg;

        if (tgtPosArgTicks != tgtTicks)
        {
            tgtTicks = tgtPosArgTicks;

            PIDCount = 0;
            PIDSkipCount = 0;
            nextPID = timestamp;
            pid.reset();
            updateLiftPID(timestamp);
        }
    }

    public void setTargetTicks(int tgtTicksArg, double timestamp)
    {
        mLiftControlState = LiftControlState.PID_CONTROL;

        if (tgtTicksArg != tgtTicks)
        {
            tgtTicks = tgtTicksArg;

            PIDCount = 0;
            PIDSkipCount = 0;
            nextPID = timestamp;
            pid.reset();
            updateLiftPID(timestamp);
        }
    }

    public  void setOpenLoop(double power)
    {
        if (mLiftControlState != LiftControlState.OPEN_LOOP)
        {
            mLiftControlState = LiftControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = power;
    }

    public  void rezero()
    {
        setOpenLoop(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public  void zerofinish(double time)
    {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateLiftPID(double timestamp)
    {
        mLiftControlState = LiftControlState.PID_CONTROL;

        // Time went backwards, or we had a gap larger than 25x expected
        // Try to re-sync on the next call
        if ((timestamp < prevPIDTime) ||  ((timestamp - prevPIDTime) > (PIDTime * 25.0)))
        {
            BotLog.logD("LIFTPID :: ", "Unexpected times %5.2f, %5.2f, %5.2f", timestamp, prevPIDTime, PIDTime);

            nextPID = timestamp + PIDTime;
            prevPIDTime = timestamp;

            return;
        }

        if (nextPID < timestamp)
        {
            if (tgtTicks > 0.0)
            {
                // Leave this here just in case we don't come through 'update()' for this path.
                readPeriodicInputs(timestamp);

                double power;
                double curPos = mPeriodicIO.lastReadTicks;

                pidTgtTicks = tgtTicks;
                PIDCount++;

                if (PIDCount > 1)
                {
                    power = pid.getOutput(curPos, pidTgtTicks, ((timestamp - prevPIDTime) / PIDTime));
                }
                else
                {
                    power = vF * pidTgtTicks;
                }

                if((tgtTicks < 15.0) && (mPeriodicIO.lastReadTicks < 15.0))
                {
                    PIDCount = 0;
                    PIDSkipCount = 0;
                    pid.reset();
                    power = 0.0;
                }

                // power = Range.clip(power, -MAX_LIFT_UP_PWR, MAX_LIFT_UP_PWR);
                // We want to avoid an oscillation around '0' so we will ignore power < +/2.5%
                if(Math.abs(power) < 0.025 )
                {
                    power = 0.0;
                }
                mPeriodicIO.demand = power;

                prevPIDTime = timestamp;
            }
            else
            {
                mPeriodicIO.demand = 0.0;
            }
            nextPID = timestamp + PIDTime;
        }
        else
        {
            PIDSkipCount++;
        }
    }


    public double getLiftCurrent()
    {
        return(mPeriodicIO.current);
    }

//    public LiftGateState getGateState()
//    {
//        // Timer is active or we've requested movement
//        if ((!GateCloseTimer.hasExpired() && mPeriodicIO.GatePos == GATE_POS.OPEN.getVal()) || (!GateOpenTimer.hasExpired() && mPeriodicIO.GatePos == GATE_POS.CLOSED.getVal()) || (mPeriodicIO.GatePos != mPeriodicIO.prevGatePos))
//        {
//            return LiftGateState.MOVING;
//        }
//        else
//        {
//            if (mPeriodicIO.GatePos == Lift.GATE_POS.CLOSED.getVal())
//            {
//                return LiftGateState.CLOSED;
//            }
//            else
//            {
//                return LiftGateState.OPEN;
//            }
//        }
//    }

    public int getLiftTargetPos() {
        return mPeriodicIO.liftPos;
    }

    public int getLiftTargetTicks() {
        return liftPositions[mPeriodicIO.liftPos];
    }

    public double getLiftTicks()
    {
        return mPeriodicIO.lastReadTicks;
    }

    public double getLiftVelocity() { return mPeriodicIO.lastReadVel; }

    public double getLiveLiftPosition()
    {
        return lift.getCurrentPosition();
    }
    public double getLiveLiftVelocity()
    {
        return lift.getVelocity();
    }

    public LiftState getLiftState()
    {
        if (closeEnough())
        {
            return LiftState.CLOSE_ENOUGH;
        }
        else
        {
            return LiftState.MOVING;
        }
    }

    public boolean closeEnough()
    {
        return Math.abs(tgtTicks - mPeriodicIO.lastReadTicks) <= 20 && Math.abs(mPeriodicIO.lastReadVel) < 200; // TODO: What is a resonable velocity
    }

    public int getNearestVerticalIndex(double pos)
    {
        for (int i = 0; i < liftPositions.length; i++)
        {
            if (liftPositions[i] >= pos)
            {
                return i;
            }
        }

        return LIFT_POS.SAMPLE.getVal();
    }

    public void readPeriodicInputs(double time)
    {
        mPeriodicIO.prevLastReadPos = mPeriodicIO.lastReadTicks;
        mPeriodicIO.prevLastReadVel = mPeriodicIO.lastReadVel;
        mPeriodicIO.current = lift.getCurrent(CurrentUnit.AMPS);
        mPeriodicIO.lastReadTicks = lift.getCurrentPosition();
        mPeriodicIO.lastReadVel = lift.getVelocity();
    }

    public void writePeriodicOutputs()
    {
        writeLiftOutputs();
    }

    public void writeLiftOutputs()
    {
        if(mPeriodicIO.prevDemand != mPeriodicIO.demand)
        {
            lift.setPower(mPeriodicIO.demand);
            mPeriodicIO.prevDemand = mPeriodicIO.demand;
        }
    }


    @Override
    public String getTelem(double time)
    {
        boolean debug = false;
        pid.logging = debug;
        String output = "";
        if( debug ) {
            output =  "   lift.pwr  :: " + mPeriodicIO.demand + "\n";
            output += "   lift.pos  :: " + mPeriodicIO.lastReadTicks + "\n";
            output += "   lift.tgt  :: " + tgtTicks + "\n";
            output += "   lift.ptgt :: " + pidTgtTicks + "\n";
            if(pid.logging)
            {
                output += "   lift.pid  :: " + pid.getTelem();
            }
        }

        return output;
    }

    public static class PeriodicIO {
        // INPUTS
        public double lastReadTicks;
        public double lastReadVel;

        // OUTPUTS
        public double demand;
        public int liftPos;
        public int GatePos;
        public int pivotPos;
        public int clawPos;
        public int armPos;

        public double current = 0;

        public double prevLastReadPos=-1;
        public double prevLastReadVel;

        public double prevDemand = -1;
    }
}
