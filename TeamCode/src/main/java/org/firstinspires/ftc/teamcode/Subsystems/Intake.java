package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.MiniPID;

public class Intake extends Subsystem {

    // Hardware
    private DcMotorEx intake;
    private DcMotorEx extendo;
    private Servo pivot;
    private ExtendoControlState mExtendoControlState;

    private double tgtTicks;
    private double pidTgtTicks;
    private double PIDTime = 0.025;
    //private double PIDTime = 0.05;    // "we need to look at the changes" -/@ coach todd
    private double nextPID;
    private int PIDCount = 0;
    private int PIDSkipCount = 0;
    private double prevPIDTime = 0.0;
    private MiniPID pid;
    private final double P = 0.0075 / 1 ;
    private final double I = 0.0015 / 1;
    private final double D = 0.045 / 1;
    private final double F = 0.0002;
    private double vF = F;
    public final double MAX_LIFT_PWR = 1;
    public final double MIN_LIFT_PWR = -0.6;
    private Servo gate;

    // Hardware states
    private PeriodicIO mPeriodicIO;

    private double[] extendoPos = new double[] {0,50,150, 700};

    public enum EXTEND_POS
    {
        //Constants with values
        STOWED(0),
        TRANSFER(1),
        GETOUT(2),
        INTAKING(3);

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
    private double[] pivotPos = new double[] {0, 30.0/355, 180.0/355};
    public enum PIVOT_POS
    {
        //Constants with values
        STOWED(0),
        INTAKING(1),
        LAUNCH(2);

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
    private double[] gatePos = new double[] {0, 30.0/355, 90.0/355};
    public enum GATE_POS
    {
        //Constants with values
        CLAMP(0),
        CATCH(1),
        OPEN(2);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        GATE_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;
    public enum ExtendoControlState
    {
        OPEN_LOOP,
        PID_CONTROL
    }

    public enum ExtendoState
    {
        CLOSE_ENOUGH,
        MOVING
    }

    public Intake(HardwareMap map)
    {
        mPeriodicIO = new PeriodicIO();

        // motors
        intake = map.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendo = map.get(DcMotorEx.class, "extendo");
        extendo.setDirection(DcMotorEx.Direction.REVERSE);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pid = new MiniPID(P, I, D, F);
        pid.reset();
        pid.setOutputLimits(MIN_LIFT_PWR, MAX_LIFT_PWR);
        pid.setOutputRampRate(1);
        vF = F;
        pid.setPID(P, I, D, vF);

        nextPID = 0;

        setExtendoOpenLoop(0);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setVelocityPIDFCoefficients(10 * Math.PI, 3 * Math.PI, 0, 3 * Math.PI);

        // servos
        pivot = map.get(Servo.class, "pivot");

        gate = map.get(Servo.class, "gate");

        setIntakeOpenLoop(0);
    }

    @Override
    public void autoInit()
    {
        setPivotPos(PIVOT_POS.STOWED.getVal());
    }

    @Override
    public void teleopInit()
    {

    }

    @Override
    public void update(double timestamp)
    {
        double startTime = 0.0;
        if (debugLoopTime)
        {
            startTime = loopTimer.milliseconds();
        }

        readPeriodicInputs(timestamp);
        switch (mExtendoControlState)
        {
            case OPEN_LOOP:
                break;
            case PID_CONTROL:
                updateExtendoPID(timestamp);
                break;
        }
        writePeriodicOutputs();

        if (debugLoopTime)
        {
            BotLog.logD("lt_intake :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }
    }

    @Override
    public void stop()
    {
        setIntakeOpenLoop(0);
        setExtendoOpenLoop(0);
    }

    public void setIntakeOpenLoop(double power)
    {
        mPeriodicIO.intake_demand = power;
    }

    public void setExtendoOpenLoop(double power)
    {
        if (mExtendoControlState != ExtendoControlState.OPEN_LOOP)
        {
            mExtendoControlState = ExtendoControlState.OPEN_LOOP;
        }

        mPeriodicIO.extendo_demand = power;
    }

    public  void setPivotPos(int pos)
    {
        mPeriodicIO.pivot_pos = pos;
        //BotLog.logD(" Intake 1 :: ", "Pos %d, %5.2f", pos, intakePos[mPeriodicIO.pivot_pos]  );
    }

    public  void setExtendoPos(int tgtPosArg, double timestamp)
    {
        tgtPosArg = Math.min(tgtPosArg,(extendoPos.length-1));
        double tgtPosArgTicks = extendoPos[tgtPosArg];

        mExtendoControlState = ExtendoControlState.PID_CONTROL;
        mPeriodicIO.extendo_pos = tgtPosArg;

        if (tgtPosArgTicks != tgtTicks)
        {
            tgtTicks = tgtPosArgTicks;

            PIDCount = 0;
            PIDSkipCount = 0;
            nextPID = timestamp;
            pid.reset();
            updateExtendoPID(timestamp);
        }
    }
    private void updateExtendoPID(double timestamp)
    {
        mExtendoControlState = ExtendoControlState.PID_CONTROL;

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
                double curPos = mPeriodicIO.lastExtendoTicks;

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

                if((tgtTicks < 10.0) && (mPeriodicIO.lastExtendoTicks < 10.0))
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
                mPeriodicIO.extendo_demand = power;

                prevPIDTime = timestamp;
            }
            else
            {
                mPeriodicIO.extendo_demand = 0.0;
            }
            nextPID = timestamp + PIDTime;
        }
        else
        {
            PIDSkipCount++;
        }
    }

    public  void setGatePos(int pos)
    {
        mPeriodicIO.gate_pos = pos;
        //BotLog.logD(" Intake 1 :: ", "Pos %d, %5.2f", pos, intakePos[mPeriodicIO.pivot_pos]  );
    }

    public double getPower()
    {
        return intake.getPower();
    }

    public double getMotorCurrent()
    {
        return mPeriodicIO.intake_current;
    }
    public Lift.LiftState getIntakeState()
    {
        if (closeEnough())
        {
            return Lift.LiftState.CLOSE_ENOUGH;
        }
        else
        {
            return Lift.LiftState.MOVING;
        }
    }
    public boolean closeEnough()
    {
        return Math.abs(tgtTicks - mPeriodicIO.lastExtendoTicks) <= 20 && Math.abs(mPeriodicIO.lastExtendoVel) < 100; // TODO: What is a resonable velocity
    }

    //public void setReadVelocity(boolean on) { readVelocity = on; }
    // With deadwheels, we are reclaiming the intake encoder so no more velocity

    public void readPeriodicInputs(double timestamp)
    {
        mPeriodicIO.intake_current = intake.getCurrent(CurrentUnit.AMPS);
        mPeriodicIO.prevLastExtendoTicks = mPeriodicIO.lastExtendoTicks;
        mPeriodicIO.prevLastExtendoVel = mPeriodicIO.lastExtendoVel;

        mPeriodicIO.lastExtendoTicks = extendo.getCurrentPosition();
        mPeriodicIO.lastExtendoVel = extendo.getVelocity();
    }

    public void writePeriodicOutputs()
    {
        if( mPeriodicIO.prevIntake_demand != mPeriodicIO.intake_demand) {

            intake.setPower(mPeriodicIO.intake_demand);
            //intake.setVelocity(mPeriodicIO.intake_demand);
            mPeriodicIO.prevIntake_demand = mPeriodicIO.intake_demand;
        }

        if(mPeriodicIO.prevPivot_pos != mPeriodicIO.pivot_pos) {
            pivot.setPosition(pivotPos[mPeriodicIO.pivot_pos]);
            mPeriodicIO.prevPivot_pos = mPeriodicIO.pivot_pos;
        }

        if(mPeriodicIO.prevGate_pos != mPeriodicIO.gate_pos) {
            gate.setPosition(gatePos[mPeriodicIO.gate_pos]);
            mPeriodicIO.prevGate_pos = mPeriodicIO.gate_pos;
        }
        if(mPeriodicIO.prevExtend_pos != mPeriodicIO.extendo_pos) {

            mPeriodicIO.prevExtend_pos = mPeriodicIO.extendo_pos;
        }

        if(mPeriodicIO.prevextendo_demand != mPeriodicIO.extendo_demand)
        {
            extendo.setPower(mPeriodicIO.extendo_demand);
            mPeriodicIO.prevextendo_demand = mPeriodicIO.extendo_demand;
        }
    }

    @Override
    public String getTelem(double time)
    {
        String output = "power :: " + mPeriodicIO.intake_demand + "\n";
        output += "intake pos :: " + mPeriodicIO.pivot_pos + "\n";
        output += "extend pos :: " + mPeriodicIO.extendo_pos + "\n";


        return output;
    }

    public static class PeriodicIO {
        public int pivot_pos;
        public int gate_pos;

        public double intake_current;
        public int extendo_pos;

        public double lastExtendoTicks;
        public double lastExtendoVel;

        public double prevLastExtendoTicks;
        public double prevLastExtendoVel;
        public double extendo_demand;
        public double prevextendo_demand;
        public double intake_demand;

        public int prevPivot_pos = -1;
        public int prevGate_pos = -1;
        public int prevExtend_pos = -1;
        public double prevIntake_demand = -2;

    }
}
