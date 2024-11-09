package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.BotLog;

import java.util.concurrent.TimeUnit;

public class Intake extends Subsystem {

    // Hardware
    private DcMotorEx intake; // E3
    private Servo extend;
    private Servo iPivot; // C0

    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private boolean readVelocity;           //  0     1     2    3    4   5    6    7     8    9    10
    private double[] intakePos = new double[] {0, 135.0/355, 180.0/355};
    private double[] extendPos = new double[] {0, 60.0/300};

    public enum INTAKE_POS
    {
        //Constants with values
        STOWED(0),
        PARALLEL(1),
        INTAKING(2);

        //Instance variable
        private final int val;

        //Constructor to initialize the instance variable
        INTAKE_POS(int v)
        {
            val = v;
        }

        public int getVal()
        {
            return val;
        }
    }
    public enum EXTEND_POS
    {
        //Constants with values
        STOWED(0),
        INTAKING(1);

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

    public enum IntakePosState
    {
        STOWED,    // Current position request is STOWED or <500ms after change from STOWED requested
        DEPLOYED   // >500ms after leaving STOWED
    }
    public enum ExtendPosState
    {
        STOWED,    // Current position request is STOWED or <500ms after change from STOWED requested
        DEPLOYED   // >500ms after leaving STOWED
    }


    private Deadline IntakePosTimer = new Deadline(200, TimeUnit.MILLISECONDS);

    private Deadline ExtendPosTimer = new Deadline(200, TimeUnit.MILLISECONDS);

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;


    public Intake(HardwareMap map)
    {
        mPeriodicIO = new PeriodicIO();

        // motors
        intake = map.get(DcMotorEx.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extend = map.get(Servo.class, "intake");

        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setVelocityPIDFCoefficients(10 * Math.PI, 3 * Math.PI, 0, 3 * Math.PI);

        // servos
        iPivot = map.get(Servo.class, "iPivot");

        setOpenLoop(0, false);
    }

    @Override
    public void autoInit()
    {
        readVelocity = false;
        setBrakeMode(false);
        setIntakePos(INTAKE_POS.STOWED.getVal());
        setExtendPos(EXTEND_POS.STOWED.getVal());
    }

    @Override
    public void teleopInit()
    {
        readVelocity = false;
        setBrakeMode(false);
        setExtendPos(EXTEND_POS.STOWED.getVal());
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
        writePeriodicOutputs();

        if (debugLoopTime)
        {
            BotLog.logD("lt_intake :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }
    }

    @Override
    public void stop()
    {
        setOpenLoop(0, false);
    }

    public synchronized void setBrakeMode(boolean on)
    {
        if (mIsBrakeMode != on)
        {
            mIsBrakeMode = on;
            DcMotor.ZeroPowerBehavior mode = on ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

            intake.setZeroPowerBehavior(mode);
        }
    }

    public synchronized void setOpenLoop(double power, boolean brake)
    {
        setBrakeMode(brake);
        mPeriodicIO.intake_demand = power;
    }

    public synchronized void setIntakePos(int pos)
    {
        mPeriodicIO.intake_pos = pos;
        //BotLog.logD(" Intake 1 :: ", "Pos %d, %5.2f", pos, intakePos[mPeriodicIO.intake_pos]  );
    }

    public synchronized void setExtendPos(int pos)
    {
        mPeriodicIO.extend_pos = pos;
        //BotLog.logD(" Intake 1 :: ", "Pos %d, %5.2f", pos, intakePos[mPeriodicIO.intake_pos]  );
    }

    public IntakePosState getIntakePosState()
    {
        // Timer is active or we've requested movement
        if((!IntakePosTimer.hasExpired()) || (mPeriodicIO.intake_pos == INTAKE_POS.STOWED.getVal()))
        {
            return IntakePosState.STOWED;
        }
        else
        {
            return IntakePosState.DEPLOYED;
        }
    }

    public ExtendPosState getExtendPosState()
    {
        // Timer is active or we've requested movement
        if((!ExtendPosTimer.hasExpired()) || (mPeriodicIO.extend_pos == EXTEND_POS.STOWED.getVal()))
        {
            return ExtendPosState.STOWED;
        }
        else
        {
            return ExtendPosState.DEPLOYED;
        }
    }

    public double getVelocity()
    {
        return mPeriodicIO.intake_velocity;
    }

    public double getPower()
    {
        return intake.getPower();
    }

    public double getMotorCurrent()
    {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    //public void setReadVelocity(boolean on) { readVelocity = on; }
    // With deadwheels, we are reclaiming the intake encoder so no more velocity
    public void setReadVelocity(boolean on)
    {
        readVelocity = false;
    }

    public void readPeriodicInputs(double timestamp)
    {
        if (readVelocity)
        {
            mPeriodicIO.intake_velocity = intake.getVelocity();
        }
    }

    public void writePeriodicOutputs()
    {
        if( mPeriodicIO.prevIntake_demand != mPeriodicIO.intake_demand) {

            intake.setPower(mPeriodicIO.intake_demand);
            //intake.setVelocity(mPeriodicIO.intake_demand);
            mPeriodicIO.prevIntake_demand = mPeriodicIO.intake_demand;
        }

        if(mPeriodicIO.prevIntake_pos != mPeriodicIO.intake_pos) {
            iPivot.setPosition(intakePos[mPeriodicIO.intake_pos]);
            //BotLog.logD(" Intake 2 :: ", "Pos %d, %5.2f", mPeriodicIO.intake_pos, intakePos[mPeriodicIO.intake_pos]  );
            if (mPeriodicIO.prevIntake_pos == INTAKE_POS.STOWED.getVal())
            {
                // We are leaving the STOWED position, wait 500ms before deployed
                IntakePosTimer.reset();
            }
            mPeriodicIO.prevIntake_pos = mPeriodicIO.intake_pos;
        }
        if(mPeriodicIO.prevExtend_pos != mPeriodicIO.extend_pos) {
            extend.setPosition(extendPos[mPeriodicIO.extend_pos]);
            //BotLog.logD(" Intake 2 :: ", "Pos %d, %5.2f", mPeriodicIO.intake_pos, intakePos[mPeriodicIO.intake_pos]  );
            if (mPeriodicIO.prevExtend_pos == EXTEND_POS.STOWED.getVal())
            {
                // We are leaving the STOWED position, wait 500ms before deployed
                ExtendPosTimer.reset();
            }
            mPeriodicIO.prevExtend_pos = mPeriodicIO.extend_pos;
        }
    }

    @Override
    public String getTelem(double time)
    {
        String output = "power :: " + mPeriodicIO.intake_demand + "\n";
        output += "intake pos :: " + mPeriodicIO.intake_pos + "\n";
        output += "extend pos :: " + mPeriodicIO.extend_pos + "\n";


        return output;
    }

    public static class PeriodicIO {
        // INPUTS
        public double intake_velocity = -1; // velocity of the intake in ticks/sec

        // OUTPUTS
        public int intake_pos;
        public int extend_pos;
        public double intake_demand;

        public int prevIntake_pos = -1;
        public int prevExtend_pos = -1;
        public double prevIntake_demand = -1;

    }
}
