package org.firstinspires.ftc.teamcode.Subsystems.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.BotLog;


public class HangOld extends Subsystem {

    // Hardware
    private DcMotorEx screw; // RED C0

    private DcMotorEx winch; // RED C0

    // Control states
    private ScrewControlState mScrewControlState;

    private WinchControlState mWinchControlState;

    // Loop Time Tracker
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean debugLoopTime = false;

    // Hardware states
    private PeriodicIO mPeriodicIO;
    private double currPos = 0.0;

    // Motor: 1,425.1 / Rev
    // 1550 max


    public HangOld(HardwareMap map)
    {
        mPeriodicIO = new PeriodicIO();

        screw = map.get(DcMotorEx.class, "screw");
        winch = map.get(DcMotorEx.class, "winch");

        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winch.setDirection(DcMotor.Direction.FORWARD);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setScrewOpenLoop(0);
        setWinchOpenLoop(0);
        //BotLog.logD("Hang :: ", "Called the Hang constructor");
    }

    @Override
    public void autoInit()
    {
        // Assume we started the screw in the '0' position.
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BotLog.logD("Hang :: ", "Called the Hang Auto Init");
    }

    @Override
    public void teleopInit()
    {
        //BotLog.logD("Hang :: ", "Called the Hang Teleop Init");
        // no clue what we want here
        screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update(double timestamp)
    {
        double startTime = 0.0;
        if(debugLoopTime)
        {
            startTime = loopTimer.milliseconds();
        }

        readPeriodicInputs(timestamp);
        writePeriodicOutputs();

        if (debugLoopTime)
        {
            BotLog.logD("lt_hang :: ", String.format("%s", loopTimer.milliseconds() - startTime));
        }
    }

    @Override
    public void stop()
    {

        setScrewOpenLoop(0);
        setWinchOpenLoop(0);
    }

    public void zeroHang()
    {
        // We don't have a hard stop, no way to '0' the hang
        boolean hardStop = true;
        if (hardStop) {
            setScrewOpenLoop(-.2);
            //setWinchOpenLoop(-0.2);


            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            while (timer.seconds() < 0.5)
            {
                // wait for 1 second
            }

            setScrewOpenLoop(0);
            setWinchOpenLoop(0);
            timer.reset();

            while (timer.seconds() < .25)
            {
                // wait for .25 second
            }
        }

        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setScrewTargetPos(int targetPos)
    {

        mScrewControlState = ScrewControlState.TARGET_CONTROL;
        if (targetPos != mPeriodicIO.screw_target_position)
        {
            mPeriodicIO.screw_target_position = targetPos;
            mPeriodicIO.ScrewDemand = 1;
        }
    }
    public void setWinchTargetPos(int targetPos)
    {

        mWinchControlState = WinchControlState.TARGET_CONTROL;
        if (targetPos != mPeriodicIO.winch_target_position)
        {
            mPeriodicIO.winch_target_position = targetPos;
            mPeriodicIO.WinchDemand = 1;
        }
    }

    public void setScrewOpenLoop(double ScrewPower)
    {
        mScrewControlState = ScrewControlState.OPEN_LOOP;
        mPeriodicIO.ScrewDemand = ScrewPower;
    }
    
    public void setWinchOpenLoop(double WinchPower)
    {
        mWinchControlState = WinchControlState.OPEN_LOOP;
        mPeriodicIO.WinchDemand = WinchPower;
    }



    //time doesnt do anything for this
    public void readPeriodicInputs(double time)
    {
        mPeriodicIO.screw_position = screw.getCurrentPosition();
        currPos = mPeriodicIO.screw_position;
    }

    public void writePeriodicOutputs()
    {
        if (mScrewControlState == ScrewControlState.TARGET_CONTROL)
        {
            screw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(mPeriodicIO.prevTarget != mPeriodicIO.screw_target_position) {
                screw.setTargetPosition(mPeriodicIO.screw_target_position);
                mPeriodicIO.prevTarget = mPeriodicIO.screw_target_position;
                screw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
            if(mPeriodicIO.prevScrewDemand != mPeriodicIO.ScrewDemand) {
                screw.setPower(mPeriodicIO.ScrewDemand);
                mPeriodicIO.prevScrewDemand = mPeriodicIO.ScrewDemand;
            }

        }else{
            screw.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            if(mPeriodicIO.prevScrewDemand != mPeriodicIO.ScrewDemand) {
                screw.setPower(mPeriodicIO.ScrewDemand);
                mPeriodicIO.prevScrewDemand = mPeriodicIO.ScrewDemand;
            }
        }

        if (mWinchControlState == WinchControlState.TARGET_CONTROL)
        {
            winch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if(mPeriodicIO.prevTarget != mPeriodicIO.winch_target_position) {
                winch.setTargetPosition(mPeriodicIO.winch_target_position);
                mPeriodicIO.prevTarget = mPeriodicIO.winch_target_position;
                winch.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
            if(mPeriodicIO.prevWinchDemand != mPeriodicIO.WinchDemand) {
                winch.setPower(mPeriodicIO.WinchDemand);
                mPeriodicIO.prevWinchDemand = mPeriodicIO.WinchDemand;
            }

        }else{
            winch.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            if(mPeriodicIO.prevWinchDemand != mPeriodicIO.WinchDemand) {
                winch.setPower(mPeriodicIO.WinchDemand);
                mPeriodicIO.prevWinchDemand = mPeriodicIO.WinchDemand;
            }
        }
        //BotLog.logD("HangPwr", "HangPwr: %5.2f, position: %d, pidtgt: %d, tgt: %d", mPeriodicIO.demand*100.0, (int)screw.getCurrentPosition(), (int)pidTgtPos, (int)tgtPos  );
    }

    public double getMotorCurrent()
    {
        return(screw.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public String getTelem(double time)
    {
        boolean debug = true;
        String output = "\n";
        if( debug ) {
            boolean tasDebug = false;
            if( !tasDebug ) {
                output = "tgtPos :: " + mPeriodicIO.screw_target_position + "\n";
                output += "currentPos :: " + mPeriodicIO.screw_position + "\n";
                output += "Screw power :: " + mPeriodicIO.ScrewDemand + "\n";
                output += "Winch power :: " + mPeriodicIO.WinchDemand + "\n";
                output += "state :: " + mScrewControlState + "\n";
            } else {
                output =  "   tgt ::" + mPeriodicIO.screw_target_position + "\n";
                output += "curPos ::" + mPeriodicIO.screw_position + "\n";
                output += "Screw power :: " + mPeriodicIO.ScrewDemand + "\n";
                output += "Winch power :: " + mPeriodicIO.WinchDemand + "\n";
                output += " state ::" + mScrewControlState + "\n";
            }
        }

        return output;
    }

    public enum ScrewControlState
    {
        OPEN_LOOP,
        TARGET_CONTROL
    }

    public enum WinchControlState
    {
        OPEN_LOOP,
        TARGET_CONTROL
    }

    public static class PeriodicIO {
        // INPUTS
        public int screw_target_position;
        public int winch_target_position;
        public double screw_position;
        public int prevTarget = 0;

        // OUTPUTS
        public double ScrewDemand;
        public double WinchDemand;

        public double prevScrewDemand = -1;
        public double prevWinchDemand = -1;
    }
}
