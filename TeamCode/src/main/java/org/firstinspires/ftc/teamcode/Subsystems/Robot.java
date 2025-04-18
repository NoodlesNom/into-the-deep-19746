package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.util.BotLog;

import java.util.ArrayList;
import java.util.List;

public class Robot {

    public Drive mDrive;
    public Intake mIntake;

    public Deposit mDeposit;
    public Lift mLift;
    //public Vision mVision;
    //public Hang mHang;

    private double V;
    private double maxA = 0.0;
    private double prevCurrentTimer = 0.0;
    private double infoCurrent = 15.0;
    private double infoDuration = 500;
    private double emergencyCurrent = 23.0;
    private double emergencyDuration = 750;
    private double oneLoopShutoff = 26.0;
    private double currentRate = 200.0;
    private double lastCurrent = 0.0;


    private ElapsedTime currentTimer = new ElapsedTime();
    private ElapsedTime emergencyCurrentWatchdog = new ElapsedTime();
    private ElapsedTime infoCurrentWatchdog = new ElapsedTime();

    private boolean enableCurrentReporting = false;
    private static boolean usingComputer = false;
    
    private ArrayList<Subsystem> subsystems;

    //private ComputerDebugging computerDebugging;

    public List<LynxModule> allHubs;

    // For Auto
    public Robot(LinearOpMode opMode)
    {
        subsystems = new ArrayList<>();

        mDrive = new Drive(opMode.hardwareMap);
        mIntake = new Intake(opMode.hardwareMap);
        mDeposit = new Deposit(opMode.hardwareMap);
        mLift = new Lift(opMode.hardwareMap);
        //mHang = new Hang(opMode.hardwareMap);
        //mVision = new Vision(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        subsystems.add(mDrive);
        subsystems.add(mIntake);
        subsystems.add(mDeposit);
        subsystems.add(mLift);
        //subsystems.add(mHang);
        //subsystems.add(mVision);

//        if (usingComputer)
//        {
//            computerDebugging = new ComputerDebugging(true);
//        }
    }
    public Robot(LinearOpMode opMode, Pose2d start, HardwareMap map)

    {
        subsystems = new ArrayList<>();
        //mDrive = new Drive(opMode.hardwareMap);
        mIntake = new Intake(opMode.hardwareMap);
        mDeposit = new Deposit(opMode.hardwareMap);
        mLift = new Lift(opMode.hardwareMap);
        //mHang = new Hang(opMode.hardwareMap);
        //mVision = new Vision(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        //subsystems.add(mDrive);
        subsystems.add(mIntake);
        subsystems.add(mDeposit);
        subsystems.add(mLift);
    }

    // For Teleop (fixLift use)
    public Robot(OpMode opMode, boolean addLift)
    {
        subsystems = new ArrayList<>();

        mDrive = new Drive(opMode.hardwareMap);
        mIntake = new Intake(opMode.hardwareMap);
        mDeposit = new Deposit(opMode.hardwareMap);
        mLift = new Lift(opMode.hardwareMap);
        //mHang = new Hang(opMode.hardwareMap);
        //mVision = new Vision(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        subsystems.add(mDrive);
        subsystems.add(mIntake);
        subsystems.add(mDeposit);
        //subsystems.add(mHang);
        //subsystems.add(mVision);
        if (addLift)
        {
            subsystems.add(mLift);
        }

//        if (usingComputer)
//        {
//            computerDebugging = new ComputerDebugging(true);
//        }
    }

    // For Teleop
    public Robot(OpMode opMode)
    {
        this(opMode, true);
    }

    public void autoInit()
    {
//        TwoDeadWheelLocalizer.test = false;
//        mDrive.drive.localizer.update();
        for (Subsystem subsystem : subsystems)
        {
            subsystem.autoInit();
        }
    }

    public void teleopInit()
    {
        for (Subsystem subsystem : subsystems)
        {
            subsystem.teleopInit();
        }
    }

    public double getHubsCurrent() {
        double total = 0;
        for (LynxModule hub : allHubs)
        {
            total += hub.getCurrent(CurrentUnit.AMPS);
        }
        return(total);
    }
    public double getHubsVoltage() {
        return(V);
    }

//    public double getMotorCurrent() {
//        return( mDrive.getMotorCurrent() + mIntake.getMotorCurrent() + mHang.getMotorCurrent() + mLift.getMotorCurrent());
//    }

    public void setEnableCurrentReporting(boolean enable)
    {
        enableCurrentReporting = enable;
        currentTimer.reset();
        emergencyCurrentWatchdog.reset();
        infoCurrentWatchdog.reset();
    }

    public void currentReporting()
    {
        // If enabled
        if (enableCurrentReporting)
        {
            // Get the time
            double now = currentTimer.milliseconds();

            // If the prev time and current time are inverted, reset everything
            if (prevCurrentTimer >= now)
            {
                prevCurrentTimer = now;
                emergencyCurrentWatchdog.reset();
                infoCurrentWatchdog.reset();
            }

            // Rate has expired?
            if ((now - prevCurrentTimer) >= currentRate)
            {
                // Get the current from the hubs
                double total = getHubsCurrent();

                // Output some debug if the voltage is below 9.5V
//                double volts = mDrive.voltage.getVoltage();
//                if(volts < 9.5) {
//                    BotLog.logD("VoltCur_Info", "T:%4.2f, V: %4.2f, I: %4.2f",
//                            currentTimer.seconds(),
//                            volts, total);
//                    // double liftBI = mLift.getBCurrent();
//                    // double liftFI = mLift.getFCurrent();
//                    // if(liftBI > 5.0 || liftFI > 5.0) {
//                    //    BotLog.logD("LiftCur_Info", "T:%4.2f, V: %4.2f, FI: %4.2f, BI: %4.2f, I: %4.2f",
//                    //            currentTimer.seconds(),
//                    //            volts, liftBI, liftFI, total);
//                    //}
//                }

                // See if we're below our threshold for info
                if (total < infoCurrent)
                {
                    infoCurrentWatchdog.reset();
                }
                else
                {
                    // If we're above threshold for long enough, report it
                    if ((infoCurrentWatchdog.milliseconds() > infoDuration) && ((now - prevCurrentTimer) < infoDuration))
                    {

                        BotLog.logD("Cur_Info", "%4.2f, %4.2f, %4.2f",
                                currentTimer.seconds(),
                                infoCurrentWatchdog.milliseconds(),
                                total);
                    }
                }

                // See if we're below our threshold for emergency
                if (total < emergencyCurrent)
                {
                    emergencyCurrentWatchdog.reset();
                    for (Subsystem system : subsystems)
                    {
                        system.shutdown = false;
                    }
                }
                else
                {
                    // If we're above threshold for long enough, report it
                    if ((emergencyCurrentWatchdog.milliseconds() > emergencyDuration) && ((now - prevCurrentTimer) < emergencyDuration))
                    {
                        for (Subsystem system : subsystems)
                        {
                            // TODO: Handle this in every subsystem's update loop
                            system.shutdown = true;
                        }

                        BotLog.logD("Cur_Emerg", "%4.2f, %4.2f, %4.2f",
                                currentTimer.seconds(),
                                emergencyCurrentWatchdog.milliseconds(),
                                total);
                    }
                }

                if (total > oneLoopShutoff)
                {
                    for (Subsystem system : subsystems)
                    {
                        // TODO: Handle this in every subsystem's update loop
                        system.shutdown = true;
                    }

                    BotLog.logD("Cur_Shutdown", "%4.2f", total);
                }

                prevCurrentTimer = now;
                lastCurrent = total;
            }
        }
    }

    public boolean isShutdown()
    {
        return (emergencyCurrentWatchdog.milliseconds() > emergencyDuration) || (lastCurrent > oneLoopShutoff);
    }

    public void update(double timestamp)
    {
//        for (LynxModule hub : allHubs)
//        {
//            hub.clearBulkCache();
//        }
        currentReporting();

        ElapsedTime updateTime = new ElapsedTime();
        boolean debug = false;
        for (Subsystem subsystem : subsystems)
        {
            if (debug) { updateTime.reset(); }

            subsystem.update(timestamp);

            if (debug) { BotLog.logD("UpdateTime :: ", "%4.0f (%s)", updateTime.milliseconds(), subsystem.getClass().getName()); }
        }
        double total = 0;
        for (LynxModule hub : allHubs)
        {
            total += hub.getInputVoltage(VoltageUnit.VOLTS);
        }
        V = total;


//        if (usingComputer)
//        {
//            if (!mDrive.isSimDoneWithTrajectory())
//            {
//                computerDebugging.sendRobotLocation("POSITION:X=" + mDrive.getSimXPos() + ",Y=" + mDrive.getSimYPos()+ ",ANGLE=" + mDrive.getSimHeadingDeg());
//                computerDebugging.markEndOfUpdate();
//            }
//            computerDebugging.sendRobotLocation("REALPOSITION:X=" + mDrive.getXPos() + ",Y=" + mDrive.getYPos()+ ",ANGLE=" + mDrive.getHeadingDeg());
//            computerDebugging.markEndOfUpdate();
//
//        }
    }

    public String getTelem(double seconds)
    {
        String output = "";
        for (Subsystem subsystem : subsystems)
        {
            output += subsystem.getTelem(seconds);
        }
//        for (Subsystem subsystem : subsystems)
//        {
//            output += subsystem.getDemands();
//        }

        boolean debug = true;
        if(debug)
        {
            double total = 0;
            String CStr = "  robot.A ::    ";
            String VStr = "  robot.V ::    ";
            for (LynxModule hub : allHubs)
            {
                double val = hub.getCurrent(CurrentUnit.AMPS);
                CStr += String.format("%2.1f A ", val);
                total += val;

                VStr += String.format("%2.1f V ", hub.getInputVoltage(VoltageUnit.VOLTS));
            }


            if (total> maxA){
                maxA =total;
            }
            if (total>0){
                output += CStr + String.format(" => %2.1f A ", total) + "\n";
                output+= String.format("%2.1f maxA ", maxA);
            }
            output += VStr + "\n";

        }

        return output;
    }

    public void stop()
    {
        for (Subsystem subsystem : subsystems)
        {
            subsystem.stop();
        }
    }

    public static boolean isUsingComputer()
    {
        return usingComputer;
    }



}