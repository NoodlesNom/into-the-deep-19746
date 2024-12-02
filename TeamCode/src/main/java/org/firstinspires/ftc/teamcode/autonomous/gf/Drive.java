//package org.firstinspires.ftc.teamcode.autonomous.gf;
//
//import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getAngle_rad;
//import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getXPos;
//import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getYPos;
//import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMath.inToCM;
//import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement.followCurve;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.debugging.ComputerDebuggingNew;
//import org.firstinspires.ftc.teamcode.autonomous.gf.CurvePoint;
//import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Path;
//import org.firstinspires.ftc.teamcode.util.MBUltraSonic;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
//import org.firstinspires.ftc.robotcore.internal.system.Deadline;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.Kinematics;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.RobotState;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Pose2d;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Pose2dWithCurvature;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Rotation2d;
//import org.firstinspires.ftc.teamcode.autonomous.gf.Twist2d;
//import org.firstinspires.ftc.teamcode.lib.trajectory.TrajectoryIterator;
//import org.firstinspires.ftc.teamcode.lib.trajectory.timing.TimedState;
//import org.firstinspires.ftc.teamcode.lib.util.BotLog;
//import org.firstinspires.ftc.teamcode.lib.util.MiniFeedforward;
//import org.firstinspires.ftc.teamcode.lib.util.MiniPID;
//import org.firstinspires.ftc.teamcode.purepursuit.DriveMotionPlanner;
//
//import java.util.ArrayList;
//import java.util.Random;
//import java.util.concurrent.TimeUnit;
//
//
///*
//        To make auto more consistent if setup is an issue, look into 254 setHeading thing that they call in robotstate reset
// */
//public class Drive extends Subsystem
//{
//
//    // Hardware
//    // IMU-Related
//    private int IMUCount = 4;
//    private IMU[] imus = new IMU[IMUCount];
//    private double[] imuCorr = new double[IMUCount];
//    private int EXT_IMU = 1;
//    private int EXT2_IMU = 2;
//    private int CH_IMU = 0;
//    private int EXP_IMU = 3;
//    private int imuIdx = 0;
//
//    private Quaternion curQ;
//    private double curHeading = 0.0;
//    private double intHeading = 0.0;
//    private double prevHeading = 0.0;
//    private double prevDelta = 0.0;
//    private double prevIMUTime = 0.0;
//    private double prevIMUAngleVel = 0.0;
//    public boolean imuFatal = false;
//    public double encoder_heading = 0.0;
//    public boolean use_encoder_heading = false;
//
//    // If this is non-zero, it creates a chance for simulated IMU failure at a rate of 1/forceIMUFailover
//    // This should NEVER be non-zero in the git repo -- debug only!
//    private int forceIMUFailover = 0;
//    // These are only used for failover-related testing/logging
//    private Deadline IMULogTimer = new Deadline(500, TimeUnit.MILLISECONDS);
//    private double curHeading0 = 0.0;
//    private double intHeading0 = 0.0;
//    private double prevHeading0 = 0.0;
//    private double prevDelta0 = 0.0;
//
//    public VoltageSensor voltage;
//    public MBUltraSonic ultraRight;
//    public MBUltraSonic ultraLeft;
//
//    private DcMotorEx BL; // Blue with also slightly less bad cable E1
//    private DcMotorEx BR; // Red with bad cable E0
//    private DcMotorEx FL; // White C2
//    private DcMotorEx FR; // C3
//
//    private DcMotorEx liftF;
//    private DcMotorEx intake;
//
//    private Servo pixelLeft; // C0
//    private Servo pixelRight; // C1
//
//    // Control states
//    private DriveControlState mDriveControlState;
//    private RobotState robot_state_ = RobotState.getInstance();
//    private boolean errorLogged;
//    private boolean debugIMU = false;
//
//    // Loop Time Tracker
//    private boolean debugLoopTime = false;
//    private ElapsedTime loopTimer = new ElapsedTime();
//
//    // Hardware states
//    public PeriodicIO mPeriodicIO;
//    private boolean mIsBrakeMode;
//    private boolean mIsPositionMode;
//    private boolean mIsOnTarget = false;
//    private boolean mIsDoneGF = false;
//    private boolean mOverrideTrajectory = false;
//    private DriveMotionPlanner mMotionPlanner;
//    private DriveMotionPlanner mMotionSim;
//    private Pose2d lastPose;
//
//    private ArrayList<CurvePoint> currGFPath;
//    private boolean GFReversed;
//
//    // Physical stuff
//    private double left_encoder_prev_distance_ = 0.0;
//    private double right_encoder_prev_distance_ = 0.0;
//    private double left_encoder_prev_ticks_ = 0.0;
//    private double right_encoder_prev_ticks_ = 0.0;
//    public double[] headingOffset = new double[IMUCount];
//
//    public double RightPixelInit = 0.74;
//    public double RightPixelPush = 0.1;
//    public double LeftPixelInit = 0.25;
//    public double LeftPixelPush = 0.9;
//
//    private static double startingHeading = 0.0;
//    private Rotation2d mTargetHeading = new Rotation2d();
//    private double mTurnSpeed = 0.0;
//    private double mDecelRads = 0.0;
//
//    private MiniPID leftPFol;
//    private MiniPID rightPFol;
//    private ElapsedTime durTimerPFol;
//    private double leftTicksPFol, rightTicksPFol;
//    private double durationPFol;
//    private double prevTimePFol;
//    private double tgtTimePFol;
//    private double pidRatePFol = 10.0;
//
//    private double lastTime = 0.0;
//    private double lastSimTime = 0.0;
//
//    private double vScale = 1.0;
//    private double tgtV = 13.5;
//    private double targetGFVolts = 12.5; // 2+8 has to work on bricks
//    private double initVolt;
//
//    private Deadline ShutDownTimer = new Deadline(500, TimeUnit.MILLISECONDS);
//
//    boolean customPID = true;
//    private int EncHistorySize = 1;    // 1 will degenerate to use getVelocity(), 10 will provide ~300ms of depth
//    private double velDelta = 0.120;    // No history, just take the previous sample
//
//    private MiniFeedforward leftFF;
//    private MiniFeedforward rightFF;
//
//    public boolean twitchReduction = false;
//    private double[] RPIDTimestamp = new double[EncHistorySize];
//    private double[] RPIDEnc = new double[EncHistorySize];
//    private int RPIDEncCount = 0;
//    private double prevRPIDTime = 0.0;
//    private double RPIDTime = 0.010;
//    private double nextRPID = 0.0;
//
//    // private double RP=1/2000.0, RI=RP/100.0, RD=0, RF=(0.10/2500.0);
//    // private double RP = 1.0 / 1600.0, RI = (0.0 / 1000.0) / 100.0, RD = 1/1000.0, RF=0;
//    // private double RP = 1.0 / 1200.0, RI = (0.0 / 1000.0) / 100.0, RD = 1/1500.0, RF=0;
//    private double RP = 1.0 / 1000.0, RI = (0.0 / 1000.0) / 100.0, RD = 1 / 1000.0, RF = 0;
//
//    // original
//    // private double rks=.11, rkv = .35 / 2040.0, rka = 14.5 / 2040.0;   // Porridge just right
//    // leftFF = new MiniFeedforward(.16, .5 / 2040.0, 16.5 / 2040.0);
//    // rightFF = new MiniFeedforward(.16, .5 / 2040.0, 16.5 / 2040.0);
//    // private double rks=.14, rkv = .61 / 2040.0, rka = 20.5 / 2040.0;  // Porridge too hot
//    private double rks = .13, rkv = .45 / 2040.0, rka = 10.5 / 2040.0;   // Porridge too cold
//    // private double rks=.13, rkv = .45 / 2040.0, rka = 14.5 / 2040.0;   // Porridge just right
//    private double lks = rks, lkv = rkv, lka = rka;
//
//    MiniPID RMiniPID = new MiniPID(RP, RI, RD, RF);
//    private int RPIDCount = 0;
//
//    private double[] LPIDTimestamp = new double[EncHistorySize];
//    private double[] LPIDEnc = new double[EncHistorySize];
//    private int LPIDEncCount = 0;
//    private double prevLPIDTime = 0.0;
//    private double LPIDTime = 0.010;
//    private double nextLPID = 0.0;
//    private double LP = RP, LI = RI, LD = RD, LF = RF;
//    MiniPID LMiniPID = new MiniPID(LP, LI, LD, LF);
//    private int LPIDCount = 0;
//
//    public Drive(HardwareMap map)
//    {
//        double volts = tgtV;
//        mPeriodicIO = new PeriodicIO();
//
//        // Control Hub Analog Sensors
//        // 0 - UltraSRight (blue)
//        // 1 - V33Right (white)
//        // 2 - UltraSLeft (blue)
//        // 3 - V33Left (white)
//        ultraRight = new MBUltraSonic(map, "V33Right", "UltraSRight");
//        ultraLeft = new MBUltraSonic(map, "V33Left", "UltraSLeft");
//
//        FL = map.get(DcMotorEx.class, "FL");
//        FR = map.get(DcMotorEx.class, "FR");
//        BL = map.get(DcMotorEx.class, "BL");
//        BR = map.get(DcMotorEx.class, "BR");
//
//        liftF = map.get(DcMotorEx.class, "liftF");
//        intake = map.get(DcMotorEx.class, "intake");
//
//        FR.setDirection(DcMotorEx.Direction.FORWARD);
//        BR.setDirection(DcMotorEx.Direction.REVERSE);
//        FL.setDirection(DcMotorEx.Direction.REVERSE);
//        BL.setDirection(DcMotorEx.Direction.FORWARD);
//
//        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        for (VoltageSensor sensor : map.voltageSensor)
//        {
//            volts = Math.min(sensor.getVoltage(), 14.2);
//            if (volts > 0)
//            {
//                vScale = ((tgtV - volts) / 7) + 1.0;
//                this.voltage = sensor;
//            }
//        }
//
//        initVolt = volts;
//
//        pixelLeft = map.get(Servo.class, "PL");
//        pixelRight = map.get(Servo.class, "PR");
//        setLeftPixel(LeftPixelInit);
//        setRightPixel(RightPixelInit);
//
//        mIsPositionMode = false;
//
//        initBHI260IMU(map);
//
//        setCustomPID(customPID);
//        setOpenLoop(DriveSignal.NEUTRAL);
//    }
//
//    public void autoInit()
//    {
//        setBrakeMode(false);
//
//        mMotionPlanner = new DriveMotionPlanner();
//
//        ultraLeft.autoInit();
//        // Left Sensor
//        // m = 7.1455
//        // b = -299.01
//        ultraLeft.sensorM = 7.1455;
//        ultraLeft.sensorB = -295.84;
//
//        ultraRight.autoInit();
//        // Right Sensor
//        // m = 7.1714
//        // b = -291.37
//        ultraRight.sensorM = 7.1714;
//        ultraRight.sensorB = -281.85;
//
//        durTimerPFol = new ElapsedTime();
//        durTimerPFol.reset();
//
//        if (Robot.isUsingComputer())
//        {
//            ComputerDebuggingNew debuggingNew = new ComputerDebuggingNew(); // have to call this just to make stuff not be null ptr exception
//            mMotionSim = new DriveMotionPlanner();
//        }
//
//        setLeftPixel(LeftPixelInit);
//        setRightPixel(RightPixelInit);
//
//        // We still do this here during init just in case we have swapping disabled.
//        // otherwise we took a null pointer exception.
//        // I would have expected the setCustomPID() in the constructor to handle
//        // this but we can investigate later.
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftFF = new MiniFeedforward(lks, lkv, lka);
//        rightFF = new MiniFeedforward(rks, rkv, rka);
//
//        RMiniPID.reset();
//        LMiniPID.reset();
//
//        RPIDEncCount = 0;
//        LPIDEncCount = 0;
//
//        RPIDCount = 0;
//        LPIDCount = 0;
//
//        RMiniPID.setOutputLimits(-1.0, 1.0);
//        RMiniPID.setMaxIOutput(0.35);
//        RMiniPID.setOutputRampRate(0.05);
//
//        //RMiniPID.ReduceErrorAtSetpoint = 0.3;
//
//        LMiniPID.setOutputLimits(-1.0, 1.0);
//        LMiniPID.setMaxIOutput(0.35);
//        LMiniPID.setOutputRampRate(0.05);
//
//        //LMiniPID.ReduceErrorAtSetpoint = 0.3;
//    }
//
//    public void teleopInit()
//    {
//        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        setOpenLoop(DriveSignal.NEUTRAL);
//        setStartPose(robot_state_.lastPose);
//
//        //robot_state_.reset(0, robot_state_.lastPose);
//        //startingHeading = robot_state_.lastPose.getRotation().getDegrees();
//
//        ultraLeft.autoInit();
//        ultraRight.autoInit();
//
//        setLeftPixel(LeftPixelInit);
//        setRightPixel(RightPixelInit);
//
//        BotLog.logD("IMU_Idx", "%d, %s", imuIdx, imuFatal);
//    }
//
//    public void update(double timestamp)
//    {
//        double startTime = 0.0;
//        if (debugLoopTime)
//        {
//            startTime = loopTimer.milliseconds();
//        }
//
//        readPeriodicInputs(timestamp);
//        updatePosition(timestamp);
//
//        switch (mDriveControlState)
//        {
//            case OPEN_LOOP:
//                break;
//            case PATH_FOLLOWING:
//                updatePathFollower(timestamp);
//
//                if (Robot.isUsingComputer())
//                {
//                    updateSim(timestamp);
//                }
//
//                break;
//            case GF_FOLLOWING:
//                updateGF();
//
//                if (Robot.isUsingComputer())
//                {
//                    ComputerDebuggingNew.sendRobotLocation(getXPos(), getYPos(), getHeadingRad());
//                    ComputerDebuggingNew.markEndOfUpdate();
//                }
//
//                break;
//            case TURN_TO_HEADING:
//                updateTurnToHeading();
//                break;
//            case PID_FOLLOWING:
//                updatePidFollower();
//                break;
//        }
//
//        writePeriodicOutputs();
//
//        if (debugLoopTime)
//        {
//            BotLog.logD("lt_drive :: ", String.format("%s", loopTimer.milliseconds() - startTime));
//        }
//    }
//
//    public void stop()
//    {
//        setOpenLoop(DriveSignal.NEUTRAL);
//        setLeftPixel(LeftPixelInit);
//        setRightPixel(RightPixelInit);
//        // Make sure to get NEUTRAL power command to the actual motors on a stop
//        writePeriodicOutputs();
//    }
//
//    public void Straight(double inches, double power, double duration, double P, double I, double D, double F)
//    {
//        int ticks = (int) ((inches / (Math.PI * Constants.kDriveWheelDiameterInches / 537.7)) / Constants.kGearRatio);
//
//        // BotLog.logD("Straight", "inches: %3.2f, ticks: %d, power: %3.2f", inches, ticks, power);
//
//        // for 24" @2000 50%:
//        // MiniPID LPID = new MiniPID(0.25/1000.0,.01/1000.0,.85/1000.0,0);
//        // for 6" @1200ms 50%:
//        // MiniPID LPID = new MiniPID(1.0/1000.0,.04/1000.0,.85/1000.0,0);
//        // for 2" @ 700ms 50%:
//        // MiniPID LPID = new MiniPID(3.0/1000.0,.04/1000.0,.85/1000.0,0);
//        // for 1" @ 350ms 50%:
//        // MiniPID LPID = new MiniPID(6.0/1000.0,.04/1000.0,.85/1000.0,0);
//        // Most recent P is (7.0/dist)/1000.0
//
//        MiniPID LPID = new MiniPID(P, I, D, F);
//        MiniPID RPID = new MiniPID(P, I, D, F);
//
//        double rpwr = 0.0;
//        double lpwr = 0.0;
//
//        ElapsedTime timer = new ElapsedTime();
//
//        double time1 = timer.milliseconds();
//        double time0 = timer.milliseconds();
//        double prevTime = time0;
//        double tgtTime = 0.0;
//
//        double pidRate = 25.0;
//
//        double rpos = FR.getCurrentPosition();
//        double lpos = FL.getCurrentPosition();
//
//        double rTgt = rpos + ticks;
//        double lTgt = lpos + ticks;
//
//        RPID.setOutputLimits(-power, power);
//        LPID.setOutputLimits(-power, power);
//        LPID.setMaxIOutput(0.35);
//        RPID.setMaxIOutput(0.35);
//        LPID.setOutputRampRate(0.09);
//        RPID.setOutputRampRate(0.09);
//        LPID.ReduceErrorAtSetpoint = 0.0;
//        RPID.ReduceErrorAtSetpoint = 0.0;
//
//        time0 = timer.milliseconds();
//        prevTime = time0;
//        time1 = time0;
//        while (timer.milliseconds() < (time1 + duration))
//        {
//            time0 = timer.milliseconds();
//            if (time0 > tgtTime)
//            {
//                rpos = FR.getCurrentPosition();
//                lpos = FL.getCurrentPosition();
//                rpwr = RPID.getOutput(rpos, rTgt, (time0 - prevTime) / pidRate);
//                lpwr = LPID.getOutput(lpos, lTgt, (time0 - prevTime) / pidRate);
//                prevTime = time0;
//                // BotLog.logD("Straight", "lpos: %4.0f, ltgt: %4.0f, lpwr: %3.2f, rpos: %4.0f, rtgt: %4.0f, rpwr: %3.2f", lpos, lTgt, lpwr, rpos, rTgt, rpwr);
//                tgtTime = time0 + pidRate;
//            }
//
//            FR.setPower(rpwr);
//            FL.setPower(lpwr);
//            BL.setPower(lpwr);
//            BR.setPower(rpwr);
//        }
//
//        FR.setPower(0.0);
//        FL.setPower(0.0);
//        BL.setPower(0.0);
//        BR.setPower(0.0);
//    }
//
//    private void addREncoderHistory(double REnc, double now)
//    {
//        // Verify we're not double adding
//        if (RPIDEncCount > 1)
//        {
//            int prev = (RPIDEncCount - 1) % EncHistorySize;
//            if ((RPIDEnc[prev] == REnc) && (RPIDTimestamp[prev] == now))
//            {
//                return;
//            }
//        }
//
//        RPIDEnc[RPIDEncCount % EncHistorySize] = REnc;
//        RPIDTimestamp[RPIDEncCount % EncHistorySize] = now;
//        RPIDEncCount++;
//
//        if (RPIDEncCount == 1)
//        {
//            for (int i = 1; i < EncHistorySize; i++)
//            {
//                RPIDEnc[i] = REnc;
//                RPIDTimestamp[i] = now;
//            }
//        }
//    }
//
//    private double getRVel(double delta)
//    {
//        // We must have 2 pieces of data to get a velocity
//        if ((RPIDEncCount <= 2) || (EncHistorySize < 2))
//        {
//            return mPeriodicIO.right_vel;
//        }
//        // The start is one back from the count, target time is offset from there
//        int start = (RPIDEncCount - 1) % EncHistorySize;
//        double tgtTime = RPIDTimestamp[start] - delta;
//
//        // Buffer idx to prevent negative, set to the start - 1
//        int idx = (start - 1) + EncHistorySize;
//
//        // Keep looking until we find an entry older than the tgtTime or run out
//        for (int i = 0; (i < (EncHistorySize - 2)) && (RPIDTimestamp[idx % EncHistorySize] > tgtTime); i++)
//        {
//            idx--;
//        }
//
//        // Constrain to our array size
//        idx = idx % EncHistorySize;
//        // Handle / 0
//        double vel = Double.NaN;
//        if ((RPIDTimestamp[start] - RPIDTimestamp[idx]) != 0)
//        {
//            vel = (RPIDEnc[start] - RPIDEnc[idx]) / (RPIDTimestamp[start] - RPIDTimestamp[idx]);
//            ;
//        }
//        //BotLog.logD("RVel","%5.1f vs %5.1f", vel, FR.getVelocity());
//        return vel;
//    }
//
//    private void addLEncoderHistory(double LEnc, double now)
//    {
//        // Verify we're not double adding
//        if (LPIDEncCount > 1)
//        {
//            int prev = (LPIDEncCount - 1) % EncHistorySize;
//            if ((LPIDEnc[prev] == LEnc) && (LPIDTimestamp[prev] == now))
//            {
//                return;
//            }
//        }
//
//        LPIDEnc[LPIDEncCount % EncHistorySize] = LEnc;
//        LPIDTimestamp[LPIDEncCount % EncHistorySize] = now;
//        LPIDEncCount++;
//
//        if (LPIDEncCount == 1)
//        {
//            for (int i = 1; i < EncHistorySize; i++)
//            {
//                LPIDEnc[i] = LEnc;
//                LPIDTimestamp[i] = now;
//            }
//        }
//    }
//
//    private double getLVel(double delta)
//    {
//        // We must have 2 pieces of data to get a velocity
//        if ((LPIDEncCount <= 2) || (EncHistorySize < 2))
//        {
//            return mPeriodicIO.left_vel;
//        }
//        // The start is one back from the count, target time is offset from there
//        int start = (LPIDEncCount - 1) % EncHistorySize;
//        double tgtTime = LPIDTimestamp[start] - delta;
//
//        // Buffer idx to prevent negative, set to the start - 1
//        int idx = (start - 1) + EncHistorySize;
//
//        // Keep looking until we find an entry older than the tgtTime or run out
//        for (int i = 0; (i < (EncHistorySize - 2)) && (LPIDTimestamp[idx % EncHistorySize] > tgtTime); i++)
//        {
//            idx--;
//        }
//
//        // Constrain to our array size
//        idx = idx % EncHistorySize;
//        // Handle / 0
//        double vel = Double.NaN;
//        if ((LPIDTimestamp[start] - LPIDTimestamp[idx]) != 0)
//        {
//            vel = (LPIDEnc[start] - LPIDEnc[idx]) / (LPIDTimestamp[start] - LPIDTimestamp[idx]);
//        }
//        // BotLog.logD("LVel","%5.1f vs %5.1f", vel, FL.getVelocity());
//        return vel;
//    }
//
//    private void updateRPID(double timestamp)
//    {
//        // Time went backwards, or we had a gap larger than 25x expected
//        // Try to re-sync on the next call
//        if ((timestamp < prevRPIDTime) || ((timestamp - prevRPIDTime) > (RPIDTime * 25.0)))
//        {
//            BotLog.logD("RPID :: ", "Unexpected times %5.2f, %5.2f, %5.2f", timestamp, prevRPIDTime, RPIDTime);
//
//            nextRPID = timestamp + RPIDTime;
//            prevRPIDTime = timestamp;
//
//            if (RPIDCount != 0)
//            {
//                return;
//            }
//        }
//
//        if ((nextRPID < timestamp) || (RPIDCount == 0))
//        {
//            // Leave this here just in case we don't come through 'update()' for this path.
//            readPeriodicInputs(timestamp);
//
//            double power = 0;
//            double curVel = getRVel(velDelta);
//
//            double pidTgtVel = radToTicks(mPeriodicIO.right_demand);
//            RPIDCount++;
//
//            if (RPIDCount > 1)
//            {
//                power = RMiniPID.getOutput(curVel, pidTgtVel, ((timestamp - prevRPIDTime) / RPIDTime));
//            }
//
//            power += rightFF.calculate(pidTgtVel, mPeriodicIO.right_accel);
//            mPeriodicIO.right_custom = power;
//
//            nextRPID = timestamp + RPIDTime;
//            prevRPIDTime = timestamp;
//        }
//    }
//
//    private void updateLPID(double timestamp)
//    {
//        // Time went backwards, or we had a gap larger than 25x expected
//        // Try to re-sync on the next call
//        if ((timestamp < prevLPIDTime) || ((timestamp - prevLPIDTime) > (LPIDTime * 25.0)))
//        {
//            BotLog.logD("LPID :: ", "Unexpected times %5.2f, %5.2f, %5.2f", timestamp, prevLPIDTime, LPIDTime);
//
//            nextLPID = timestamp + LPIDTime;
//            prevLPIDTime = timestamp;
//
//            if (LPIDCount != 0)
//            {
//                return;
//            }
//        }
//
//        if ((nextLPID < timestamp) || (LPIDCount == 0))
//        {
//            // Leave this here just in case we don't come through 'update()' for this path.
//            readPeriodicInputs(timestamp);
//
//            double power = 0;
//            double curVel = getLVel(velDelta);
//
//            double pidTgtVel = radToTicks(mPeriodicIO.left_demand);
//            LPIDCount++;
//
//            if (LPIDCount > 1)
//            {
//                power = LMiniPID.getOutput(curVel, pidTgtVel, ((timestamp - prevLPIDTime) / LPIDTime));
//            }
//
//            power += leftFF.calculate(pidTgtVel, mPeriodicIO.left_accel);
//            mPeriodicIO.left_custom = power;
//
//            nextLPID = timestamp + LPIDTime;
//            prevLPIDTime = timestamp;
//        }
//    }
//
//    public synchronized double getVoltageScale()
//    {
//        if (initVolt > 12.8)
//        {
//            return Math.pow(targetGFVolts / initVolt, 2);
//        }
//
//        return Math.pow(12.7 / initVolt, 2);
//
//    }
//
//    public synchronized double getInitVolt()
//    {
//        return initVolt;
//    }
//
//    public synchronized void setWantGFPath(ArrayList<CurvePoint> path, boolean rev)
//    {
//        setBrakeMode(false);
//        setCustomPID(true); // force driver encoders
//
//        currGFPath = path;
//        GFReversed = rev;
//
//        GFMovement.initForMove();
//        GFMovement.initCurve();
//        GFMovement.updateRobotVars(getXPos(), getYPos(), getHeadingRad(), 0, 0, 0);
//
//        mDriveControlState = DriveControlState.GF_FOLLOWING;
//    }
//
//    public synchronized void setWantGFPath(Path path)
//    {
//        setWantGFPath(path.getPath(), path.getReversed());
//    }
//
//
//    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory)
//    {
//        if (mMotionPlanner != null)
//        {
//            // allow trajectory control
//            setBrakeMode(true);
//            mOverrideTrajectory = false;
//
//            // set up motion planner
//            mMotionPlanner.reset();
//            mMotionPlanner.setTrajectory(trajectory);
//
//            if (Robot.isUsingComputer())
//            {
//                // set up simulator
//                mMotionSim.reset();
//                mMotionSim.setTrajectory(new TrajectoryIterator<>(trajectory));
//                lastPose = mMotionSim.setpoint().state().getPose();
//            }
//
//            errorLogged = false;
//
//            // set state
//            mDriveControlState = DriveControlState.PATH_FOLLOWING;
//
//            if (customPID)
//            {
//                // Reset custom VelPID and history
//                RMiniPID.reset();
//                LMiniPID.reset();
//                RPIDEncCount = 0;
//                LPIDEncCount = 0;
//                RPIDCount = 0;
//                LPIDCount = 0;
//            }
//        }
//    }
//
//    public synchronized void setWantTurnToHeading(Rotation2d heading, double turnSpeed, double decelRads)
//    {
//        if (mDriveControlState != DriveControlState.TURN_TO_HEADING)
//        {
//            setBrakeMode(true);
//            mDriveControlState = DriveControlState.TURN_TO_HEADING;
//        }
//
//        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3)
//        {
//            mTurnSpeed = turnSpeed;
//            mDecelRads = decelRads;
//            mTargetHeading = heading;
//            mIsOnTarget = false;
//        }
//    }
//
//    public void setCustomPID(boolean use)
//    {
//        boolean enableSwapping = true;
//        if (enableSwapping)
//        {
//            customPID = use;
//
//            if (use)
//            {
//                FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                leftFF = new MiniFeedforward(lks, lkv, lka);
//                rightFF = new MiniFeedforward(rks, rkv, rka);
//
//                RMiniPID.reset();
//                LMiniPID.reset();
//
//                RPIDEncCount = 0;
//                LPIDEncCount = 0;
//
//                RPIDCount = 0;
//                LPIDCount = 0;
//
//                RMiniPID.setOutputLimits(-1.0, 1.0);
//                RMiniPID.setMaxIOutput(0.35);
//                RMiniPID.setOutputRampRate(0.05);
//
//                //RMiniPID.ReduceErrorAtSetpoint = 0.3;
//
//                LMiniPID.setOutputLimits(-1.0, 1.0);
//                LMiniPID.setMaxIOutput(0.35);
//                LMiniPID.setOutputRampRate(0.05);
//
//                //LMiniPID.ReduceErrorAtSetpoint = 0.3;
//            }
//            else
//            {
//                FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                vScale = Range.clip(vScale, 0.85, 1.2);
//
//                double P = 33.0;
//                double I = 0.34;
//                double D = 0.0;
//                double F = 3.3;
//
//                FR.setVelocityPIDFCoefficients(P * vScale, I * vScale, D * vScale, F * vScale); // p :: 15, i :: 4.5, d :: 0, f :: 11.1
//                BR.setVelocityPIDFCoefficients(P * vScale, I * vScale, D * vScale, F * vScale);
//                FL.setVelocityPIDFCoefficients(P * vScale, I * vScale, D * vScale, F * vScale);
//                BL.setVelocityPIDFCoefficients(P * vScale, I * vScale, D * vScale, F * vScale);
//            }
//        }
//    }
//
//    public void setWantPidFollowing(double leftIn, double rightIn, double pow, double dur, double lP, double lI, double lD, double lF, double rP, double rI, double rD, double rF)
//    {
//        if (mDriveControlState != DriveControlState.PID_FOLLOWING)
//        {
//            setBrakeMode(false);
//            mDriveControlState = DriveControlState.PID_FOLLOWING;
//        }
//
//        leftTicksPFol = mPeriodicIO.left_position_ticks + (int) ((leftIn / (Math.PI * Constants.kDriveWheelDiameterInches / 537.7)) / Constants.kGearRatio);
//        rightTicksPFol = mPeriodicIO.right_position_ticks + (int) ((rightIn / (Math.PI * Constants.kDriveWheelDiameterInches / 537.7)) / Constants.kGearRatio);
//        BotLog.logD("Straight", "leftIn: %3.2f, leftTicks: %3.2f, rightIn: %3.2f, rightTicks: %3.2f, pow: %3.2f, dur: %3.0f, lact: %3.0f, ract: %3.0f",
//                leftIn, leftTicksPFol, rightIn, rightTicksPFol, pow, dur, mPeriodicIO.left_position_ticks, mPeriodicIO.right_position_ticks);
//
//        leftPFol = new MiniPID(lP, lI, lD, lF);
//        rightPFol = new MiniPID(rP, rI, rD, rF);
//
//        leftPFol.setOutputLimits(-pow, pow);
//        rightPFol.setOutputLimits(-pow, pow);
//
//        leftPFol.setMaxIOutput(.35);
//        rightPFol.setMaxIOutput(.35);
//
//        leftPFol.setOutputRampRate(.065 / (25.0 / pidRatePFol));
//        rightPFol.setOutputRampRate(.065 / (25.0 / pidRatePFol));
//
//        leftPFol.ReduceErrorAtSetpoint = 0.0;
//        rightPFol.ReduceErrorAtSetpoint = 0.0;
//
//        durTimerPFol.reset();
//        prevTimePFol = 0.0;
//        tgtTimePFol = 0.0;
//        durationPFol = dur;
//
//        /*
//        // prevent everything from resetting while this variable is still being called... we should REALLY switch states once we call this method though
//        if (durTimer.milliseconds() > duration)
//        {
//            startTime = durTimer.milliseconds();
//            tgtTime = durTimer.milliseconds();
//            duration = dur;
//        }
//        */
//    }
//
//    public synchronized void setBrakeMode(boolean on)
//    {
//        if (mIsBrakeMode != on)
//        {
//            mIsBrakeMode = on;
//            DcMotor.ZeroPowerBehavior mode = on ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
//
//            FL.setZeroPowerBehavior(mode);
//            FR.setZeroPowerBehavior(mode);
//            BR.setZeroPowerBehavior(mode);
//            BL.setZeroPowerBehavior(mode);
//        }
//    }
//
//    public synchronized void setOpenLoop(DriveSignal signal)
//    {
//        if (mDriveControlState != DriveControlState.OPEN_LOOP)
//        {
//            setBrakeMode(false);
//            mDriveControlState = DriveControlState.OPEN_LOOP;
//        }
//
//        setBrakeMode(signal.mBrakeMode);
//
//        mPeriodicIO.left_demand = signal.getLeft();
//        mPeriodicIO.right_demand = signal.getRight();
//        mPeriodicIO.left_feedforward = 0.0;
//        mPeriodicIO.right_feedforward = 0.0;
//    }
//
//    public synchronized void setTurnVel(DriveSignal signal)
//    {
//        if (mDriveControlState != DriveControlState.TURN_TO_HEADING)
//        {
//            setBrakeMode(true);
//            mDriveControlState = DriveControlState.TURN_TO_HEADING;
//        }
//
//        mPeriodicIO.left_demand = signal.getLeft();
//        mPeriodicIO.right_demand = signal.getRight();
//        mPeriodicIO.left_feedforward = 0.0;
//        mPeriodicIO.right_feedforward = 0.0;
//    }
//
//    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward)
//    {
//        if (mDriveControlState != DriveControlState.PATH_FOLLOWING)
//        {
//            setBrakeMode(true);
//            mDriveControlState = DriveControlState.PATH_FOLLOWING;
//        }
//
//        setBrakeMode(signal.mBrakeMode);
//
//        mPeriodicIO.left_demand = signal.getLeft();
//        mPeriodicIO.right_demand = signal.getRight();
//        mPeriodicIO.left_feedforward = feedforward.getLeft();
//        mPeriodicIO.right_feedforward = feedforward.getRight();
//
//        mPeriodicIO.left_demand_final = radToTicks(mPeriodicIO.left_demand + Constants.kDriveVelocityKd * mPeriodicIO.left_accel + Constants.kDriveVelocityKf * mPeriodicIO.left_feedforward);
//        mPeriodicIO.right_demand_final = radToTicks(mPeriodicIO.right_demand + Constants.kDriveVelocityKd * mPeriodicIO.right_accel + Constants.kDriveVelocityKf * mPeriodicIO.right_feedforward);
//    }
//
//    private void setPower(boolean finished)
//    {
//        if (mDriveControlState != DriveControlState.GF_FOLLOWING)
//        {
//            setBrakeMode(true);
//            mDriveControlState = DriveControlState.GF_FOLLOWING;
//        }
//
//        double left = GFMovement.getMovement_y() + GFMovement.getMovement_rad();
//        double right = GFMovement.getMovement_y() - GFMovement.getMovement_rad();
//
//        double maxPower = Math.max(Math.abs(left), Math.abs(right));
//
//        if (maxPower > 1)
//        {
//            left /= maxPower;
//            right /= maxPower;
//        }
//
//        if (!finished)
//        {
//            mPeriodicIO.left_demand = left;
//            mPeriodicIO.right_demand = right;
//        }
//        else
//        {
//            mPeriodicIO.left_demand = 0;
//            mPeriodicIO.right_demand = 0;
//        }
//    }
//
//    public boolean isDoneWithGF()
//    {
//        if (mDriveControlState != DriveControlState.GF_FOLLOWING)
//        {
//            return true;
//        }
//
//        return mIsDoneGF;
//    }
//
//    public boolean isDoneWithTrajectory()
//    {
//        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING)
//        {
//            // BotLog.logD("planner", String.format("%s", (mMotionPlanner == null) + ""));
//            // BotLog.logD("state", String.format("%s", (mDriveControlState != DriveControlState.PATH_FOLLOWING) + ""));
//            return true;
//        }
//
//        if (!errorLogged)
//        {
//            //BotLog.logD("error", String.format("%s", mPeriodicIO.error + ""));
//            errorLogged = true;
//        }
//
//        return mMotionPlanner.isDone() || mOverrideTrajectory;
//    }
//
//    public boolean isSimDoneWithTrajectory()
//    {
//        if (mMotionSim == null || mDriveControlState != DriveControlState.PATH_FOLLOWING)
//        {
//            return true;
//        }
//        return mMotionSim.isDone();
//    }
//
//    public synchronized boolean isDoneWithTurn()
//    {
//        if (mDriveControlState == DriveControlState.TURN_TO_HEADING)
//        {
//            return mIsOnTarget;
//        }
//
//        return false;
//    }
//
//    public boolean isDoneWithPID()
//    {
//        if ((mDriveControlState != DriveControlState.PID_FOLLOWING) || (durTimerPFol.milliseconds() > durationPFol))
//        {
//            // consider returning true if mPeriodicIO.left_position_ticks == leftTicks or is close
//            return true;
//        }
//
//        return false;
//    }
//
//    public TimedState<Pose2dWithCurvature> getPathSetpoint()
//    {
//        return mPeriodicIO.path_setpoint;
//    }
//
//    public Pose2d getTargetPose()
//    {
//        if (mDriveControlState == DriveControlState.GF_FOLLOWING)
//        {
//            return GFMovement.getPose();
//        }
//        else
//        {
//            return mPeriodicIO.path_setpoint.state().getPose();
//        }
//    }
//
//    public void overrideTrajectory(boolean value)
//    {
//        mOverrideTrajectory = value;
//    }
//
//    public synchronized Rotation2d getHeading()
//    {
//        return mPeriodicIO.gyro_heading;
//    }
//
//    public double getLeftEncoderRotations()
//    {
//        return mPeriodicIO.left_position_ticks / 537.7;
//    }
//
//    public double getRightEncoderRotations()
//    {
//        return mPeriodicIO.right_position_ticks / 537.7;
//    }
//
//    public double getLeftEncoderDistance()
//    {
//
//        return rotationsToInches(getLeftEncoderRotations());
//    }
//
//    public double getRightEncoderDistance()
//    {
//        return rotationsToInches(getRightEncoderRotations());
//    }
//
//    // rad/sec that the motor is spinning, not the wheel
//    public double getRightLinearVelRad()
//    {
//        return (mPeriodicIO.right_vel_ticks / 537.7) * 2 * Math.PI;
//    }
//
//    // rad/sec that the motor is spinning, not the wheel
//    public double getLeftLinearVelRad()
//    {
//        return (mPeriodicIO.left_vel_ticks / 537.7) * 2 * Math.PI;
//    }
//
//    // rad/sec that the wheel is spinning
//    public double getLeftLinearVelMotorRad()
//    {
//        //return (((BL.getVelocity() + FL.getVelocity()) / 2.0 ) / 537.7) * 2 * Math.PI * Constants.kGearRatio;
//        return ((FL.getVelocity()) / 537.7) * 2 * Math.PI * Constants.kGearRatio;
//
//    }
//
//    // rad/sec that the wheel is spinning
//    public double getRightLinearVelMotorRad()
//    {
//        //return (((BR.getVelocity() + FR.getVelocity()) / 2.0 ) / 537.7) * 2 * Math.PI * Constants.kGearRatio;
//        return ((FR.getVelocity()) / 537.7) * 2 * Math.PI * Constants.kGearRatio;
//    }
//
//    // inches/sec that the wheel is moving
//    public double getLeftLinearVelocityInches()
//    {
//        return rotationsToInches(mPeriodicIO.left_vel_ticks / 537.7);
//    }
//
//    // inches/sec that the wheel is moving
//    public double getRightLinearVelocityInches()
//    {
//        return rotationsToInches(mPeriodicIO.right_vel_ticks / 537.7);
//    }
//
//    // inches/sec that the robot is moving
//    public double getLinearVelocityInches()
//    {
//        return (getLeftLinearVelocityInches() + getRightLinearVelocityInches()) / 2.0;
//    }
//
//    // rad/sec that the robot is moving
//    public double getAngularVelocity()
//    {
//        return (getRightLinearVelocityInches() - getLeftLinearVelocityInches()) / Constants.kDriveWheelTrackWidthInches;
//    }
//
//    // deg/sec that the robot is moving
//    public double getAngularVelocityMotor()
//    {
//        return Math.toDegrees((rotationsToInches(FL.getVelocity() / 537.7) - rotationsToInches(FR.getVelocity() / 537.7)) / Constants.kDriveWheelTrackWidthInches);
//    }
//
//    // inches/sec that the robot is moving
//    public double getMotorLinearVelInches()
//    {
//        double left = mPeriodicIO.left_vel / 537.7;
//        double right = mPeriodicIO.right_vel / 537.7;
//        return ((rotationsToInches(left) + rotationsToInches(right)) / 2.0);
//    }
//
//    // inches/sec that the wheel is spinning
//    public double getLeftLinearVelMotorInches()
//    {
//        return rotationsToInches(mPeriodicIO.left_vel / 537.7);
//    }
//
//    // inches/sec that the wheel is spinning
//    public double getRightLinearVelMotorInches()
//    {
//        return rotationsToInches(mPeriodicIO.right_vel / 537.7);
//    }
//
//    // rotation of motor to inches of wheel movement
//    private static double rotationsToInches(double rotations)
//    {
//        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI * Constants.kGearRatio);
//    }
//
//    // inches of wheel movement to required motor rotation
//    private static double inchesToRotations(double inches)
//    {
//        return inches / (Constants.kDriveWheelDiameterInches * Math.PI * Constants.kGearRatio);
//    }
//
//    // rad that the wheel should rotate to ticks of the motor
//    public static double radToTicks(double rad)
//    {
//        return ((rad / (2 * Math.PI)) / Constants.kGearRatio) * 537.7;
//    }
//
//    public double getBRCurrent()
//    {
//        return (BR.getCurrent(CurrentUnit.AMPS));
//    }
//
//    public double getBLCurrent()
//    {
//        return (BL.getCurrent(CurrentUnit.AMPS));
//    }
//
//    public double getFRCurrent()
//    {
//        return (FR.getCurrent(CurrentUnit.AMPS));
//    }
//
//    public double getFLCurrent()
//    {
//        return (FL.getCurrent(CurrentUnit.AMPS));
//    }
//
//    public double getMotorCurrent()
//    {
//        return (getBRCurrent() + getBLCurrent() + getFRCurrent() + getFLCurrent());
//    }
//
//
//    public Pose2d getPose()
//    {
//        return robot_state_.getLatestFieldToVehicle();
//    }
//
//    private void updateSim(double timestamp)
//    {
//        DriveMotionPlanner.Output output = mMotionSim.update(timestamp, lastPose);
//        Twist2d delta = Kinematics.forwardKinematics(output.left_velocity * (timestamp - lastSimTime) * (Constants
//                .kDriveWheelDiameterInches / 2.0), output.right_velocity * (timestamp - lastSimTime) * (Constants
//                .kDriveWheelDiameterInches / 2.0));
//        lastPose = lastPose.transformBy(Pose2d.exp(delta));
//        lastSimTime = timestamp;
//    }
//
//    private void updatePathFollower(double timestamp)
//    {
//        if (mDriveControlState == DriveControlState.PATH_FOLLOWING)
//        {
//
//            final double now = timestamp;
//
//            //output returns rad/sec that wheels should rotate
//            DriveMotionPlanner.Output output = mMotionPlanner.update(now, robot_state_.getFieldToVehicle());
//
//            mPeriodicIO.error = mMotionPlanner.error();
//            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();
//
//            if (!mOverrideTrajectory)
//            {
//                setVelocity(new DriveSignal(output.right_velocity, output.left_velocity), new DriveSignal(output.right_feedforward_voltage / 12, output.left_feedforward_voltage / 12));
//
//                if (customPID)
//                {
//                    mPeriodicIO.left_accel = output.right_accel;
//                    mPeriodicIO.right_accel = output.left_accel;
//                    updateLPID(timestamp);
//                    updateRPID(timestamp);
//
//                    if (twitchReduction)
//                    {
//                        // Make sure we're not causing one side of the drivetrain to run in reverse
//                        // of the other side and in reverse of the requested velocity
//                        if ((Math.signum(mPeriodicIO.left_custom) == Math.signum(mPeriodicIO.left_demand)) !=
//                                (Math.signum(mPeriodicIO.right_custom) == Math.signum(mPeriodicIO.right_demand)))
//                        {
//                            if (Math.signum(mPeriodicIO.left_custom) != Math.signum(mPeriodicIO.left_demand))
//                            {
//                                mPeriodicIO.left_custom = -0.02;
//                            }
//                            else
//                            {
//                                mPeriodicIO.right_custom = -0.02;
//                            }
//                        }
//                    }
//                }
//                else
//                {
//                    // 254 divided by 1000
//                    mPeriodicIO.left_accel = output.right_accel / 1000.0;
//                    mPeriodicIO.right_accel = output.left_accel / 1000.0;
//                }
//            }
//            else
//            {
//                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
//                mPeriodicIO.left_accel = 0.0;
//                mPeriodicIO.right_accel = 0.0;
//            }
//        }
//    }
//
//    private void updateGF()
//    {
//        if (mDriveControlState == DriveControlState.GF_FOLLOWING)
//        {
//            double xV = inToCM(getMotorLinearVelInches()) * Math.cos(getHeadingRad());
//            double yV = inToCM(getMotorLinearVelInches()) * Math.sin(getHeadingRad());
//            double rV = getAngularVelocity();
//
//            GFMovement.updateRobotVars(inToCM(getXPos()), inToCM(getYPos()), getHeadingRad(), xV, yV, rV);
//            //BotLog.logD("pos :: ", String.format("x :: %.2f, y :: %.2f, rad :: %.2f", inToCM(getXPos()), inToCM(getYPos()), getHeadingRad()));
//            mIsDoneGF = followCurve(currGFPath, GFReversed ? Math.toRadians(-90) : Math.toRadians(90));
//
//            setPower(mIsDoneGF); // set all updated powers and make sure we scale down below 1
//        }
//    }
//
//    private void updateTurnToHeading()
//    {
//        // Figure out the rotation necessary to turn to face the goal.
//        final Rotation2d field_to_robot = robot_state_.getLatestFieldToVehicle().getRotation();
//        final double relativePointAngle = field_to_robot.inverse().rotateBy(mTargetHeading).getRadians() - (-.14 * getAngularVelocity());
//
//        if (Math.abs(relativePointAngle) < Math.toRadians(5))
//        {
//            mIsOnTarget = true;
//        }
//
//        double turnSpeed = (relativePointAngle / mDecelRads) * mTurnSpeed;
//        double adjustedTurnSpeed = Range.clip(turnSpeed, -mTurnSpeed, mTurnSpeed);
//        adjustedTurnSpeed *= Range.clip(Math.abs(relativePointAngle) / Math.toRadians(3), 0, 1);
//
//        setTurnVel(new DriveSignal(adjustedTurnSpeed, -adjustedTurnSpeed));
//
//    }
//
//    private void updatePidFollower()
//    {
//        if (durTimerPFol.milliseconds() < durationPFol)
//        {
//            double timePFol = durTimerPFol.milliseconds();
//
//            if (prevTimePFol == 0.0)
//            {
//                prevTimePFol = timePFol;
//            }
//
//            if (timePFol > tgtTimePFol)
//            {
//                mPeriodicIO.left_demand = leftPFol.getOutput(mPeriodicIO.left_position_ticks, leftTicksPFol, (timePFol - prevTimePFol) / pidRatePFol);
//                mPeriodicIO.right_demand = rightPFol.getOutput(mPeriodicIO.right_position_ticks, rightTicksPFol, (timePFol - prevTimePFol) / pidRatePFol);
//                BotLog.logD("Straight", "lpw: %5.2f, ltg: %5.2f, lps: %5d, rpw: %5.2f, rtg: %5.2f, rps: %5d, d_t: %5.2f",
//                        mPeriodicIO.left_demand, leftTicksPFol, (int) mPeriodicIO.left_position_ticks,
//                        mPeriodicIO.right_demand, rightTicksPFol, (int) mPeriodicIO.right_position_ticks,
//                        timePFol - prevTimePFol);
//
//                prevTimePFol = timePFol;
//                tgtTimePFol = timePFol + pidRatePFol;
//            }
//        }
//        else
//        {
//            // BotLog.logD("Straight", "Stopping: pwr=0");
//            mPeriodicIO.left_demand = 0.0;
//            mPeriodicIO.right_demand = 0.0;
//        }
//    }
//
//    public void setLeftPixel(double position)
//    {
//        mPeriodicIO.left_pixel_servo_position = position;
//    }
//
//    public void setRightPixel(double position)
//    {
//        mPeriodicIO.right_pixel_servo_position = position;
//    }
//
//    public void initLocalizer(double timestamp)
//    {
//        readPeriodicInputs(timestamp);
//        robot_state_.initLocalizer((int) mPeriodicIO.parallel_encoder, (int) mPeriodicIO.lateral_encoder, mPeriodicIO.gyro_heading.getRadians());
//    }
//
//    private void updatePosition(double timestamp)
//    {
//        final double left_distance = getLeftEncoderDistance(); // inches that the robot has moved
//        final double right_distance = getRightEncoderDistance(); // inches that the robot has moved
//
//        final double delta_left = left_distance - left_encoder_prev_distance_;
//        final double delta_right = right_distance - right_encoder_prev_distance_;
//
//        if (RobotState.usingDeadwheels)
//        {
//            final Pose2d newPose = robot_state_.generateScuffed2WheelOdom((int) mPeriodicIO.parallel_encoder, (int) mPeriodicIO.lateral_encoder, mPeriodicIO.gyro_heading.getRadians());
//            robot_state_.addFieldToVehicleObservation(timestamp, newPose);
//        }
//        else
//        {
//            final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(delta_left, delta_right, mPeriodicIO.gyro_heading);
//            robot_state_.addObservations(timestamp, odometry_velocity);
//        }
//
//        left_encoder_prev_distance_ = left_distance;
//        right_encoder_prev_distance_ = right_distance;
//    }
//
//    public void relocalize(double timestamp, Pose2d pose)
//    {
//        robot_state_.addFieldToVehicleObservation(timestamp, pose);
//        robot_state_.relocalize(pose);
//    }
//
//    public void logIMUs()
//    {
//        Orientation D[] = new Orientation[IMUCount];
//
//        // Each of these adds 20ms to the loop time, don't call this lightly
//        for (int i = 0; i < imus.length; i++)
//        {
//            D[i] = imus[i].getRobotOrientationAsQuaternion().toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            if (i == imuIdx)
//            {
//                BotLog.logD(String.format("imu[%d]", i), "IMU:%5.1f, Orien:%5.1f, HO: %5.2f, intIMU:%5.1f, Gyro2d:%s", D[i].firstAngle, (D[i].firstAngle * imuCorr[i]) - Math.toDegrees(headingOffset[i]), Math.toDegrees(headingOffset[i]), Math.toDegrees(intHeading), mPeriodicIO.gyro_heading);
//            }
//            else
//            {
//                BotLog.logD(String.format("imu[%d]", i), "IMU:%5.1f, Orien:%5.1f, HO: %5.2f", D[i].firstAngle, (D[i].firstAngle * imuCorr[i]) - Math.toDegrees(headingOffset[i]), Math.toDegrees(headingOffset[i]));
//            }
//        }
//        BotLog.logD("imu[encoders]", "IMU:%5.1f, Orien:%5.1f, HO: %5.2f", Math.toDegrees(encoder_heading), Math.toDegrees(encoder_heading), 0.0);
//    }
//
//    public void setStartPose(Pose2d startPose)
//    {
//        startingHeading = startPose.getRotation().inverse().getRadians();
//        robot_state_.reset(startPose);
//    }
//
//    private void initBHI260IMU(HardwareMap hardwareMap)
//    {
//        // Get the HW
//        imus[EXT_IMU] = hardwareMap.get(IMU.class, "eIMU");
//        imus[EXT2_IMU] = hardwareMap.get(IMU.class, "eIMU2");
//        imus[CH_IMU] = hardwareMap.get(IMU.class, "imu");
//        imus[EXP_IMU] = hardwareMap.get(IMU.class, "expIMU");
//
//        // Describe the position
//        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
//        IMU.Parameters imuParams[] = new IMU.Parameters[IMUCount];
//        imuParams[EXT_IMU] = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)); // extIMU (eIMU)
//        imuParams[EXT2_IMU] = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)); // extIMU (eIMU)
//        imuParams[CH_IMU] = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));   // CH (imu)
//        imuParams[EXP_IMU] = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN));  // ExpHub (expIMU)
//
//        // Assign correction factors for each IMU
//        imuCorr[EXT_IMU] = 1814.0 / 1800.0;
//        imuCorr[EXT2_IMU] = 1806.0 / 1800.0;
//        imuCorr[CH_IMU] = 1796.0 / 1800.0;
//        imuCorr[EXP_IMU] = 1809.6 / 1800.0;
//
//        // Initialize
//        for (int i = 0; i < imus.length; i++)
//        {
//            imus[i].initialize(imuParams[i]);
//        }
//
//        // Wait 500ms to allow the IMUs to init
//        ElapsedTime timer = new ElapsedTime();
//        while (timer.seconds() < 0.5)
//        {
//        }
//
//        // Start at '1', the order is assigned by change the *_IMU values
//        imuIdx = 0;
//
//        initIMUPrevAndOffset();
//    }
//
//    public void initIMUPrevAndOffset()
//    {
//        // Record starting heading for logging and init the prev variables
//        for (int i = 0; i < imus.length; i++)
//        {
//            headingOffset[i] = imus[i].getRobotOrientationAsQuaternion().toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//        }
//
//        prevHeading = headingOffset[imuIdx];
//        prevDelta = 0.0;
//        prevHeading0 = prevHeading;
//    }
//
//
//    public synchronized void simpleReadIMU(double timestamp)
//    {
//        double delta;
//
//        // We get the current Quaternion
//        curQ = imus[imuIdx].getRobotOrientationAsQuaternion();
//        Orientation curOrientation = curQ.toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//
//        curHeading = curOrientation.firstAngle;
//        // delta = AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(prevHeading - curHeading) * imuCorr[imuIdx]);
//        delta = AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(prevHeading - curHeading));
//        intHeading = AngleUnit.normalizeRadians(intHeading - delta);
//        prevHeading = curHeading;
//        mPeriodicIO.gyro_heading = Rotation2d.fromRadians(AngleUnit.normalizeRadians(startingHeading + intHeading)).inverse();
//
//    }
//
//
//    public synchronized void readIMUWithFailover(double timestamp)
//    {
//        boolean failedOver = false;
//        double delta;
//
//        // We get the current Quaternion
//        curQ = imus[imuIdx].getRobotOrientationAsQuaternion();
//        Orientation curOrientation = curQ.toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//
//        // If the force Fail is enabled
//        if (forceIMUFailover > 0)
//        {
//            Random rand = new Random();
//            if (rand.nextInt(forceIMUFailover) == 0)
//            {
//                // Did we get lucky?  Pick a fail type and modify the curOrientation
//                int fail = rand.nextInt(3);
//                BotLog.logD("imu_FrcFail", "Rate:%d, Type:%d (0=time, 1=angle, 2=both)", forceIMUFailover, fail);
//
//                if (fail == 0 || fail == 2)
//                {
//                    curOrientation.acquisitionTime = 0;
//                }
//
//                if (fail == 1 || fail == 2)
//                {
//                    curOrientation.firstAngle = Float.NaN;
//                }
//
//                failedOver = true;
//            }
//        }
//
//        if ((curOrientation.acquisitionTime == 0) || (Float.isNaN(curOrientation.firstAngle)))
//        {
//            // Our IMU returned bad data.  Probably an ESD event.  Log a message and try to recover
//            BotLog.logD("imu_Err", "%s %s, imuIdx %d --> %d", curOrientation.acquisitionTime, curOrientation, imuIdx, (imuIdx + 1) % imus.length);
//
//            // Advance to the next IMU in our list
//            imuIdx = (imuIdx + 1) % imus.length;
//
//            // Re-read the heading from the next IMU
//            curQ = imus[imuIdx].getRobotOrientationAsQuaternion();
//            curOrientation = curQ.toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//            if ((curOrientation.acquisitionTime == 0) || (Float.isNaN(curOrientation.firstAngle)))
//            {
//                // This one bad too?  Fatal, we need to stop auto
//                BotLog.logD("imu_Fatal", "%s %s", curOrientation.acquisitionTime, curOrientation);
//                imuFatal = true;
//                // Stop auto
//            }
//            curHeading = curOrientation.firstAngle;
//            // For recovery, we are going to ass-u-me the current AngularVel is the same as the
//            // previous AngularVel and hope our angular acceleration was insignificant.
//            if (timestamp > prevIMUTime)
//            {
//                prevHeading = curHeading + ((prevIMUAngleVel * (timestamp - prevIMUTime) / imuCorr[imuIdx]));
//            }
//            else
//            {
//                prevHeading = curHeading + (prevDelta / imuCorr[imuIdx]);
//            }
//        }
//
//        // Apply our IMU correction factor.
//        curHeading = curOrientation.firstAngle;
//        delta = AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(prevHeading - curHeading) * imuCorr[imuIdx]);
//        intHeading = AngleUnit.normalizeRadians(intHeading - delta);
//
//        // Save some values
//        prevDelta = delta;
//        prevHeading = curHeading;
//        if (timestamp > prevIMUTime)
//        {
//            prevIMUAngleVel = prevDelta / (timestamp - prevIMUTime);
//        }
//        else
//        {
//            prevIMUAngleVel = 0;
//        }
//        prevIMUTime = timestamp;
//
//        mPeriodicIO.gyro_heading = Rotation2d.fromRadians(AngleUnit.normalizeRadians(startingHeading + intHeading)).inverse();
//
//        // If the forcefail is enabled and we failed or if just enough time has passed, update out imu0 and display the delta.
//        if ((forceIMUFailover > 0) && (failedOver || IMULogTimer.hasExpired()))
//        {
//            curHeading0 = imus[0].getRobotOrientationAsQuaternion().toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//            prevDelta0 = AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(prevHeading0 - curHeading0) * imuCorr[0]);
//            intHeading0 = AngleUnit.normalizeRadians(intHeading0 - prevDelta0);
//            prevHeading0 = curHeading0;
//            BotLog.logD("imu_failover", "intHeading0:%3.1f, intHeading: %3.1f, delta: %3.1f, prevAV/s: %3.1f",
//                    Math.toDegrees(intHeading0), Math.toDegrees(intHeading), Math.toDegrees(intHeading - intHeading0), Math.toDegrees(prevIMUAngleVel));
//            IMULogTimer.reset();
//        }
//    }
//
//    public synchronized void readIMU(double timestamp)
//    {
//        double delta;
//        // We get the current Quaternion
//        curQ = imus[imuIdx].getRobotOrientationAsQuaternion();
//        Orientation curOrientation = curQ.toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//
//        if ((curOrientation.acquisitionTime == 0) || (Float.isNaN(curOrientation.firstAngle)))
//        {
//            BotLog.logD("imu_Fatal", "%s %s", curOrientation.acquisitionTime, curOrientation);
//            imuFatal = true;
//        }
//
//        // Apply our IMU correction factor.
//        curHeading = curOrientation.firstAngle;
//        delta = AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(prevHeading - curHeading) * imuCorr[imuIdx]);
//        intHeading = AngleUnit.normalizeRadians(intHeading - delta);
//        prevHeading = curHeading;
//
//        mPeriodicIO.gyro_heading = Rotation2d.fromRadians(AngleUnit.normalizeRadians(startingHeading + intHeading)).inverse();
//    }
//
//    public synchronized double IMUDeltaFromPrev()
//    {
//        // We get the current Quaternion
//        Quaternion mycurQ = imus[imuIdx].getRobotOrientationAsQuaternion();
//        Orientation mycurOrientation = mycurQ.toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//
//        // Apply our IMU correction factor.
//        double mycurHeading = mycurOrientation.firstAngle;
//        return (AngleUnit.normalizeRadians(AngleUnit.normalizeRadians(prevHeading - mycurHeading) * imuCorr[imuIdx]));
//    }
//
//    public boolean isImuFatal()
//    {
//        return imuFatal;
//    }
//
//    public synchronized void readPeriodicInputs(double timestamp)
//    {
//        if (lastTime != timestamp)
//        {
//            left_encoder_prev_ticks_ = mPeriodicIO.left_position_ticks;
//            right_encoder_prev_ticks_ = mPeriodicIO.right_position_ticks;
//
//            //mPeriodicIO.left_position_ticks = (BL.getCurrentPosition() + FL.getCurrentPosition()) / 2;
//            //mPeriodicIO.right_position_ticks = (BR.getCurrentPosition() + FR.getCurrentPosition()) / 2;
//            mPeriodicIO.left_position_ticks = FL.getCurrentPosition();
//            mPeriodicIO.right_position_ticks = FR.getCurrentPosition();
//            mPeriodicIO.left_vel = FL.getVelocity();
//            mPeriodicIO.right_vel = FR.getVelocity();
//
//            // DeadWheels
//            mPeriodicIO.lateral_encoder_prev = mPeriodicIO.lateral_encoder;
//            mPeriodicIO.parallel_encoder_prev = mPeriodicIO.parallel_encoder;
//            mPeriodicIO.lateral_encoder = 1.0 * liftF.getCurrentPosition();
//            mPeriodicIO.parallel_encoder = 1.0 * intake.getCurrentPosition();
//
//            boolean logging = false;
//            if (logging)
//            {
//                mPeriodicIO.B_left_position_ticks = BL.getCurrentPosition();
//                mPeriodicIO.B_right_position_ticks = BR.getCurrentPosition();
//                mPeriodicIO.B_left_vel = BL.getVelocity();
//                mPeriodicIO.B_right_vel = BR.getVelocity();
//
//            }
//            if (customPID)
//            {
//                addREncoderHistory(mPeriodicIO.right_position_ticks, timestamp);
//                addLEncoderHistory(mPeriodicIO.left_position_ticks, timestamp);
//            }
//
//            double leftDistanceTicks = mPeriodicIO.left_position_ticks - left_encoder_prev_ticks_;
//            double rightDistanceTicks = mPeriodicIO.right_position_ticks - right_encoder_prev_ticks_;
//
//            mPeriodicIO.left_vel_ticks = leftDistanceTicks / (timestamp - lastTime);
//            mPeriodicIO.right_vel_ticks = rightDistanceTicks / (timestamp - lastTime);
//
//            mPeriodicIO.left_distance += (leftDistanceTicks / 537.7) * Math.PI * Constants.kDriveWheelDiameterInches * Constants.kGearRatio;
//            mPeriodicIO.right_distance += (rightDistanceTicks / 537.7) * Math.PI * Constants.kDriveWheelDiameterInches * Constants.kGearRatio;
//
//            if (use_encoder_heading)
//            {
//                double delta_v = ((((rightDistanceTicks - leftDistanceTicks) / 2.0)) / 537.7) * Math.PI * Constants.kDriveWheelDiameterInches * Constants.kGearRatio;
//                double delta_a = (delta_v * 2.0 / (Constants.kDriveWheelTrackWidthInches * Constants.kTrackScrubFactor));
//                encoder_heading = AngleUnit.normalizeRadians(encoder_heading + delta_a);
//                mPeriodicIO.gyro_heading = Rotation2d.fromRadians(AngleUnit.normalizeRadians(startingHeading + encoder_heading)).inverse();
//            }
//            else
//            {
//                readIMUWithFailover(timestamp);
//            }
//
//            lastTime = timestamp;
//        }
//    }
//
//    public synchronized void writePeriodicOutputs()
//    {
//        double shutDownPower = 1.0;
//
//        if (shutdown)
//        {
//            // Start a timer any time we're in shutdown
//            ShutDownTimer.reset();
//        }
//
//        if (!ShutDownTimer.hasExpired())
//        {
//            // If the timer is still running, cut power by 50%
//            shutDownPower = 0.5;
//        }
//
//        if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveControlState == DriveControlState.TURN_TO_HEADING || mDriveControlState == DriveControlState.PID_FOLLOWING)
//        {
//            if (mPeriodicIO.left_demand != mPeriodicIO.last_left_demand || mPeriodicIO.right_demand != mPeriodicIO.last_right_demand || !ShutDownTimer.hasExpired())
//            {
//                // Multiplying by shutDownPower here instead of changing the actual demand
//                // value guarantees that we will restore full power as soon as the timer expires.
//                BL.setPower(mPeriodicIO.left_demand * shutDownPower / 1.008);
//                BR.setPower(mPeriodicIO.right_demand * shutDownPower);
//                FR.setPower(mPeriodicIO.right_demand * shutDownPower);
//                FL.setPower(mPeriodicIO.left_demand * shutDownPower / 1.008);
//                // BotLog.logD("WPI_Drive", "SetP: %5.3f, %5.3f", BL.getPower(), BR.getPower());
//
//                mPeriodicIO.last_left_demand = mPeriodicIO.left_demand * shutDownPower;
//                mPeriodicIO.last_right_demand = mPeriodicIO.right_demand * shutDownPower;
//            }
//        }
//        else if (mDriveControlState == DriveControlState.PATH_FOLLOWING)
//        {
//            if (customPID)
//            {
//                double left_pwr = mPeriodicIO.left_custom;
//                double right_pwr = mPeriodicIO.right_custom;
//                double maxAbsPwr = Math.max(Math.abs(left_pwr), Math.abs(right_pwr));
//
//                if (maxAbsPwr > 1.0)
//                {
//                    left_pwr /= maxAbsPwr;
//                    right_pwr /= maxAbsPwr;
//                }
//
//                FL.setPower(left_pwr / 1.008);
//                FR.setPower(right_pwr);
//                BR.setPower(right_pwr);
//                BL.setPower(left_pwr / 1.008);
//
//                boolean logging = false;
//                if (logging)
//                {
//                    double tgtVel = getPathSetpoint().velocity();
//                    Pose2d act = getPose();
//                    Pose2d tgt = getPathSetpoint().state().getPose();
//                    double err = tgt.distance(act);
//                    double FLV = mPeriodicIO.left_vel;
//                    double BLV = mPeriodicIO.B_left_vel;
//                    double FRV = mPeriodicIO.right_vel;
//                    double BRV = mPeriodicIO.B_right_vel;
//                    int FLE = (int) mPeriodicIO.left_position_ticks;
//                    int BLE = (int) mPeriodicIO.B_left_position_ticks;
//                    int FRE = (int) mPeriodicIO.right_position_ticks;
//                    int BRE = (int) mPeriodicIO.B_right_position_ticks;
//
//                    BotLog.logD("Vel", "tgV: %5.2f," +
//                                    " FLV: %5.2f," +
//                                    " tgA: %5.2f," +
//                                    " dLV: %5.2f, dRV: %5.2f," +
//                                    " lpw: %5.2f, rpw: %5.2f," +
//                                    " spw: %5.2f, vpw: %5.2f, apw: %f," +
//                                    " err: %5.2f," +
//                                    // " FLV: %5.2f, BLV: %5.2f," +
//                                    // " dFV: %5.2f," +
//                                    //" FRV: %5.2f, BRV: %5.2f," +
//                                    // " dBV: %5.2f," +
//                                    //" FLE: %5d, BLE: %5d," +
//                                    // " dFE: %5d," +
//                                    //" FRE: %5d, BRE: %5d," +
//                                    // " dBE: %5d," +
//                                    " tgt: %s vs act: %s",
//                            tgtVel,
//                            rotationsToInches(FLV / 537.7),
//                            leftFF.pa / leftFF.ka,
//                            radToTicks(mPeriodicIO.left_demand), radToTicks(mPeriodicIO.right_demand),
//                            left_pwr, right_pwr,
//                            leftFF.ps, leftFF.pv, leftFF.pa,
//                            err,
//                            // FLV, BLV,
//                            //FLV - BLV,
//                            //FRV, BRV,
//                            //FRV - BRV,
//                            //FLE, BLE,
//                            //FLE - BLE,
//                            //FRE, BRE,
//                            //FRE - BRE,
//                            tgt, act);
//                }
//            }
//            else
//            {
//                if (mPeriodicIO.right_demand_final != mPeriodicIO.last_right_demand_final || mPeriodicIO.left_demand_final != mPeriodicIO.last_left_demand_final)
//                {
//                    FL.setVelocity(mPeriodicIO.left_demand_final / 1.008);
//                    FR.setVelocity(mPeriodicIO.right_demand_final);
//                    BR.setVelocity(mPeriodicIO.right_demand_final);
//                    BL.setVelocity(mPeriodicIO.left_demand_final / 1.008);
//                    //BotLog.logD("WPI_Drive", "SetV: %5.3f, %5.3f", mPeriodicIO.left_demand_final * 1.0/1.008, mPeriodicIO.right_demand_final);
//
//                    mPeriodicIO.last_left_demand_final = mPeriodicIO.left_demand_final;
//                    mPeriodicIO.last_right_demand_final = mPeriodicIO.right_demand_final;
//
//                    // BotLog.logD("Pwr", "LPwr: %5.2f, RPwr: %5.2f", mPeriodicIO.left_custom, mPeriodicIO.right_custom);
//                }
//            }
//        }
//        else if (mDriveControlState == DriveControlState.GF_FOLLOWING)
//        {
//            if (mPeriodicIO.left_demand != mPeriodicIO.last_left_demand || mPeriodicIO.right_demand != mPeriodicIO.last_right_demand || !ShutDownTimer.hasExpired())
//            {
//                BL.setPower(mPeriodicIO.left_demand / 1.008);
//                BR.setPower(mPeriodicIO.right_demand);
//                FR.setPower(mPeriodicIO.right_demand);
//                FL.setPower(mPeriodicIO.left_demand / 1.008);
//
//                mPeriodicIO.last_left_demand = mPeriodicIO.left_demand;
//                mPeriodicIO.last_right_demand = mPeriodicIO.right_demand;
//            }
//        }
//
//        writePixelOutputs();
//    }
//
//    public synchronized void writePixelOutputs()
//    {
//        if (mPeriodicIO.left_pixel_servo_position != mPeriodicIO.prev_left_pixel_servo_position)
//        {
//            pixelLeft.setPosition(mPeriodicIO.left_pixel_servo_position);
//            mPeriodicIO.prev_left_pixel_servo_position = mPeriodicIO.left_pixel_servo_position;
//        }
//
//        if (mPeriodicIO.right_pixel_servo_position != mPeriodicIO.prev_right_pixel_servo_position)
//        {
//            pixelRight.setPosition(mPeriodicIO.right_pixel_servo_position);
//            mPeriodicIO.prev_right_pixel_servo_position = mPeriodicIO.right_pixel_servo_position;
//        }
//    }
//
//    public double getXPos()
//    {
//        return robot_state_.getLatestFieldToVehicle().getTranslation().x();
//    }
//
//    public double getYPos()
//    {
//        return robot_state_.getLatestFieldToVehicle().getTranslation().y();
//    }
//
//    public double getHeadingDeg()
//    {
//        return robot_state_.getLatestFieldToVehicle().getRotation().getDegrees();
//    }
//
//    public double getHeadingRad()
//    {
//        return robot_state_.getLatestFieldToVehicle().getRotation().getRadians();
//    }
//
//    public Pose2d getError()
//    {
//        return mPeriodicIO.error;
//    }
//
//    public double getSimXPos()
//    {
//        return lastPose.getTranslation().x();
//    }
//
//    public double getSimYPos()
//    {
//        return lastPose.getTranslation().y();
//    }
//
//    public double getSimHeadingDeg()
//    {
//        return lastPose.getRotation().getDegrees();
//    }
//
//
//    public String getTelem(double timestamp)
//    {
//        String output = RobotState.getInstance().getFieldToVehicle().toString() + "\n";
//        output += "state :: " + mDriveControlState + "\n";
//        output += "time :: " + timestamp + "\n";
//        output += "linear vel :: " + getLinearVelocityInches() + "\n";
//        output += "target vel :: " + (Constants.kDriveWheelRadiusInches * ((mPeriodicIO.left_demand + mPeriodicIO.right_demand) / 2.0)) + "\n";
//        output += "angular vel :: " + getAngularVelocity() + "\n";
//        output += "left_demand_final :: " + mPeriodicIO.left_demand_final + "\n";
//        output += "right_demand_final :: " + mPeriodicIO.right_demand_final + "\n";
//        output += "left_demand :: " + mPeriodicIO.left_demand + "\n";
//        output += "right_demand :: " + mPeriodicIO.right_demand + "\n";
//        output += "left_feedforward :: " + mPeriodicIO.left_feedforward + "\n";
//        output += "right_feedforward :: " + mPeriodicIO.right_feedforward + "\n";
//        output += "left_accel :: " + mPeriodicIO.left_accel + "\n";
//        output += "right_accel :: " + mPeriodicIO.right_accel + "\n";
//        output += "error :: " + mPeriodicIO.error + "\n";
//        output += "path_setpoint :: " + mPeriodicIO.path_setpoint + "\n";
//
//        return output;
//    }
//
//    // The robot drivetrain's various states.
//    public enum DriveControlState
//    {
//        OPEN_LOOP,
//        PATH_FOLLOWING,
//        GF_FOLLOWING,
//        TURN_TO_HEADING,
//        PID_FOLLOWING,
//    }
//
//
//    public static class DriveSignal
//    {
//        private double mLeftMotor;
//        private double mRightMotor;
//        private boolean mBrakeMode;
//
//        public DriveSignal(double left, double right)
//        {
//            this(left, right, false);
//        }
//
//        public DriveSignal(double left, double right, boolean brakeMode)
//        {
//            mLeftMotor = left;
//            mRightMotor = right;
//            mBrakeMode = brakeMode;
//        }
//
//        public static DriveSignal NEUTRAL = new DriveSignal(0, 0);
//        public static DriveSignal BRAKE = new DriveSignal(0, 0, true);
//
//        public double getLeft()
//        {
//            return mLeftMotor;
//        }
//
//        public double getRight()
//        {
//            return mRightMotor;
//        }
//
//        public boolean getBrakeMode()
//        {
//            return mBrakeMode;
//        }
//
//        @Override
//        public String toString()
//        {
//            return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
//        }
//    }
//
//    public static class PeriodicIO
//    {
//        // INPUTS
//        public double lateral_encoder;
//        public double parallel_encoder;
//        public double lateral_encoder_prev;
//        public double parallel_encoder_prev;
//
//        public double left_position_ticks; // ticks that the left motors have spun
//        public double right_position_ticks; // ticks that the right motors have spun
//        public double B_left_position_ticks; // ticks that the left motors have spun
//        public double B_right_position_ticks; // ticks that the right motors have spun
//        public double left_vel;
//        public double right_vel;
//        public double B_left_vel;
//        public double B_right_vel;
//        public double left_distance; // in that the left side of the dt has moved
//        public double right_distance; // in that the right side of the dt has moved
//        public double left_vel_ticks; // velocity of the left motors in ticks
//        public double right_vel_ticks; // velocity of the right motors in ticks
//        public Rotation2d gyro_heading = Rotation2d.identity();
//        public Pose2d error = Pose2d.identity();
//
//        // OUTPUTS
//        public double left_demand_final; // stupid hack
//        public double right_demand_final; // stupid hack v2;
//        public double last_left_demand_final = -1; // stupid hack
//        public double last_right_demand_final = -1; // stupid hack v2;
//        public double last_left_demand = -1; // stupid hack
//        public double last_right_demand = -1; // stupid hack v2
//        public double left_demand; // the rad/sec that the left wheel should move at
//        public double right_demand; // the rad/sec that the right wheel should move at
//        public double left_custom = 0;
//        public double right_custom = 0;
//        public double left_accel;
//        public double right_accel;
//        public double left_feedforward;
//        public double right_feedforward;
//        public double left_pixel_servo_position; // position of left purple pixel servo
//        public double prev_left_pixel_servo_position = -1; // last position of left purple pixel servo
//        public double right_pixel_servo_position; // position of right purple pixel servo
//        public double prev_right_pixel_servo_position = -1; // last position of right purple pixel servo
//        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
//    }
//
//    public double average2(double a1, double a2)
//    {
//        double bisect1 = AngleUnit.normalizeRadians((a1 + a2) / 2.0);
//        double bisect2 = bisect1 + Math.toRadians(180.0);
//
//        double delta1 = Math.abs(AngleUnit.normalizeRadians(bisect1 - a1));
//        double delta2 = Math.abs(AngleUnit.normalizeRadians(bisect2 - a1));
//
//        return (delta1 < delta2) ? bisect1 : bisect2;
//    }
//
//    public double average3(double a1, double a2, double a3)
//    {
//        double sins = Math.sin(a1) + Math.sin(a2) + Math.sin(a3);
//        double coss = Math.cos(a1) + Math.cos(a2) + Math.cos(a3);
//        double avg = Math.atan2(sins, coss);
//        return (avg);
//    }
//
//    public Pose2d relocH(Robot robot, String id, boolean reloc)
//    {
//        Pose2d retPose = robot.mDrive.getPose();
//
//        boolean enableRelocH = false;
//        if (enableRelocH)
//        {
//            double goldenHeading;
//            robot.mDrive.logIMUs();
//
//            Orientation orien[] = new Orientation[IMUCount];
//            double heading[] = new double[IMUCount];
//            int other1 = (imuIdx + 1) % 3;
//            int other2 = (imuIdx + 2) % 3;
//
//            for (int i = 0; i < imus.length; i++)
//            {
//                orien[i] = imus[i].getRobotOrientationAsQuaternion().toOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//                heading[i] = (orien[i].firstAngle * imuCorr[i]) - headingOffset[i];
//            }
//
//            goldenHeading = heading[imuIdx];
//
//            if ((Math.abs(AngleUnit.normalizeRadians(heading[imuIdx] - heading[other1])) < Math.toRadians(0.5)) || (Math.abs(AngleUnit.normalizeRadians(heading[imuIdx] - heading[other2])) < Math.toRadians(0.5)))
//            {
//                BotLog.logD("RelocH", "%s, Use0: %5.2f, 0: %5.2f, 1: %5.2f, 2: %5.2f", id, Math.toDegrees(goldenHeading), Math.toDegrees(heading[imuIdx]), Math.toDegrees(heading[other1]), Math.toDegrees(heading[other2]));
//                return (retPose);
//            }
//            else
//            {
//                if (Math.abs(AngleUnit.normalizeRadians(heading[other1] - heading[other2])) < Math.toRadians(0.5))
//                {
//                    // The other 2 agree, use the average
//                    goldenHeading = average2(heading[other1], heading[other2]);
//                    BotLog.logD("RelocH", "%s Avg2: %5.2f, 0: %5.2f, 1: %5.2f, 2: %5.2f", id, Math.toDegrees(goldenHeading), Math.toDegrees(heading[imuIdx]), Math.toDegrees(heading[other1]), Math.toDegrees(heading[other2]));
//
//                }
//                else
//                {
//                    goldenHeading = average3(heading[imuIdx], heading[other1], heading[other2]);
//                    BotLog.logD("RelocH", "%s Avg3: %5.2f, 0: %5.2f, 1: %5.2f, 2: %5.2f", id, Math.toDegrees(goldenHeading), Math.toDegrees(heading[imuIdx]), Math.toDegrees(heading[other1]), Math.toDegrees(heading[other2]));
//                }
//            }
//            if (reloc)
//            {
//                // Limit the correction to a max of 1 degree for now
//                double delta = AngleUnit.normalizeRadians(goldenHeading - heading[imuIdx]);
//                if (Math.abs(delta) > Math.toRadians(1.0))
//                {
//                    delta = Math.toRadians(1.0) * Math.signum(delta);
//                }
//                goldenHeading = AngleUnit.normalizeRadians(heading[imuIdx] + delta);
//                prevHeading = AngleUnit.normalizeRadians(prevHeading + AngleUnit.normalizeRadians(heading[imuIdx] - goldenHeading));
//                headingOffset[imuIdx] = AngleUnit.normalizeRadians(headingOffset[imuIdx] + AngleUnit.normalizeRadians(heading[imuIdx] - goldenHeading));
//                headingOffset[other1] = AngleUnit.normalizeRadians(heading[other1] + AngleUnit.normalizeRadians(heading[other1] - goldenHeading));
//                headingOffset[other2] = AngleUnit.normalizeRadians(heading[other2] + AngleUnit.normalizeRadians(heading[other2] - goldenHeading));
//                relocH(robot, id, reloc);
//            }
//            retPose = new Pose2d(retPose.getTranslation(), Rotation2d.fromRadians(goldenHeading));
//            BotLog.logD("RelocH", "%s %s %s", id, robot.mDrive.getPose(), retPose);
//        }
//        return (retPose);
//    }
//}