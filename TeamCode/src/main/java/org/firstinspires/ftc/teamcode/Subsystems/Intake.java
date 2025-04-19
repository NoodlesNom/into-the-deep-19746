package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.MiniPID;
@Config
public class Intake extends Subsystem {

    // Hardware
    private DcMotorEx intake;
    private DcMotorEx extendo;
    private ServoImplEx pivot;

    //private AnalogInput pivotEncoder;

    private Servo claw;


    private int redDetections =0;
    private int greenDetections =0;
    private int blueDetections =0;
    private RevColorSensorV3 color;
    private ExtendoControlState mExtendoControlState;

    private double tgtTicks;
    private double pidTgtTicks;
    private double PIDTime = 0.025;
    //private double PIDTime = 0.05;    // "we need to look at the changes" -/@ coach todd
    private double nextPID;
    private int PIDCount = 0;
    private int PIDSkipCount = 0;
    private double prevPIDTime = 0.0;
    public static double RedThreshold = 1.25;
    public static double BlueThreshold = 2.25;
    public static double GreenThreshold = 1;
    private MiniPID pid;
    public static double P = 0.0075/ 1 ;
    public static double I = 0.0015 / 1;
    public static double D = 0.045 *1;

    public static double crush = 0.2;
    public static double F = 0;
    private double vF = F;
    public static double MAX_EXTENDO_PWR = 1;
    public static double MIN_EXTENDO_PWR = -0.9;
    private Servo gate;

    // Hardware states
    private PeriodicIO mPeriodicIO;

    private double[] extendoPos = new double[] {1,170,350, 650, 440, 380, 650-5/0.033};

    public enum EXTEND_POS
    {
        //Constants with values
        STOWED(0),
        TRANSFER(1),
        GETOUT(2),
        INTAKING(3),
        AUTOINTAKEPREPARE(4),
        AUTOINTAKEPREPARESHORT(5),
        INTAKINGSHORT(6);

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
    //0.565 with fixed intake
    public static double[] pivotPos = new double[] {0, 0.63, 0.12, 0.30, 0.30, 0.30, 0.495, 0.395, 0.55, 0.155, 0.63, 0.145};
    public enum PIVOT_POS
    {
        //Constants with values
        INTAKING(1),
        LAUNCH(2), 
        IDLE(3),
        TRANSFER(4),
        TRAP(5),
        AUTOINIT(6),
        INTAKEPREP(7),
        BLOCKCLEAR(8),
        HANG(9),
        INTAKINGTALL(10),
        SHOOTLOW(11);

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

    private double[] clawPos = new double[]{0.31, 0.49};

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
    private double[] gatePos = new double[] {0.89, 0.89, 0.2};
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
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendo = map.get(DcMotorEx.class, "extendo");
        extendo.setDirection(DcMotorEx.Direction.REVERSE);
        //extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        claw = map.get(Servo.class, "intakeclaw");

        pid = new MiniPID(P, I, D, F);
        pid.ReduceErrorAtSetpoint = crush;
        pid.reset();
        pid.setOutputLimits(MIN_EXTENDO_PWR, MAX_EXTENDO_PWR);
        pid.setOutputRampRate(0.35);
        vF = F;
        pid.setPID(P, I, D, vF);

        color = map.get(RevColorSensorV3.class, "color");
        color.enableLed(true);


        nextPID = 0;

        setExtendoOpenLoop(0);
        //intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intake.setVelocityPIDFCoefficients(10 * Math.PI, 3 * Math.PI, 0, 3 * Math.PI);

        // servos
        pivot = map.get(ServoImplEx.class, "pivot");

        //pivotEncoder = map.get(AnalogInput.class, "pivot encoder");


        gate = map.get(Servo.class, "gate");

        setIntakeOpenLoop(0);
    }

    @Override
    public void autoInit()
    {
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setGatePos(GATE_POS.CATCH.getVal());
        setPivotPos(PIVOT_POS.LAUNCH.getVal());
    }

    @Override
    public void teleopInit()
    {
        setGatePos(GATE_POS.CATCH.getVal());
        setPivotPos(PIVOT_POS.IDLE.getVal());
    }

    public void resetExtendoTicks(){
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        extendo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mPeriodicIO.extendo_demand = power;
    }

    public  void setPivotPos(int pos)
    {
        mPeriodicIO.pivot_pos = pos;
        //BotLog.logD(" Intake 1 :: ", "Pos %d, %5.2f", pos, intakePos[mPeriodicIO.pivot_pos]  );
    }

    public  void setClawPos(int pos)
    {
        mPeriodicIO.claw_pos = pos;
        //BotLog.logD(" Intake 1 :: ", "Pos %d, %5.2f", pos, intakePos[mPeriodicIO.pivot_pos]  );
    }

    public void setExtendoTicks(int tgtTicksArg, double timestamp)
    {
        mExtendoControlState = ExtendoControlState.PID_CONTROL;
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (tgtTicksArg != tgtTicks)
        {
            tgtTicks = tgtTicksArg;

            PIDCount = 0;
            PIDSkipCount = 0;
            nextPID = timestamp;
            pid.reset();
            updateExtendoPID(timestamp);
        }
    }

    public void setOutputLimits(double min, double max){
        if (min!=MIN_EXTENDO_PWR||max!=MAX_EXTENDO_PWR) {
            MIN_EXTENDO_PWR = min;
            MAX_EXTENDO_PWR = max;
            pid.setOutputLimits(MIN_EXTENDO_PWR, MAX_EXTENDO_PWR);
        }
    }

    public void pwmdisable(){
        if (pivot.isPwmEnabled()) {
            pivot.setPwmDisable();
        }
    }
    public void pwmenable(){
        if (!pivot.isPwmEnabled()) {

            pivot.setPwmEnable();
        }
    }
    public  void setExtendoPos(int tgtPosArg, double timestamp)
    {
        tgtPosArg = Math.min(tgtPosArg,(extendoPos.length-1));
        double tgtPosArgTicks = extendoPos[tgtPosArg];

        mExtendoControlState = ExtendoControlState.PID_CONTROL;
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            BotLog.logD("EXTENDOPID :: ", "Unexpected times %5.2f, %5.2f, %5.2f", timestamp, prevPIDTime, PIDTime);

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

                if((tgtTicks < 15) && (mPeriodicIO.lastExtendoTicks < 15))
                {
                    PIDCount = 0;
                    PIDSkipCount = 0;
                    pid.reset();
                    power = 0.0;
                }

                // power = Range.clip(power, -MAX_EXTENDO_UP_PWR, MAX_EXTENDO_UP_PWR);
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

    public double getLiveExtendoPosition()
    {
        return extendo.getCurrentPosition();
    }
    public double getExtendoPosition()
    {
        return mPeriodicIO.lastExtendoTicks;
    }

    public double getPivotEncoderPos()
    {
        return mPeriodicIO.pivotEncoderPos;
    }

    public double getTargetExtendoPosition()
    {
        return extendoPos[mPeriodicIO.extendo_pos];
    }
    public double getTargetExtendoIdx()
    {
        return mPeriodicIO.extendo_pos;
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

    public double getIntakeCurrent()
    {
        return mPeriodicIO.intake_current;
    }
    public double getExtendoCurrent()
    {
        return mPeriodicIO.extendo_current;
    }
    public ExtendoState getExtendoState()
    {
        if (closeEnough())
        {
            return ExtendoState.CLOSE_ENOUGH;
        }
        else
        {
            return ExtendoState.MOVING;
        }
    }

    public ExtendoControlState getExtendoControlState()
    {
        return mExtendoControlState;
    }
    public boolean closeEnough()
    {
        boolean nonZero = Math.abs(tgtTicks - mPeriodicIO.lastExtendoTicks) <= 20 && Math.abs(mPeriodicIO.lastExtendoVel) < 200; // TODO: What is a resonable velocity
        boolean nearZero = ( (Math.abs(tgtTicks - mPeriodicIO.lastExtendoTicks) <= 20) || (mPeriodicIO.lastExtendoTicks<0) )  && Math.abs(mPeriodicIO.lastExtendoVel) < 100; // TODO: What is a resonable velocity

        if (tgtTicks < 20) {
            return nearZero;
        } else {
            return nonZero;
        }
    }

    public boolean closeEnoughAuto()
    {
        boolean nonZero = Math.abs(tgtTicks - mPeriodicIO.lastExtendoTicks) <= 20; // TODO: What is a resonable velocity
        boolean nearZero = ( (Math.abs(tgtTicks - mPeriodicIO.lastExtendoTicks) <= 20) || (mPeriodicIO.lastExtendoTicks<0) ); // TODO: What is a resonable velocity

        if (tgtTicks < 20) {
            return nearZero;
        } else {
            return nonZero;
        }
    }

    public boolean past()
    {
        double prevpos = extendoPos[mPeriodicIO.prevExtend_pos];
        double currpos = extendoPos[mPeriodicIO.extendo_pos];
        return ((prevpos<currpos&&mPeriodicIO.lastExtendoTicks>currpos)||(prevpos>currpos&&mPeriodicIO.lastExtendoTicks<currpos));
    }

    public  void rezero()
    {
        setExtendoOpenLoop(0);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public  void zerofinish(double time)
    {
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //public void setReadVelocity(boolean on) { readVelocity = on; }
    // With deadwheels, we are reclaiming the intake encoder so no more velocity

    public void readPeriodicInputs(double timestamp)
    {
        mPeriodicIO.intake_current = 0;//intake.getCurrent(CurrentUnit.AMPS);
        mPeriodicIO.extendo_current = 0;//extendo.getCurrent(CurrentUnit.AMPS);
        mPeriodicIO.prevLastExtendoTicks = mPeriodicIO.lastExtendoTicks;
        mPeriodicIO.prevLastExtendoVel = mPeriodicIO.lastExtendoVel;
        //mPeriodicIO.pivotEncoderPos = pivotEncoder.getVoltage()/3.3*360;

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

        if(mPeriodicIO.prevClaw_pos != mPeriodicIO.claw_pos) {
            claw.setPosition(clawPos[mPeriodicIO.claw_pos]);
            mPeriodicIO.prevClaw_pos = mPeriodicIO.claw_pos;
        }

        if(mPeriodicIO.prevGate_pos != mPeriodicIO.gate_pos) {
            //gate.setPosition(gatePos[mPeriodicIO.gate_pos]);
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

    public boolean  detectedBlue(){
        NormalizedRGBA colors = color.getNormalizedColors();
        if ((colors.blue / colors.red) > BlueThreshold){
            blueDetections++;
        }else{
            blueDetections = 0;
        }
        if (blueDetections>4){
            blueDetections = 4;
        }
        return blueDetections == 4;
    }
    public boolean  detectedRed(){
        NormalizedRGBA colors = color.getNormalizedColors();
        if ((colors.red / colors.blue) > RedThreshold && (colors.red / colors.green) > GreenThreshold){
            redDetections++;
        }else{
            redDetections = 0;
        }
        if (redDetections>4){
            redDetections = 4;
        }
        return redDetections == 4;
    }

    public boolean  detectedYellow(){
        NormalizedRGBA colors = color.getNormalizedColors();
        if ((colors.red / colors.blue) > RedThreshold && (colors.red / colors.green) < GreenThreshold){
            greenDetections++;
        }else{
            greenDetections = 0;
        }
        if (greenDetections>4){
            greenDetections = 4;
        }
        return greenDetections == 4;
    }



    public boolean  detectedImmediateBlue(){
        NormalizedRGBA colors = color.getNormalizedColors();
        if ((colors.blue / colors.red) > BlueThreshold){
            return true;
        }else{
            return false;
        }
    }
    public boolean  detectedImmeditaeRed(){
        NormalizedRGBA colors = color.getNormalizedColors();
        if ((colors.red / colors.blue) > RedThreshold && (colors.red / colors.green) > GreenThreshold){
            return true;
        }else{
            return false;
        }
    }

    public boolean  detectedImmediateYellow(){
        NormalizedRGBA colors = color.getNormalizedColors();
        if ((colors.red / colors.blue) > RedThreshold && (colors.red / colors.green) < GreenThreshold){
            return true;
        }else{
            return false;
        }
    }
    @Override
    public String getTelem(double time)
    {
        boolean debug = false;
        pid.logging = debug;
        String output = "";
        if( debug ) {
            output =   " intake.pwr  :: " + mPeriodicIO.intake_demand + "\n";
            output +=  " extend.pwr  :: " + mPeriodicIO.extendo_demand + "\n";
            output +=  " extend.pos  :: " + mPeriodicIO.lastExtendoTicks + "\n";
            output +=  " extend.tgt  :: " + tgtTicks + "\n";
            output +=  " extend.ptgt :: " + pidTgtTicks + "\n";
            output +=  "  pivot.tgt  :: " + pivot.getPosition() + "\n";
            output +=  "   gate.tgt  :: " +  gate.getPosition() + "\n";
            if(pid.logging)
            {
                output += " extend.pid :: " + pid.getTelem();
            }
        }

        return output;
    }
    @Override
    public String getDemands(){
        boolean debug = true;
        String output = "";
        if( debug ) {
            output =  "   intake.pwr  :: " + mPeriodicIO.intake_demand + "\n";
            //output += "   intake.amps  :: " + mPeriodicIO.intake_current + "\n";
            output += "   extendo.pwr  :: " + mPeriodicIO.extendo_demand + "\n";
            //output += "   extendo.amps  :: " + mPeriodicIO.extendo_current + "\n";
        }
        return output;
    }


    public double getPowerDemandSum(){
        return Math.abs(mPeriodicIO.extendo_demand)+ Math.abs(mPeriodicIO.intake_demand);
    }



    public static class PeriodicIO {
        public int pivot_pos;

        public int claw_pos;
        public int gate_pos;

        public double intake_current = 0;
        public double extendo_current = 0;
        public int extendo_pos;

        public double lastExtendoTicks;
        public double lastExtendoVel;

        public double prevLastExtendoTicks;
        public double prevLastExtendoVel;
        public double extendo_demand;
        public double prevextendo_demand = -1;
        public double intake_demand;

        public int prevPivot_pos = -1;

        public double pivotEncoderPos =-1;

        public int prevClaw_pos = -1;
        public int prevGate_pos = -1;
        public int prevExtend_pos = -1;
        public double prevIntake_demand = -2;

    }
}
