package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.concurrent.TimeUnit;
@Config
@TeleOp(name = "deposit+lift+intake tester")
public class deposit_lift_intake_tester extends LinearOpMode {

    // * Xbox/PS4 Button - Motor
    // *   Y / Δ         - Up 1 Idx
    // *   A / X         - Down 1 Idx
    // *   X / ▢         - Nothing
    // *   B / O         - Nothing
    // *
    // *                   / ______ \
    // *     ------------.-'   _  '-..+
    // *              /   _  ( Y )  _  \
    // *             |  ( X )  _  ( B ) |
    // *        ___  '.      ( A )     /|
    // *      .'    '.    '-._____.-'  .'
    // *     |       |                 |
    // *      '.___.' '.               |
    // *               '.             /
    // *                \.          .'
    // *                  \________/
    // *
    private StickyButton liftIdxIncStickyButton = new StickyButton();
    private StickyButton liftIdxDecStickyButton = new StickyButton();

    private StickyButton liftIncStickyButton = new StickyButton();
    private StickyButton liftDecStickyButton = new StickyButton();

    private StickyButton liftMiniIncStickyButton = new StickyButton();
    private StickyButton liftMiniDecStickyButton = new StickyButton();

    private StickyButton liftResetStickyButton = new StickyButton();
    private StickyButton liftModeStickyButton = new StickyButton();

    private ServoImplEx claw;
    private ServoImplEx pivotL;
    private ServoImplEx pivotR;

    private ServoImplEx intake;
    private ServoImplEx diffyL;
    private ServoImplEx diffyR;
    public static double upper = 2400;
    public static double lower = 500;

    public static int pitch = 0;
    public static int roll = 0;

    public static double pivot_pos = 0.5;
    public static double claw_pos = 0.3;
    public static double intakepos = 0.1;
    public static boolean move = false;

    private Lift lift;
    private ElapsedTime myTimer;

    public static int liftTicks = 1;
    private int liftTicksMax = 900;
    private int smallTicks = 10;
    private int bigTicks = 50;
    private int liftPosIdx = 0;
    private boolean idxMode = true;

    private Deadline telemTimer = new Deadline(100, TimeUnit.MILLISECONDS);

    private void mapControls() {

        liftResetStickyButton.update(gamepad1.left_bumper);
        liftPosIdx = liftResetStickyButton.getState() ? 0 : liftPosIdx;
        liftTicks = liftResetStickyButton.getState() ? 0 : liftTicks;

        liftModeStickyButton.update(gamepad1.right_bumper);
        idxMode = liftModeStickyButton.getState() ? !idxMode : idxMode;

        if(idxMode)
        {

            // Manage the LiftPosIdx controls
            liftIdxIncStickyButton.update(gamepad1.y);
            liftPosIdx += liftIdxIncStickyButton.getState() ? 1 : 0;

            liftIdxDecStickyButton.update(gamepad1.a);
            liftPosIdx -= liftIdxDecStickyButton.getState() ? 1 : 0;

            // Bounds check
            liftPosIdx = Range.clip(liftPosIdx, 0, Lift.LIFT_POS.MAX.getVal());
        }
        else
        {
            liftMiniIncStickyButton.update(gamepad1.dpad_right);
            liftTicks += liftMiniIncStickyButton.getState() ? smallTicks : 0;

            liftMiniDecStickyButton.update(gamepad1.dpad_left);
            liftTicks -= liftMiniDecStickyButton.getState() ? smallTicks : 0;

            liftIncStickyButton.update(gamepad1.dpad_up);
            liftTicks += liftIncStickyButton.getState() ? bigTicks : 0;

            liftDecStickyButton.update(gamepad1.dpad_down);
            liftTicks -= liftDecStickyButton.getState() ? bigTicks : 0;

            // Bounds check
            liftTicks = Range.clip(liftTicks, 1, liftTicksMax);
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {

        lift = new Lift(hardwareMap);
        myTimer = new ElapsedTime();

        lift.teleopInit();

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();
        pivotL = hardwareMap.get(ServoImplEx .class, "pivotL");
        pivotR = hardwareMap.get(ServoImplEx .class, "pivotR");
        intake = hardwareMap.get(ServoImplEx.class, "pivot");
        pivotR.setDirection(ServoImplEx.Direction.REVERSE);
        pivotL.setPosition(0.447);
        pivotR.setPosition(0.467);

        claw = hardwareMap.get(ServoImplEx .class, "claw");
        claw.setPosition(claw_pos);
        diffyL = hardwareMap.get(ServoImplEx .class, "diffyL");
        diffyR = hardwareMap.get(ServoImplEx .class, "diffyR");
        diffyR.setDirection(ServoImplEx.Direction.REVERSE);
        diffyL.setPosition(0.5);
        diffyR.setPosition(0.5);


        waitForStart();
        timer.reset();

        if (isStopRequested()) return;

        myTimer.reset();
        telemetry.clearAll();
        telemTimer.reset();

        while (!isStopRequested()) {
            mapControls();
            if (move) {
                claw.setPosition(claw_pos);
                pivotL.setPosition(pivot_pos - 0.02);
                pivotR.setPosition(pivot_pos);

                double targetL = 0.5+((pitch/340.0)+(roll/320.0));
                double targetR = 0.5+((pitch/340.0)-(roll/320.0));
                intake.setPosition(intakepos);
                diffyL.setPosition(targetL);
                diffyR.setPosition(targetR);
            }
            pivotL.setPwmRange(new PwmControl.PwmRange(lower,upper));
            pivotR.setPwmRange(new PwmControl.PwmRange(lower,upper));
            telemetry.addLine("Press buttons to adjust lift position");
            telemetry.addLine();
            telemetry.addLine("Xbox/PS4 Button - Action");
            telemetry.addLine("lft_bumper: reset position to 0");
            telemetry.addLine("rt_bumper : Change mode");

            if(idxMode)
            {
                telemetry.addLine("Idx Mode");
                telemetry.addLine("Y         : +1 Idx");
                telemetry.addLine("A         : -1 Idx");
                telemetry.addData("lift Pos   : ", liftPosIdx);
            } else {
                telemetry.addLine("Ticks Mode");
                telemetry.addLine(String.format("dpad_right: +%4d ticks", smallTicks));
                telemetry.addLine(String.format("dpad_left : -%4d ticks", smallTicks));
                telemetry.addLine(String.format("dpad_up   : +%4d ticks", bigTicks));
                telemetry.addLine(String.format("dpad_down : -%4d ticks", bigTicks));
                telemetry.addData("lift Ticks : ", liftTicks);
            }

            telemetry.addData("lift Enc: ", lift.getLiveLiftPosition());
            telemetry.addLine();

            telemetry.update();

            if (idxMode)
            {
                lift.setTargetPos(liftPosIdx, myTimer.seconds());
            } else {
                lift.setTargetTicks(liftTicks, myTimer.seconds());
            }
            lift.update(myTimer.seconds());

            boolean debug = true;
            if(debug)
            {
                if( telemTimer.hasExpired() )
                {
                    BotLog.logD("LiftTuner", lift.getTelem(myTimer.seconds()));
                    telemTimer.reset();
                }
            }
        }
    }
}