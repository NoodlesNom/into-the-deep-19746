package org.firstinspires.ftc.teamcode.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "liftPIDTuner")
public class liftPIDTuner extends LinearOpMode {

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

    private Lift lift;
    private ElapsedTime myTimer;

    private int liftTicks = 1;
    private int liftTicksMax = 900;
    private int smallTicks = 100;
    private int bigTicks = 200;
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
            liftTicks += liftMiniIncStickyButton.getState() ? 100 : 0;

            liftMiniDecStickyButton.update(gamepad1.dpad_left);
            liftTicks -= liftMiniDecStickyButton.getState() ? 100 : 0;

            liftIncStickyButton.update(gamepad1.dpad_up);
            liftTicks += liftIncStickyButton.getState() ? 200 : 0;

            liftDecStickyButton.update(gamepad1.dpad_down);
            liftTicks -= liftDecStickyButton.getState() ? 200 : 0;

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

        waitForStart();

        if (isStopRequested()) return;

        myTimer.reset();
        telemetry.clearAll();
        telemTimer.reset();

        while (!isStopRequested()) {
            mapControls();
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

            telemetry.addData("lift Enc: ", lift.getLiveRightLiftPosition());
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