package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.concurrent.TimeUnit;
@Config
@Disabled
@TeleOp(name = "extendoPIDTuner")
public class extendoPIDTuner extends LinearOpMode {

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
    private StickyButton extendoIdxIncStickyButton = new StickyButton();
    private StickyButton extendoIdxDecStickyButton = new StickyButton();

    private StickyButton extendoIncStickyButton = new StickyButton();
    private StickyButton extendoDecStickyButton = new StickyButton();

    private StickyButton extendoMiniIncStickyButton = new StickyButton();
    private StickyButton extendoMiniDecStickyButton = new StickyButton();

    private StickyButton extendoResetStickyButton = new StickyButton();
    private StickyButton extendoModeStickyButton = new StickyButton();

    private Intake extendo;
    private ElapsedTime myTimer;

    public static int extendoTicks = 1;
    public static boolean  inches = true;
    private int extendoTicksMax = 900;
    private int smallTicks = 100;
    private int bigTicks = 200;

    public static int pivot = 8;
    private int extendoPosIdx = 0;
    private boolean idxMode = true;

    private Deadline telemTimer = new Deadline(100, TimeUnit.MILLISECONDS);

    private void mapControls() {

        extendoResetStickyButton.update(gamepad1.left_bumper);
        extendoPosIdx = extendoResetStickyButton.getState() ? 0 : extendoPosIdx;
        extendoTicks = extendoResetStickyButton.getState() ? 0 : extendoTicks;

        extendoModeStickyButton.update(gamepad1.right_bumper);
        idxMode = extendoModeStickyButton.getState() ? !idxMode : idxMode;

        if(idxMode)
        {

            // Manage the extendoPosIdx controls
            extendoIdxIncStickyButton.update(gamepad1.y);
            extendoPosIdx += extendoIdxIncStickyButton.getState() ? 1 : 0;

            extendoIdxDecStickyButton.update(gamepad1.a);
            extendoPosIdx -= extendoIdxDecStickyButton.getState() ? 1 : 0;

            // Bounds check
            extendoPosIdx = Range.clip(extendoPosIdx, 0, Intake.EXTEND_POS.INTAKING.getVal());
        }
        else
        {
            extendoMiniIncStickyButton.update(gamepad1.dpad_right);
            extendoTicks += extendoMiniIncStickyButton.getState() ? 100 : 0;

            extendoMiniDecStickyButton.update(gamepad1.dpad_left);
            extendoTicks -= extendoMiniDecStickyButton.getState() ? 100 : 0;

            extendoIncStickyButton.update(gamepad1.dpad_up);
            extendoTicks += extendoIncStickyButton.getState() ? 200 : 0;

            extendoDecStickyButton.update(gamepad1.dpad_down);
            extendoTicks -= extendoDecStickyButton.getState() ? 200 : 0;

            // Bounds check
            extendoTicks = Range.clip(extendoTicks, 1, extendoTicksMax);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        extendo = new Intake(hardwareMap);
        myTimer = new ElapsedTime();

        extendo.teleopInit();

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        myTimer.reset();
        telemetry.clearAll();
        telemTimer.reset();

        while (!isStopRequested()) {
            mapControls();
            telemetry.addLine("Press buttons to adjust extendo position");
            telemetry.addLine();
            telemetry.addLine("Xbox/PS4 Button - Action");
            telemetry.addLine("lft_bumper: reset position to 0");
            telemetry.addLine("rt_bumper : Change mode");

            if(idxMode)
            {
                telemetry.addLine("Idx Mode");
                telemetry.addLine("Y         : +1 Idx");
                telemetry.addLine("A         : -1 Idx");
                telemetry.addData("extendo Pos   : ", extendoPosIdx);
            } else {
                telemetry.addLine("Ticks Mode");
                telemetry.addLine(String.format("dpad_right: +%4d ticks", smallTicks));
                telemetry.addLine(String.format("dpad_left : -%4d ticks", smallTicks));
                telemetry.addLine(String.format("dpad_up   : +%4d ticks", bigTicks));
                telemetry.addLine(String.format("dpad_down : -%4d ticks", bigTicks));
                telemetry.addData("extendo Ticks : ", extendoTicks);
            }

            telemetry.addData("extendo Enc: ", extendo.getLiveExtendoPosition());
            telemetry.addLine();

            telemetry.update();

            if (idxMode)
            {
                extendo.setExtendoPos(extendoPosIdx, myTimer.seconds());
            } else {
                if (inches) {
                    extendo.setExtendoTicks((int) ((extendoTicks * 25.4) / 1.25), myTimer.seconds());
                }else{
                    extendo.setExtendoTicks(extendoTicks, myTimer.seconds());
                }
            }
            extendo.update(myTimer.seconds());
            extendo.setPivotPos(pivot);

            boolean debug = true;
            if(debug)
            {
                if( telemTimer.hasExpired() )
                {
                    BotLog.logD("extendoTuner", extendo.getTelem(myTimer.seconds()));
                    telemTimer.reset();
                }
            }
        }
    }
}