package org.firstinspires.ftc.teamcode.util.tests;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BotLog;
import org.firstinspires.ftc.teamcode.util.StickyButton;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "drive train tester")
public class driveTrainTester extends LinearOpMode {
    private Drive drive;
    private ElapsedTime myTimer;
    private StickyButton button = new StickyButton();
    private Deadline telemTimer = new Deadline(100, TimeUnit.MILLISECONDS);
    private ArrayList<Double> times = new ArrayList<>();
    private ArrayList<Double> vel = new ArrayList<>();
    private ElapsedTime stopTimer = new ElapsedTime();
    private boolean started = false;

    private enum State {
        accel,
        finished
    }

    private State auto = State.finished;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new Drive(hardwareMap);
        myTimer = new ElapsedTime();

        //lift.teleopInit();

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        myTimer.reset();
        telemetry.clearAll();
        telemTimer.reset();

        while (!isStopRequested()) {
            drive.update(myTimer.seconds());
            button.update(gamepad1.a);
            double driveInput = -gamepad1.left_stick_y;
            double strafeInput = -gamepad1.left_stick_x;
            double turnInput = -gamepad1.right_stick_x;
            if (gamepad1.dpad_up){
                driveInput=1;
                strafeInput=0;
                turnInput=0;
            }else if (gamepad1.dpad_right){
                driveInput=0;
                turnInput=0;
                strafeInput=-1;

            }
            else if (gamepad1.dpad_left){
                driveInput=0;
                turnInput=0;
                strafeInput=1;

            }else if (gamepad1.dpad_down){
                driveInput=-1;
                turnInput=0;
                strafeInput=0;

            }
            drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(driveInput, strafeInput), turnInput));
            switch (auto)
            {
                case accel:
                {
                    if (button.getState())
                    {
                        //robot.mDrive.setOpenLoop(Drive.DriveSignal.NEUTRAL);
                        stopTimer.reset();

                        BotLog.logD("t", String.format("%s", times.toString()));
                        BotLog.logD("v", String.format("%s", vel.toString()));

                        auto = State.finished;


                    }
                    if (!gamepad1.atRest()||gamepad1.dpad_down||gamepad1.dpad_up||gamepad1.dpad_right||gamepad1.dpad_left){
                        started=true;
                    }
                    if (started) {
                        times.add(myTimer.seconds());
                        vel.add(drive.drive.mPeriodicIO.vel.linearVel.norm());
                    }else{
                        times.clear();
                        vel.clear();
                        myTimer.reset();
                    }

                    break;
                }
                case finished:
                {
                    if (button.getState()){
                        times.clear();
                        vel.clear();
                        myTimer.reset();
                        started=false;
                        auto = State.accel;
                    }
                    break;
                }

            }


        }
    }
}