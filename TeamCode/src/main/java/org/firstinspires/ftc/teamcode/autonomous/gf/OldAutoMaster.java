package org.firstinspires.ftc.teamcode.autonomous.gf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.BotLog;

import java.util.concurrent.TimeUnit;


public abstract class OldAutoMaster extends LinearOpMode
{
    private GFRobot robot = null;
    private ElapsedTime test = new ElapsedTime();
    public static RevBlinkinLedDriver.BlinkinPattern team= RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public String getTelem() { return ""; };

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Creating robot");
        telemetry.update();
        robot = new GFRobot(this);

        telemetry.addLine("Initing robot");
        telemetry.update();
        robot.autoInit();

        telemetry.addLine("Setting start pos");
        telemetry.update();
        setStartPose(robot);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while  (!isStopRequested()&&!opModeIsActive()){
            robot.mDeposit.setLiveLed(team);
            telemetry.addLine("PRESS A (BOTTOM) FOR BLUE");
            telemetry.addLine("PRESS B (RIGHT) FOR RED");
            if (gamepad1.a){
                team = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            }
            if (gamepad1.b){
                team = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            telemetry.update();
            robot.mDeposit.autoInit();
            robot.mIntake.setPivotPos(Intake.PIVOT_POS.INTAKING.getVal());
            robot.update(test.seconds());
        }

        test.reset();

        telemetry.addLine("Starting opMode");
        telemetry.update();

        boolean debug = false;
        Deadline logging = new Deadline(200, TimeUnit.MILLISECONDS);
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        // We're through init, lets go back to bulk caching and clear caches
        //for (LynxModule hub : robot.allHubs)
        //{
        //    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        //    hub.clearBulkCache();
        //}

        while (!isStopRequested())
        {
            //for (LynxModule hub : robot.allHubs)
            //{
            //    hub.clearBulkCache();
            //}

            runMain(robot, test.seconds());
            telemetry.addData("x power (sideways)", GFMovement.getMovement_x());
            telemetry.addData("y power (forward)", GFMovement.getMovement_y());
            telemetry.addData("turn power", GFMovement.getMovement_rad());
            telemetry.addData("x", robot.mDrive.mPeriodicIO.currentpose.position.x);
            telemetry.addData("y", robot.mDrive.mPeriodicIO.currentpose.position.y);
            telemetry.addData("y state", GFMovement.state_movement_y_prof.name());
            telemetry.addData("x state", GFMovement.state_movement_x_prof.name());
            telemetry.addData("turn state", GFMovement.state_turning_prof.name());

            if(debug) {
                if(logging.hasExpired())
                {
                    logging.reset();
                    String debugMsg = robot.getTelem(test.seconds());
                    debugMsg += getTelem();
                    debugMsg += String.format("loopTime = %4.0f\n", loopTimer.milliseconds());
                    telemetry.addLine(debugMsg);
                    BotLog.logD("BSDbg", debugMsg);
                }
                loopTimer.reset();
            }
            telemetry.update();
        }

        robot.stop();

    }

    public abstract void setStartPose(GFRobot robot);

    public abstract void runMain(GFRobot robot, double time);


}
