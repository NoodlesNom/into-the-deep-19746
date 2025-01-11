package org.firstinspires.ftc.teamcode.autonomous.gf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



public abstract class OldAutoMaster extends LinearOpMode
{
    private GFRobot robot = null;
    private ElapsedTime test = new ElapsedTime();
    public static RevBlinkinLedDriver.BlinkinPattern team= RevBlinkinLedDriver.BlinkinPattern.BLACK;
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
        }

        test.reset();

        telemetry.addLine("Starting opMode");
        telemetry.update();

        while (!isStopRequested())
        {
            runMain(robot, test.seconds());
            telemetry.addData("x power (sideways)", GFMovement.getMovement_x());
            telemetry.addData("y power (forward)", GFMovement.getMovement_y());
            telemetry.addData("turn power", GFMovement.getMovement_rad());
            telemetry.addData("x", robot.mDrive.mPeriodicIO.currentpose.position.x);
            telemetry.addData("y", robot.mDrive.mPeriodicIO.currentpose.position.y);
            telemetry.addData("y state", GFMovement.state_movement_y_prof.name());
            telemetry.addData("x state", GFMovement.state_movement_x_prof.name());
            telemetry.addData("turn state", GFMovement.state_turning_prof.name());
            telemetry.addLine(robot.getTelem(test.seconds()));
            telemetry.update();
        }

        robot.stop();

    }

    public abstract void setStartPose(GFRobot robot);

    public abstract void runMain(GFRobot robot, double time);


}
