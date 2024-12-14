package org.firstinspires.ftc.teamcode.autonomous.gf;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



public abstract class OldAutoMaster extends LinearOpMode
{
    private GFRobot robot = null;
    private ElapsedTime test = new ElapsedTime();

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

        waitForStart();

        test.reset();

        telemetry.addLine("Starting opMode");
        telemetry.update();

        while (!isStopRequested())
        {
            runMain(robot, test.seconds());

            telemetry.addLine(robot.getTelem(test.seconds()));
            telemetry.update();
        }

        robot.stop();

    }

    public abstract void setStartPose(GFRobot robot);

    public abstract void runMain(GFRobot robot, double time);


}
