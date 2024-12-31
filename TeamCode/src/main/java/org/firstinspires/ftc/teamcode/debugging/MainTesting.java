package org.firstinspires.ftc.teamcode.debugging;


import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getAngle_rad;
import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getXPos;
import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getYPos;

import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;

public class MainTesting
{

    public static void main(String[] args)
    {
        new MainTesting().run();
    }

    /**
     * The program runs here
     */
    public void run()
    {
        ComputerDebuggingNew computerDebugging = new ComputerDebuggingNew();
        RobotTesting robot = new RobotTesting();
        OpmodeTesting opMode = new BlueSideAuto();
        opMode.init();
        GFMovement.setVoltageComp(1);

        ComputerDebuggingNew.clearLogPoints();

        try
        {
            Thread.sleep(1000);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        while (true)
        {

            opMode.loop();

            try
            {
                Thread.sleep(30);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            robot.update();
            ComputerDebuggingNew.sendRobotLocation(getXPos(), getYPos(), getAngle_rad());
            ComputerDebuggingNew.markEndOfUpdate();
        }
    }
}
