package org.firstinspires.ftc.teamcode.debugging;

import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement.*;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;

public class RobotTesting extends OpmodeTesting
{

    /**
     * Creates a robot simulation
     */

    public RobotTesting()
    {
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = 0;
    }

    // the actual speed the robot is moving
    public static double xSpeed = 0;
    public static double ySpeed = 0;
    public static double turnSpeed = 0;

    private static double worldXPosition;
    private static double worldYPosition;
    private static double worldAngle_rad;

    public static double getXPos()
    {
        return worldXPosition;
    }

    public static double getYPos()
    {
        return worldYPosition;
    }

    public static double getAngle_rad()
    {
        return worldAngle_rad;
    }

    /**
     * USE THIS TO SET OUR POSITION
     **/
    public static void setPosition(double x, double y, double angle)
    {
        worldXPosition = x;
        worldYPosition = y;
        worldAngle_rad = angle;
    }

    //last update time
    private long lastUpdateTime = 0;

    public static int elapsedMillisThisUpdate = 0;

    /**
     * Calculates the change in position of the robot
     */
    public void update()
    {
        // get the current time
        long currentTimeMillis = System.currentTimeMillis();
        // get the elapsed time
        double elapsedTime = (currentTimeMillis - lastUpdateTime) / 1000.0;
        elapsedMillisThisUpdate = (int) (currentTimeMillis - lastUpdateTime);
        // remember the lastUpdateTime
        lastUpdateTime = currentTimeMillis;
        if (elapsedTime > 1)
        {
            return;
        }


        //increment the positions
        double totalSpeed = Math.hypot(xSpeed, ySpeed);
        double angle = Math.atan2(ySpeed, xSpeed) - Math.toRadians(90);
        double outputAngle = worldAngle_rad + angle;
        worldXPosition += totalSpeed * Math.cos(outputAngle) * elapsedTime * 1000 * 0.2;
        worldYPosition += totalSpeed * Math.sin(outputAngle) * elapsedTime * 1000 * 0.2;

        worldAngle_rad += GFMovement.getMovement_rad() * elapsedTime * 20 / (2 * Math.PI);
        worldAngle_rad = AngleWrap(worldAngle_rad);

        xSpeed += Range.clip((GFMovement.getMovement_x() - xSpeed) / 0.5, -1, 1) * elapsedTime;
        ySpeed += Range.clip((GFMovement.getMovement_y() - ySpeed) / 0.5, -1, 1) * elapsedTime;
        turnSpeed += Range.clip((GFMovement.getMovement_rad() - turnSpeed) / 0.5, -1, 1) * elapsedTime;

        xSpeed *= 1.0 - (elapsedTime);
        ySpeed *= 1.0 - (elapsedTime);
        turnSpeed *= 1.0 - (elapsedTime);
    }

    @Override
    public void init()
    {
        setPosition(0, 0,
                0);
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {

    }
}
