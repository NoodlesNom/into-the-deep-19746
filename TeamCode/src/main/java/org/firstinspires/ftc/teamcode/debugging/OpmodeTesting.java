package org.firstinspires.ftc.teamcode.debugging;


import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getAngle_rad;
import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getXPos;
import static org.firstinspires.ftc.teamcode.debugging.RobotTesting.getYPos;

import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;

public abstract class OpmodeTesting
{

    public boolean stageFinished = true;

    public long stateStartTime = 0;
    public static int programStage = 0;

    public double blockStartingX = 0;
    public double blockStartingY = 0;
    public double blockStartingAngle_rad = 0;

    // holds the stage we are going to next
    int nextStage = 0;

    public void nextStage(int ordinal)
    {
        nextStage = ordinal;
    }

    /**
     * Increments the programStage
     */
    public void nextStage()
    {
        nextStage(programStage + 1);

    }

    private void incrementStage()
    {
        programStage = nextStage;
        stageFinished = true;
    }

    /**
     * called during the init of any stage
     */
    public void initializeStateVariables()
    {
        stageFinished = false;
        blockStartingX = getXPos();
        blockStartingY = getYPos();
        blockStartingAngle_rad = getAngle_rad();
        stateStartTime = System.currentTimeMillis();
        GFMovement.initForMove();
        GFMovement.initCurve();
    }

    private double startingPos_x = 0;
    private double startingPos_y = 0;
    private double startingPos_angle_rad = 0;

    public void setStartingPosition(double x, double y, double angle_rad)
    {
        startingPos_x = x;
        startingPos_y = y;
        startingPos_angle_rad = angle_rad;
    }

    public void MainStateMachine()
    {

    }

    public abstract void init();

    public abstract void init_loop();

    public abstract void start();

    public abstract void loop();
}
