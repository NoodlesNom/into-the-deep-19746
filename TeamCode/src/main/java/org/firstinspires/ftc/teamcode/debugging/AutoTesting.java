package org.firstinspires.ftc.teamcode.debugging;

import org.firstinspires.ftc.teamcode.autonomous.gf.GFMovement;

public class AutoTesting extends RobotTesting
{
    public boolean stageFinished = true;

    public long programStartTime = 0;
    public long stateStartTime = 0;
    public static int programStage = 0;

    public double blockStartingX = 0;
    public double blockStartingY = 0;
    public double blockStartingAngle_rad = 0;

    //holds the stage we are going to next
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

    /**
     * Mostly makes sure we start in 18 inches
     */
    @Override
    public void init()
    {
        super.init();

    }


    /**
     * Set's our position to the starting postition although this is dumb
     * because who even uses this anyway, we'll reset when we get down
     */
    @Override
    public void init_loop()
    {
        super.init_loop();
        // Now we can set our position
        setPosition(startingPos_x, startingPos_y, startingPos_angle_rad);
    }

    @Override
    public void start()
    {
        super.start();
        // Now we can set our position
        setPosition(startingPos_x, startingPos_y, startingPos_angle_rad);

        stageFinished = true; // need to call initialize state variables
        programStage = 0; // start on the first state
        programStartTime = System.currentTimeMillis();//record the start time of the program
    }

    @Override
    public void loop()
    {
        super.loop();
        DebugController();
    }

    /**
     * allows debugging the autonomous
     */
    private void DebugController()
    {
        MainStateMachine();
    }


    // Override me
    // This really should be an abstract class
    public void MainStateMachine()
    {

    }
}
