package org.firstinspires.ftc.teamcode.autonomous.gf;

import java.util.ArrayList;

public class Path
{
    private ArrayList<CurvePoint> points;
    private boolean rev = false;

    private double angle = Math.toRadians(90);

    private boolean constant = false;

    public Path(ArrayList<CurvePoint> path, boolean reversed)
    {
        points = path;
        rev = reversed;
    }
    public Path(ArrayList<CurvePoint> path, boolean reversed, double followangle)
    {
        points = path;
        rev = reversed;
        angle = followangle;
    }
    public Path(ArrayList<CurvePoint> path, double followangle)
    {
        points = path;
        angle = followangle;
    }

    public Path(ArrayList<CurvePoint> path, boolean reversed, double followangle, boolean followconstant)
    {
        points = path;
        rev = reversed;
        angle = followangle;
        constant = followconstant;
    }
    public Path(ArrayList<CurvePoint> path, double followangle, boolean followconstant)
    {
        points = path;
        angle = followangle;
        constant = followconstant;
    }

    public boolean getReversed()
    {
        return rev;
    }

    public ArrayList<CurvePoint> getPath()
    {
        return points;
    }

    public double getAngle(){return angle;}
    public boolean getConstant(){return constant;}
}
