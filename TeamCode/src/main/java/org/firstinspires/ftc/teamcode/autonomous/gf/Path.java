package org.firstinspires.ftc.teamcode.autonomous.gf;

import java.util.ArrayList;

public class Path
{
    private ArrayList<CurvePoint> points;
    private boolean rev;

    public Path(ArrayList<CurvePoint> path, boolean reversed)
    {
        points = path;
        rev = reversed;
    }

    public boolean getReversed()
    {
        return rev;
    }

    public ArrayList<CurvePoint> getPath()
    {
        return points;
    }
}
