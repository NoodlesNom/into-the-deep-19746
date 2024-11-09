package org.firstinspires.ftc.teamcode.Subsystems;

public abstract class Subsystem {

    public boolean shutdown = false;

    public abstract void autoInit();

    public abstract void teleopInit();

    public abstract void update(double timestamp);

    public abstract void stop();

    public abstract String getTelem(double time);

    public abstract void readPeriodicInputs(double time);

    public abstract void writePeriodicOutputs();

}
