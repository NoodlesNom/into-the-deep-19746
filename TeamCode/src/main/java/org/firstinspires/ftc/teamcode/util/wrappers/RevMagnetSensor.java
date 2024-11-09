package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class RevMagnetSensor {
    /**
     * The magnet sensor.
     */
    private DigitalChannel magnet;

    /**
     * Instantiates a new magnet sensor.
     *
     * @param magnet the magnet sensor
     */
    public RevMagnetSensor(DigitalChannel magnet)
    {
        this.magnet = magnet;
    }

    /**
     * Returns true if magnet sensor is triggered.
     * Returns false if magnet sensor is not triggered.
     *
     * @return whether the magnet sensor is triggered
     */
    public boolean isTriggered()
    {
        return !magnet.getState();
    }
}
