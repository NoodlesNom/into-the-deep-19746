package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.utils.Range;

public class EncoderInRangeCanceller extends SensorInRangeCanceller {

    private DcMotor motor;
    private double ticksPerInch;

    /**
     * Instantiates a new motor encoder canceller.
     *
     * @param range     the range
     * @param motor     the motor
     */
    public EncoderInRangeCanceller(Range range, DcMotor motor, double ticksPerInch)
    {
        super(range);

        this.motor = motor;
        this.ticksPerInch = ticksPerInch;
    }

    @Override
    protected double getCurrentValue() {
        return motor.getCurrentPosition() / ticksPerInch;
    }
}