package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.utils.Range;

public class RelativeEncoderInRangeCanceller extends EncoderInRangeCanceller {

    /**
     * Instantiates a new motor encoder canceller.
     *
     * @param range     the range
     * @param motor      the motor
     */
    public RelativeEncoderInRangeCanceller(Range range, DcMotor motor, double ticksPerInch)
    {
        super(range, motor, ticksPerInch);

        reset();
    }

    public void reset()
    {
        range.shiftOriginalRange(getCurrentValue());
    }

    public void shiftRange()
    {
        range.shiftOriginalRange(-30.0);
    }
}