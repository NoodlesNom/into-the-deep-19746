package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderCanceller extends SensorCanceller {
    private DcMotor motor;

    /**
     * Instantiates a new motor encoder canceller.
     *
     * @param target     the target
     * @param motor     the motor
     * @param condition  the condition
     */
    public EncoderCanceller(double target, DcMotor motor, Condition condition)
    {
        super(target, condition);

        this.motor = motor;
    }

    @Override
    protected double getCurrentValue() {
        return motor.getCurrentPosition();
    }
}