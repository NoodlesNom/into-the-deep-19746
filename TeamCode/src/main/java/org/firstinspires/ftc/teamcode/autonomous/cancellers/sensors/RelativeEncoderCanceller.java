package org.firstinspires.ftc.teamcode.autonomous.cancellers.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RelativeEncoderCanceller extends EncoderCanceller {
    /**
     * Instantiates a new motor encoder canceller.
     *
     * @param target     the target
     * @param motor      the motor
     * @param condition  the condition
     */
    public RelativeEncoderCanceller(double target, DcMotor motor, Condition condition)
    {
        super(motor.getTargetPosition() + target, motor, condition);
    }

    @Override
    public void setTarget(double target) {
        super.setTarget(getCurrentValue() + target);
    }
}