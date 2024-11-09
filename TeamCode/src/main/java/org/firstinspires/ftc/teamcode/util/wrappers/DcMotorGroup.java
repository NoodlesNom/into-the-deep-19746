package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The class DC motor group.
 */
public class DcMotorGroup
{
	/**
	 * The grouped motors.
	 */
	private List<DcMotor> motors = new ArrayList<>();

	public DcMotorGroup(DcMotor... dcMotors)
	{
		Collections.addAll(motors, dcMotors);
	}

	public ArrayList<Integer> getPositions()
	{
		ArrayList<Integer> positions = new ArrayList<>();
		for (DcMotor motor : motors)
			positions.add(motor.getCurrentPosition());
		return positions;
	}

	public int singleGetPosition(int index)
	{
		return motors.get(index).getCurrentPosition();
	}


	/**
	 * Sets power to each motor in the group.
	 *
	 * @param power desired power to set to each motor
	 */
	public void setPower(double power)
	{
		for (DcMotor motor : motors)
			motor.setPower(power);
	}

	public void singleSetPower(int index, double power)
	{
		motors.get(index).setPower(power);
	}

	public void setFourPowers(double firstPower, double secondPower, double thirdPower, double fourthPower)
	{
		singleSetPower(0, firstPower);
		singleSetPower(1, secondPower);
		singleSetPower(2, thirdPower);
		singleSetPower(3, fourthPower);

	}

	public void setRunMode(DcMotor.RunMode runMode)
	{
		for (DcMotor motor : motors)
			motor.setMode(runMode);
	}

	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
	{
		for (DcMotor motor : motors)
			motor.setZeroPowerBehavior(zeroPowerBehavior);
	}

	/**
	 * Stops motors in the group.
	 */
	public void stop()
	{
		for (DcMotor motor : motors)
			motor.setPower(0);
	}
}
