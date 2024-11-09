package org.firstinspires.ftc.teamcode.autonomous.enums;

/**
 * The enum Team Color to use in autonomous.
 */
public enum AllianceColor
{
	BLUE(0),
	RED(1);

	private final int value;

	AllianceColor(int value) {
		this.value = value;
	}

	public int getValue() {
		return value;
	}
}
