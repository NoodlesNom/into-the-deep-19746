package org.firstinspires.ftc.teamcode.util.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SmoothDrive {

    ////////////
    //DRIVER 1//
    ////////////
    /*
        NEW DRIVER 1 CONTROLS
        Left Stick X:
        Left Stick Y: Drive left side
        Left Stick Button: Press left button once to enable collector flip servos and move it to exactly horizontal, press again to disable flip servos
        Right Stick X:
        Right Stick Y: Drive right side
        Right Stick Button:
        D-pad Up:
        D-pad Left:
        D-pad Down: Carousel left wheel move , click again to spin other way
        D-pad Right:
        Start:
        X: Press X once to close the gate servo, extend the depositor out, and deposit the element.
        Press X again to close the deposit servo, retract depositor, and open gate servo.
        B: Press B once to close the depositor gate servo, and press B once more to open it
        Y:
        A: Press A once to put collector gate servo in and click A again to put it out.
        Left Bumper: Reverse collector when holding
        Left Trigger: Move lift down
        Right Bumper: Toggle collector on/off
        Right Trigger: Move lift up
     */

    // Mech drive related variables
    private int[] speedIdx = new int[] {0, 0};
    private double[] speedModifier = new double[] {0.75, 0.90};
    private boolean[] FODEnable = new boolean[] {false, true};
    private double[] forward = new double[2], strafe = new double[2], rotate = new double[2];
    private double[] prevForward = new double[2], prevStrafe = new double[2], prevRotate = new double[2];
    private double[] prevTime = new double[2];
    private double maxForwardChange=4.0, maxRotateChange=5.0, maxStrafeChange=3.0;
    private boolean smoothDrive = true;
    private double now;
    private double deltaT;

    public double l_f_motor_power;
    public double l_b_motor_power;
    public double r_f_motor_power;
    public double r_b_motor_power;
    private double maxPwr = 0.0;
 
    private FOD[] fod = new FOD[]{new FOD(0), new FOD(0)};
    private ElapsedTime runtime = new ElapsedTime();
    Gamepad gamepads[] = new Gamepad[] {null, null};

    public SmoothDrive(Gamepad gamepad1) {
        this.gamepads[0] = gamepad1;
        this.gamepads[1] = null;
    }

    public SmoothDrive(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepads[0] = gamepad1;
        this.gamepads[1] = gamepad2;
    }

    // We should probably just manage all this inside the mapControls
    // And handle all the buttons there?
    public void setSpeedIdx1(int g1) {
        speedIdx[0] = g1;
    }

    public void setSpeedIdx2(int g2) {
        speedIdx[1] = g2;
    }

    public void setFODEnable1( boolean g1) {
        FODEnable[0] = g1;
    }

    public void setFODEnable2( boolean g2) {
        FODEnable[1] = g2;
    }

    public void setFODAdjustAngle1(double d) {
        fod[0].setAdjust(d);
    }

    public void setFODAdjustAngle2(double d) {
        fod[1].setAdjust(d);
    }

    private void mapControls() {
    }

    public void update() {
        this.update(0);
    }

    public void update( double heading ) {
        int padIdx;
        Gamepad gamepad;

        // Later, maybe need to move some of the button readings in here
        mapControls();

        l_f_motor_power = 0 ;
        l_b_motor_power = 0;
        r_f_motor_power = 0;
        r_b_motor_power = 0;

        for (padIdx = 0; padIdx < gamepads.length; padIdx++) {
            gamepad = gamepads[padIdx];
            if ( gamepad == null ) { continue; }
  
            // Read the controller 1 (driver) stick positions
            strafe[padIdx] = gamepad.left_stick_x;
            forward[padIdx] = -gamepad.left_stick_y;
            rotate[padIdx] = gamepad.right_stick_x;
 
            /* High Precision Mode is 25% power */
            if( gamepad.left_stick_button ) {
                strafe[padIdx] *= .25;
                forward[padIdx] *= .25;
            }
 
            if( gamepad.right_stick_button ) {
                rotate[padIdx] *= .25;
            }
 
            // Convert to FOD as soon as possible so all the smooth drive code applies correctly
            if (FODEnable[padIdx] && (heading != 0)) {
                fod[padIdx].setCar(strafe[padIdx], forward[padIdx], heading);

                // Replace the strafe and forward power with translated values
                // Now the robot moves in orientation of the field
                strafe[padIdx] = fod[padIdx].getNewX();
                forward[padIdx] = fod[padIdx].getNewY();
            }
 
            // Adjustments to make strafe/rotate feel more responsive/controllable
            strafe[padIdx] *= 1.25;
            rotate[padIdx] *= 0.75;

            if(smoothDrive) {
                now = runtime.seconds();
                deltaT = now - prevTime[padIdx];
 
                if ((prevStrafe[padIdx] != 0) && (strafe[padIdx] != 0) && (Math.signum(prevStrafe[padIdx]) != Math.signum(strafe[padIdx]))) {
                    strafe[padIdx] = 0;
                }
                if ((prevForward[padIdx] != 0) && (forward[padIdx] != 0) && (Math.signum(prevForward[padIdx]) != Math.signum(forward[padIdx]))) {
                    forward[padIdx] = 0;
                }
                if ((prevRotate[padIdx] != 0) && (rotate[padIdx] != 0) && (Math.signum(prevRotate[padIdx]) != Math.signum(rotate[padIdx]))) {
                    rotate[padIdx] = 0;
                }

                if (Math.abs(strafe[padIdx]) > 0.20) {
                    if (prevStrafe[padIdx] < strafe[padIdx]) {
                        strafe[padIdx] = Math.min(strafe[padIdx], prevStrafe[padIdx] + (maxStrafeChange * deltaT));
                    } else {
                        strafe[padIdx] = Math.max(strafe[padIdx], prevStrafe[padIdx] - (maxStrafeChange * deltaT));
                    }
                }
                if (Math.abs(forward[padIdx]) > 0.20) {
                    if (prevForward[padIdx] < forward[padIdx]) {
                        forward[padIdx] = Math.min(forward[padIdx], prevForward[padIdx] + (maxForwardChange * deltaT));
                    } else {
                        forward[padIdx] = Math.max(forward[padIdx], prevForward[padIdx] - (maxForwardChange * deltaT));
                    }
                }
                if (Math.abs(rotate[padIdx]) > 0.20) {
                    if (prevRotate[padIdx] < rotate[padIdx]) {
                        rotate[padIdx] = Math.min(rotate[padIdx], prevRotate[padIdx] + (maxRotateChange * deltaT));
                    } else {
                        rotate[padIdx] = Math.max(rotate[padIdx], prevRotate[padIdx] - (maxRotateChange * deltaT));
                    }
                }
                prevTime[padIdx] = now;
                prevStrafe[padIdx] = strafe[padIdx];
                prevForward[padIdx] = forward[padIdx];
                prevRotate[padIdx] = rotate[padIdx];

                boolean lessStrafe = true;
                // Remove 15% deadzone
                if(lessStrafe) {
                    if (strafe[padIdx] >= 0.25) {
                        strafe[padIdx] = (strafe[padIdx] * 0.85) + 0.15;
                    } else if (strafe[padIdx] > 0.0) {
                        strafe[padIdx] = 0.0;
                    }
                } else {
                    if (strafe[padIdx] >= 0.025) {
                        strafe[padIdx] = (strafe[padIdx] * 0.85) + 0.15;
                    }
                }
                if (forward[padIdx] >= 0.025) {
                    forward[padIdx] = (forward[padIdx] * 0.85) + 0.15;
                }
                if (rotate[padIdx] >= 0.025) {
                    rotate[padIdx] = (rotate[padIdx] * 0.85) + 0.15;
                }
                if(lessStrafe) {
                    if (strafe[padIdx] <= -0.25) {
                        strafe[padIdx] = (strafe[padIdx] * 0.85) - 0.15;
                    } else if (strafe[padIdx] < 0.0) {
                        strafe[padIdx] = 0.0 ;
                    }
                } else {
                    if (strafe[padIdx] <= -0.025) {
                        strafe[padIdx] = (strafe[padIdx] * 0.85) - 0.15;
                    }
                }
                if (forward[padIdx] <= -0.025) {
                    forward[padIdx] = (forward[padIdx] * 0.85) - 0.15;
                }
                if (rotate[padIdx] <= -0.025) {
                    rotate[padIdx] = (rotate[padIdx] * 0.85) - 0.15;
                }
            }

            // Rotate a little left
            if (gamepad.dpad_left) {
                rotate[padIdx] -= 0.4;
            }
            // Rotate a little right
            if (gamepad.dpad_right) {
                rotate[padIdx] += 0.4;
            }
 
            // This adds the powers from both controllers together scaled for each controller and FOD
            l_f_motor_power += ((forward[padIdx] + strafe[padIdx] + rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
            l_b_motor_power += ((forward[padIdx] - strafe[padIdx] + rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
            r_f_motor_power += ((forward[padIdx] - strafe[padIdx] - rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);
            r_b_motor_power += ((forward[padIdx] + strafe[padIdx] - rotate[padIdx]) * speedModifier[speedIdx[padIdx]]);

        }
 
        if(smoothDrive) {
            // Find the largest power request ignoring sign
            maxPwr = Math.max(Math.max(Math.max(Math.abs(l_f_motor_power), Math.abs(l_b_motor_power)),
                    Math.abs(r_f_motor_power)), Math.abs(r_b_motor_power));
 
            // If this is greater than 1.0, need to scale everything back equally
            // Max is now guaranteed positive which is good to reduce magnitude without changing sign
            // Now the power is scaled and limited to range of {-1.0, 1.0)
            if (maxPwr > 1.0) {
                l_f_motor_power /= maxPwr;
                l_b_motor_power /= maxPwr;
                r_f_motor_power /= maxPwr;
                r_b_motor_power /= maxPwr;
            }
        }

        // This code needs to be implemented by the caller
        // We follow different logic based on whether we are in manual driver control or switch
        // control to the automatic mode
        // robot.drive.setMotorPowers(l_f_motor_power, l_b_motor_power, r_b_motor_power, r_f_motor_power);
    }
}
