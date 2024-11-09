package org.firstinspires.ftc.teamcode.util;//package org.firstinspires.ftc.teamcode.util;
//
//import org.firstinspires.ftc.teamcode.Kinematics;
//import org.firstinspires.ftc.teamcode.lib.geometry.Twist2d;
//import org.firstinspires.ftc.teamcode.subsystems.Drive;
//
//public class RLHelper {
//
//    // These factor determine how fast the wheel traverses the "non linear" sine curve.
//    private static final double kWheelNonLinearity = 0.5;
//
//    private static final double kNegInertiaThreshold = 0.5;
//    private static final double kNegInertiaTurnScalar = 0;
//    private static final double kNegInertiaCloseScalar = 0; // was 4.0
//    private static final double kNegInertiaFarScalar = 0;
//
//    private static final double kSensitiity = 0.75; // was .65
//
//    private static double mOldWheel = 0.0;
//    private static double mQuickStopAccumlator = 0.0;
//    private static double mNegInertiaAccumlator = 0.0;
//
//    private static final double kQuickStopDeadband = 0.5;
//    private static final double kQuickStopWeight = 0.1;
//    private static final double kQuickStopScalar = 4.0;
//
//    public static Drive.DriveSignal RLDrive(double throttle, double wheel, boolean isQuickTurn) {
//
//        double negInertia = wheel - mOldWheel;
//        mOldWheel = wheel;
//
//        double wheelNonLinearity;
//        wheelNonLinearity = kWheelNonLinearity;
//        //final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
//        // Apply a sin function that's scaled to make it feel better.
//        //wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
//        //wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
//        //wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
//
//        double leftPwm, rightPwm, overPower;
//        double sensitivity;
//
//        double angularPower;
//        double linearPower;
//
//        // Negative inertia!
//        double negInertiaScalar;
//
//        if (wheel * negInertia > 0) {
//            // If we are moving away from 0.0, aka, trying to get more wheel.
//            negInertiaScalar = kNegInertiaTurnScalar;
//        } else {
//            // Otherwise, we are attempting to go back to 0.0.
//            if (Math.abs(wheel) > kNegInertiaThreshold) {
//                negInertiaScalar = kNegInertiaFarScalar;
//            } else {
//                negInertiaScalar = kNegInertiaCloseScalar;
//            }
//        }
//        sensitivity = kSensitiity;
//        double negInertiaPower = negInertia * negInertiaScalar;
//        mNegInertiaAccumlator += negInertiaPower;
//
//        wheel = wheel + mNegInertiaAccumlator;
//        if (mNegInertiaAccumlator > 1) {
//            mNegInertiaAccumlator -= 1;
//        } else if (mNegInertiaAccumlator < -1) {
//            mNegInertiaAccumlator += 1;
//        } else {
//            mNegInertiaAccumlator = 0;
//        }
//        linearPower = throttle;
//
//        // Quickturn!
//        if (isQuickTurn) {
//            if (Math.abs(linearPower) < kQuickStopDeadband) {
//                double alpha = kQuickStopWeight;
//                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
//                        + alpha * util.limit(wheel, 1.0) * kQuickStopScalar;
//            }
//            overPower = 1.0;
//            angularPower = wheel;
//        } else {
//            overPower = 0.0;
//            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
//            if (mQuickStopAccumlator > 1) {
//                mQuickStopAccumlator -= 1;
//            } else if (mQuickStopAccumlator < -1) {
//                mQuickStopAccumlator += 1;
//            } else {
//                mQuickStopAccumlator = 0.0;
//            }
//        }
//
//        rightPwm = leftPwm = linearPower;
//        leftPwm += angularPower;
//        rightPwm -= angularPower;
//
//        if (leftPwm > 1.0) {
//            rightPwm -= overPower * (leftPwm - 1.0);
//            leftPwm = 1.0;
//        } else if (rightPwm > 1.0) {
//            leftPwm -= overPower * (rightPwm - 1.0);
//            rightPwm = 1.0;
//        } else if (leftPwm < -1.0) {
//            rightPwm += overPower * (-1.0 - leftPwm);
//            leftPwm = -1.0;
//        } else if (rightPwm < -1.0) {
//            leftPwm += overPower * (-1.0 - rightPwm);
//            rightPwm = -1.0;
//        }
//        return new Drive.DriveSignal(leftPwm, rightPwm);
//    }
//
//    public static Drive.DriveSignal RLDrive2(double throttle, double wheel, boolean isQuickTurn) {
//
//        final double kWheelGain = 0.05;
//        final double kWheelNonlinearity = 0.05;
//        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
//        // Apply a sin function that's scaled to make it feel better.
//        if (!isQuickTurn) {
//            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
//            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
//            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
//        }
//
//        wheel *= kWheelGain;
//        Drive.DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
//        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
//        return (new Drive.DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
//
//    }
//
//}