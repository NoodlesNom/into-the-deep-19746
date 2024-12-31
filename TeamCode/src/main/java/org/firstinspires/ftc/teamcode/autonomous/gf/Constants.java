package org.firstinspires.ftc.teamcode.autonomous.gf;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    // Measured by experimental data (outer-inner) for 5 pivot revolutions translated to radius
    public static final double kDriveWheelTrackWidthInches = 15.85;
    // Experimental data average of 3 manual trials~: 4.054432073423333
    // Actual straight line path following (101.8mm diameter = 96.4mm wheel + (2*2.54) tread)
    public static final double kDriveWheelDiameterInches = 4.01043190623;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    // This is ratio of the experiment data inner vs outer wheel path + 1.0
    public static final double kTrackScrubFactor = 1.035;
    public static final double kGearRatio = 60.0 / 48.0;

    // Tuned dynamics
    public static final double kRobotLinearInertia = 12.701;  // kg
    public static final double kRobotAngularInertia = .38430233;  // kg m^2, tune: I = r * m * kAspin / kAlinear, r = trackwidth m = mass
    public static final double kRobotAngularDrag = 1;  // N*m / (rad/sec) TODO tune: guess and check
    public static final double kDriveVIntercept = 0.422303;  // V: use tuning op mode spin == .612673
    public static final double kDriveKv = 0.207290;  // V per rad/s: use tuning op mode spin == .230144
    public static final double kDriveKa = 0.045167;  // V per rad/s^2: use tuning op mode spin == .053805
    public static final double kDriveVelocityKd = 1;
    public static final double kDriveVelocityKf = 1;

    public static double ySlipDistanceFor1CMPS = 0.2;
    public static double xSlipDistanceFor1CMPS = 0.16;
    public static double turnSlipAmountFor1RPS = 0.07         ; // Experimentally this is .2 but like that is just not right

    // Vision
    public static boolean hsv = true;
    public static double lowerRedR = 130, upperRedR = 230, lowerRedG = 100, upperRedG = 200, lowerRedB = 100, upperRedB = 200;
    public static double lowerBlueR = 100, upperBlueR = 200, lowerBlueG = 135, upperBlueG = 235, lowerBlueB = 140, upperBlueB = 240;

    public static double lowerRedH = 159, upperRedH = 220, lowerRedS = 15, upperRedS = 255, lowerRedV = 70, upperRedV = 255;
    public static double lowerBlueH = 90, upperBlueH = 107, lowerBlueS = 50, upperBlueS = 235, lowerBlueV = 70, upperBlueV = 255;


    /* CONTROL LOOP GAINS */

    public static double kPathKX = 1.25;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering .57
    public static final double kPathMinLookaheadDistance = 6.0;  // inches 6
}
