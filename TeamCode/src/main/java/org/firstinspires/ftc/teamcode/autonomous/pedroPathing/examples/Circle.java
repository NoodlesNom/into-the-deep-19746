package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.LConstants;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Circle", group = "Examples")
public class Circle extends OpMode {
    private Telemetry telemetryA;

    public static double RADIUS = 10;

    private Follower follower;
    int path = 0;

    private PathChain circle1, circle2;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(11,136-72, Math.toRadians(-27)));
        follower.drawOnDashBoard();

        circle1 = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Point(11.000, 136.000-72, Point.CARTESIAN),
                                new Point(20.000, 132.000-72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-27), Math.toRadians(-19))

                .build();
        circle2 = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Point(20, 132-72, Point.CARTESIAN),
                                new Point(11, 136-72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-19), Math.toRadians(-27))


                .build();


        follower.followPath(circle1);
        path = 1;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly circular shape of radius " + RADIUS
                            + ", starting on the right-most edge. So, make sure you have enough "
                            + "space to the left, front, and back to run the OpMode.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            path++;
            if (path%2==0) {
                follower.followPath(circle2, true);
            }else{
                follower.followPath(circle1, true);
            }
        }

        follower.telemetryDebug(telemetryA);
    }
}
