package org.firstinspires.ftc.teamcode.autonomous.rr.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrivePeriodic;
import org.firstinspires.ftc.teamcode.autonomous.rr.drive.TankDrive;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(24-7.25, 7.25, Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive pd = new PinpointDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                pd.actionBuilder(beginPose)
//                        .splineTo(new Vector2d(30, 42), Math.PI / 2)
//                        .splineTo(new Vector2d(30, 150), Math.PI / 2)
//                        .strafeToLinearHeading(new Vector2d(30,0), Math.toRadians(0))
//                        .turn(Math.toRadians(90))
//                        .turn(Math.toRadians(45))
//                        .turn(Math.toRadians(-45))
//                        .splineTo(new Vector2d(20, 120), Math.PI / 2)
//                        .splineTo(new Vector2d(20, 150), Math.PI / 2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(30, 30), 3*Math.PI / 2)
//                        .splineTo(new Vector2d(30, 0), 3*Math.PI / 2)
//                        .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(35,49), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40,90))
                        .splineToConstantHeading(new Vector2d(42,57), Math.toRadians(0), new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-40,50))
                        .splineToConstantHeading(new Vector2d(48,49), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40,90))
                        .splineToConstantHeading(new Vector2d(48,21), Math.toRadians(-90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45,90))
                        .strafeToLinearHeading(new Vector2d(48,21),Math.toRadians(90))
                        .waitSeconds(0.05)
                        .splineToConstantHeading(new Vector2d(48,49), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40,90))
                        .splineToConstantHeading(new Vector2d(53,57), Math.toRadians(0), new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-40,50))
                        .splineToConstantHeading(new Vector2d(58,49), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40,90))
                        .splineToConstantHeading(new Vector2d(58,21), Math.toRadians(-90),  new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45,90))
                        .strafeToLinearHeading(new Vector2d(58,21),Math.toRadians(90))

                        .waitSeconds(0.05)
                        .splineToConstantHeading(new Vector2d(58,49), Math.toRadians(90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40,90))
                        .splineToConstantHeading(new Vector2d(60.5,56), Math.toRadians(0), new TranslationalVelConstraint(30 ), new ProfileAccelConstraint(-40,50))
                        .splineToConstantHeading(new Vector2d(63,49), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-40,90))
                        .splineToConstantHeading(new Vector2d(63,21), Math.toRadians(-90), new TranslationalVelConstraint(60 ), new ProfileAccelConstraint(-45,90))
                        .strafeToLinearHeading(new Vector2d(63,21),Math.toRadians(90))

                        .waitSeconds(0.1)
                        .splineToLinearHeading(new Pose2d(36,8, Math.toRadians(90)), Math.toRadians(270))
                        .waitSeconds(0.1)
                        .splineToConstantHeading(new Vector2d(0, 40), Math.toRadians(90))
                        .waitSeconds(1)









                        //.turn(90)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
