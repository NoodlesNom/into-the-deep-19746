package org.firstinspires.ftc.teamcode.autonomous.rr.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.rr.drive.TankDrive;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.PinpointDrive;
@Disabled
public final class TurnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
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
                        .turn(360)







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
