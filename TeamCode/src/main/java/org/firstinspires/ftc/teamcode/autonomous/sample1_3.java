package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.rr.localizer.TwoDeadWheelLocalizer;

import java.util.Arrays;

@Config
@Autonomous(name = "1+3 sample, left", group = "Autonomous")


public class sample1_3 extends LinearOpMode {
    public Robot robot;
    public ElapsedTime timer;

    public class robotController {

        public class Update implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.update(timer.seconds());
                return true;
            }
        }

        public Action updateRobot() {
            return new Update();
        }

        public class OpenClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setClawPos(0);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setClawPos(1);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class PivotDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setServoTime(250);

                robot.mLift.setPivotPos(1);
                return false;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }

        public class PivotDownSlow implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setServoTime(1250);
                robot.mLift.setPivotPos(1);
                return false;
            }
        }

        public Action pivotDownSlow() {
            return new PivotDown();
        }

        public class PivotUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setServoTime(250);

                robot.mLift.setPivotPos(2);
                return false;
            }
        }

        public Action pivotUp() {
            return new PivotUp();
        }

        public class LiftDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(0, timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(3, timer.seconds());
                return !robot.mLift.closeEnough();
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftUpRailnBail implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setTargetPos(3, timer.seconds());
                return false;
            }
        }

        public Action liftUpRailnBail() {
            return new LiftUpRailnBail();
        }

        public class ArmDown implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setArmPos(1);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        public class ArmUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mLift.setArmPos(0);
                return false;
            }
        }

        public Action armUp() {
            return new ArmUp();
        }

        public class HangUp implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mHang.setScrewOpenLoop(0.8);
                return false;
            }
        }

        public Action hangUp() {
            return new HangUp();
        }

        public class HangStop implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.mHang.stop();
                return false;
            }
        }

        public Action hangStop() {
            return new HangStop();
        }
    }

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        robotController controller = new robotController();
        Pose2d initialPose = new Pose2d(24+(17.3/2), 72-((14.9/2)), Math.toRadians(0));
        //claw is 1.5 inches to the right
        robot = new Robot(this, initialPose , hardwareMap);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder traj = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(54.5, 54.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUp(),
                                controller.pivotDown()
                        )
                )
                .waitSeconds(0.4)
                .stopAndAdd(controller.openClaw())
                .waitSeconds(0.1)
                .stopAndAdd(
                        new SequentialAction(
                                controller.pivotUp(),
                                controller.closeClaw()
                        )
                )
                .waitSeconds(0.2)
                .stopAndAdd(controller.liftDown())
                .strafeToLinearHeading(new Vector2d(50, 50.5-(17.3/2)), Math.toRadians(-90))
                .stopAndAdd(
                        new SequentialAction(
                                controller.openClaw(),
                                controller.pivotDownSlow()
                        )

                )
                .waitSeconds(1.25)
                .stopAndAdd(
                        controller.closeClaw()
                )
                .waitSeconds(0.2)
                .stopAndAdd(
                        controller.pivotUp()
                )
                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUpRailnBail(),
                                controller.liftUpRailnBail(),
                                controller.liftUpRailnBail()
                        )
                )
                .strafeToLinearHeading(new Vector2d(54, 56), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUp(),
                                controller.pivotDown()
                        )
                )
                .waitSeconds(0.5)
                .stopAndAdd(controller.openClaw())
                .waitSeconds(0.1)
                .stopAndAdd(
                        new SequentialAction(
                                controller.pivotUp(),
                                controller.closeClaw()
                        )
                )
                .waitSeconds(0.2)
                .stopAndAdd(controller.liftDown())
                .strafeToLinearHeading(new Vector2d(59, 52-(17.3/2)), Math.toRadians(-90))
                .stopAndAdd(
                        new SequentialAction(
                                controller.openClaw(),
                                controller.pivotDownSlow()
                        )

                )
                .waitSeconds(1.25)
                .stopAndAdd(
                        controller.closeClaw()
                )
                .waitSeconds(0.1)
                .stopAndAdd(
                        controller.pivotUp()
                )
                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUpRailnBail(),
                                controller.liftUpRailnBail(),
                                controller.liftUpRailnBail()
                        )
                )
                .strafeToLinearHeading(new Vector2d(55.5, 57), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUp(),
                                controller.pivotDown()
                        )
                )
                .waitSeconds(0.5)
                .stopAndAdd(controller.openClaw())
                .waitSeconds(0.1)
                .waitSeconds(0.1)
                .stopAndAdd(
                        new SequentialAction(
                                controller.pivotUp(),
                                controller.closeClaw()
                        )
                )
                .waitSeconds(0.2)
                .stopAndAdd(controller.liftDown())

                .strafeToLinearHeading(new Vector2d(72-(14.9/2), 52-(17.3/2)), Math.toRadians(-91))
                .stopAndAdd(controller.armDown())
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(61-(14.9/2), 52.5-(17.3/2)), Math.toRadians(-96),new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(30),
                        new AngularVelConstraint(Math.PI)
                )))
                .stopAndAdd(controller.armUp())
                .strafeToLinearHeading(new Vector2d(61, 54.5-(17.3/2)), Math.toRadians(-90))
                .stopAndAdd(
                        new SequentialAction(
                                controller.openClaw(),
                                controller.pivotDownSlow()
                        )

                )
                .waitSeconds(1.25)
                .stopAndAdd(
                        controller.closeClaw()
                )
                .waitSeconds(0.2)
                .stopAndAdd(
                        controller.pivotUp()
                )

                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUpRailnBail(),
                                controller.liftUpRailnBail(),
                                controller.liftUpRailnBail()
                        )
                )
                .strafeToLinearHeading(new Vector2d(55.5, 57.5), Math.toRadians(45))
                .stopAndAdd(
                        new SequentialAction(
                                controller.liftUp(),
                                controller.pivotDown()
                        )
                )
                .waitSeconds(0.5)
                .stopAndAdd(controller.openClaw())
                .waitSeconds(0.1)
                .stopAndAdd(
                        new SequentialAction(
                                controller.pivotUp(),
                                controller.closeClaw()
                        )
                )
                .waitSeconds(0.2)
                .stopAndAdd(controller.liftDown())
                .setReversed(true)
                .stopAndAdd(
                        controller.hangUp()
                )
                .strafeToLinearHeading(new Vector2d(36, 8), Math.toRadians(-90))
                .stopAndAdd(
                        controller.hangStop()
                )
                .strafeToLinearHeading(new Vector2d(24, 8), Math.toRadians(-90));




        //.stopAndAdd()




        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {
            robot.update(timer.seconds());
            timer.reset();
            robot.autoInit();

        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = traj.build();

        robot.mLift.setClawPos(1);
        robot.mLift.setPivotPos(2);
        Actions.runBlocking(
                new ParallelAction(
                        controller.updateRobot(),
                        trajectoryActionChosen
                )
        );
        robot.stop();
    }
}
