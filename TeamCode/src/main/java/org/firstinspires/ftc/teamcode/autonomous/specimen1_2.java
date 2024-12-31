//package org.firstinspires.ftc.teamcode.autonomous;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.autonomous.rr.drive.MecanumDrive;
//
//@Config
//@Autonomous(name = "1+2 specimen, right", group = "Autonomous")
//
//
//public class specimen1_2 extends LinearOpMode {
//    public Robot robot;
//    public ElapsedTime timer;
//
//    public class robotController {
//
//        public class Update implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.update(timer.seconds());
//                return true;
//            }
//        }
//
//        public Action updateRobot() {
//            return new Update();
//        }
//
//        public class OpenClaw implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setClawPos(0);
//                return false;
//            }
//        }
//
//        public Action openClaw() {
//            return new OpenClaw();
//        }
//
//        public class CloseClaw implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setClawPos(1);
//                return false;
//            }
//        }
//
//        public Action closeClaw() {
//            return new CloseClaw();
//        }
//
//        public class PivotDown implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setPivotPos(1);
//                return false;
//            }
//        }
//
//        public Action pivotDown() {
//            return new PivotDown();
//        }
//
//        public class PivotUp implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setPivotPos(0);
//                return false;
//            }
//        }
//
//        public Action pivotUp() {
//            return new PivotUp();
//        }
//
//        public class LiftDown implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setTargetPos(0, timer.seconds());
//                return !robot.mLift.closeEnough();
//            }
//        }
//
//        public Action liftDown() {
//            return new LiftDown();
//        }
//
//        public class LiftUp implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setTargetPos(2, timer.seconds());
//                return !robot.mLift.closeEnough();
//            }
//        }
//
//        public Action liftUp() {
//            return new LiftUp();
//        }
//
//        public class LiftPlace implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setTargetPos(1, timer.seconds());
//                return !robot.mLift.closeEnough();
//            }
//        }
//
//        public Action liftPlace() {
//            return new LiftPlace();
//        }
//
//        public class LiftUpRailnBail implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setTargetPos(2, timer.seconds());
//                return false;
//            }
//        }
//
//        public Action liftUpRailnBail() {
//            return new LiftUpRailnBail();
//        }
//
//        public class LiftDownRailnBail implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setTargetPos(0, timer.seconds());
//                return false;
//            }
//        }
//
//        public Action liftDownRailnBail() {
//            return new LiftDownRailnBail();
//        }
//
//        public class ArmDown implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setArmPos(1);
//                return false;
//            }
//        }
//        public Action armDown() {
//            return new ArmDown();
//        }
//
//        public class ArmUp implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mLift.setArmPos(0);
//                return false;
//            }
//        }
//
//        public Action armUp() {
//            return new ArmUp();
//        }
//
//        public class HangUp implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mHang.setScrewOpenLoop(0.7);
//                return false;
//            }
//        }
//
//        public Action hangUp() {
//            return new HangUp();
//        }
//
//        public class HangStop implements Action {
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                robot.mHang.stop();
//                return false;
//            }
//        }
//
//        public Action hangStop() {
//            return new HangStop();
//        }
//    }
//
//    @Override
//    public void runOpMode() {
//        timer = new ElapsedTime();
//        robotController controller = new robotController();
//        Pose2d initialPose = new Pose2d((14.9/2), 72-(17.3/2), Math.toRadians(-90));
//        //claw is 1.5 inches to the right
//        robot = new Robot(this, initialPose , hardwareMap);
//        MecanumDrive drive = new MecanumDrive(this.hardwareMap, initialPose);
//
//        // vision here that outputs position
//        int visionOutputPosition = 1;
//
//        TrajectoryActionBuilder traj = drive.actionBuilder(initialPose)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.liftUpRailnBail(),
//                                controller.liftUpRailnBail(),
//                                controller.liftUpRailnBail(),
//                                controller.pivotUp()
//                        )
//                )
//                .strafeToLinearHeading(new Vector2d((14.9/2), 25+(17.3/2)), Math.toRadians(-90))
//                .waitSeconds(0.2)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.liftUp(),
//                                controller.liftPlace()
//                        )
//                )
//                .waitSeconds(0.1)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.openClaw(),
//                                controller.liftDownRailnBail(),
//                                controller.liftDownRailnBail(),
//                                controller.liftDownRailnBail()
//                        )
//                )
//                .waitSeconds(0.1)
//                .setTangent(Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-24,36), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-36,26), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-36,20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-48,13), Math.toRadians(180))
//                .waitSeconds(0.1)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-58,55, Math.toRadians(-55)), Math.toRadians(-55+180))
//                .setReversed(false)
//                .strafeToLinearHeading(new Vector2d(-36,70-(17.3/2)), Math.toRadians(90))
//                .waitSeconds(0.2)
//                .stopAndAdd(controller.closeClaw())
//                .waitSeconds(0.2)
//                .stopAndAdd(controller.liftUpRailnBail())
//                .stopAndAdd(controller.liftUpRailnBail())
//                .stopAndAdd(controller.liftUpRailnBail())
//                .waitSeconds(0.3)
//                .strafeToLinearHeading(new Vector2d((14.9/2-4), 25+(17.3/2)), Math.toRadians(-90))
//                .waitSeconds(0.2)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.liftUp(),
//                                controller.liftPlace()
//                        )
//                )
//                .waitSeconds(0.1)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.openClaw(),
//                                controller.liftDownRailnBail(),
//                                controller.liftDownRailnBail(),
//                                controller.liftDownRailnBail()
//                        )
//                )
//                .waitSeconds(0.1)
//                .strafeToLinearHeading(new Vector2d(-36,70-(17.3/2)), Math.toRadians(90))
//                .waitSeconds(0.2)
//                .stopAndAdd(controller.closeClaw())
//                .waitSeconds(0.2)
//                .stopAndAdd(controller.liftUpRailnBail())
//                .stopAndAdd(controller.liftUpRailnBail())
//                .stopAndAdd(controller.liftUpRailnBail())
//                .waitSeconds(0.3)
//                .strafeToLinearHeading(new Vector2d((14.9/2-8), 25+(17.3/2)), Math.toRadians(-90))
//                .waitSeconds(0.2)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.liftUp(),
//                                controller.liftPlace()
//                        )
//                )
//                .waitSeconds(0.1)
//                .stopAndAdd(
//                        new SequentialAction(
//                                controller.openClaw(),
//                                controller.liftDownRailnBail(),
//                                controller.liftDownRailnBail(),
//                                controller.liftDownRailnBail()
//                        )
//                )
//                .waitSeconds(0.1)
//                .strafeToLinearHeading(new Vector2d((14.9/2-5), 25+(17.3/2)), Math.toRadians(-90));
//
//
//
//
//
//        //.stopAndAdd()
//
//
//
//
//        // actions that need to happen on init; for instance, a claw tightening.
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            robot.update(timer.seconds());
//            timer.reset();
//            robot.autoInit();
//
//        }
//
//        int startPosition = visionOutputPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Action trajectoryActionChosen = traj.build();
//
//        robot.mLift.setClawPos(1);
//        robot.mLift.setPivotPos(2);
//        Actions.runBlocking(
//                new ParallelAction(
//                        controller.updateRobot(),
//                        trajectoryActionChosen
//
//                )
//        );
//        robot.stop();
//    }
//}