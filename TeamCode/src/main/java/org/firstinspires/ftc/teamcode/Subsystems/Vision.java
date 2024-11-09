package org.firstinspires.ftc.teamcode.Subsystems;//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.visionPipeline;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//public class Vision extends Subsystem {
//
//    private VisionPortal visionPortal;
//    private HardwareMap hwMap;
//
//    private AprilTagProcessor aprilTag;
//    public visionPipeline visionPipeline;
//
//    public Vision(HardwareMap map)
//    {
//        hwMap = map;
//    }
//
//    @Override
//    public void autoInit()
//    {
//        visionPipeline = new visionPipeline(1, 600, 400, org.firstinspires.ftc.teamcode.visionPipeline.Color.RED);
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hwMap.get(WebcamName.class, "Webcam"));
//        builder.addProcessor(aprilTag);
//        builder.addProcessor(visionPipeline);
//        visionPortal = builder.build();
//    }
//
//    public void stopVision()
//    {
//        // This  line results in an exception
//        //visionPortal.stopStreaming();
//        visionPortal.close();
//    }
//
//    public int doVision()
//    {
//        return visionPipeline.getPosition();
//    }
//
//    public List<AprilTagDetection> getAprilTags()
//    {
//        return aprilTag.getDetections();
//    }
//
//    @Override
//    public void teleopInit()
//    {
//        visionPipeline = new visionPipeline(1, 600, 400, org.firstinspires.ftc.teamcode.visionPipeline.Color.RED);
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hwMap.get(WebcamName.class, "Webcam"));
//        builder.addProcessor(aprilTag);
//        builder.addProcessor(visionPipeline);
//        visionPortal = builder.build();
//    }
//
//    @Override
//    public void update(double timestamp)
//    {
//        // we don't call this outside of init for now
//    }
//
//    @Override
//    public void stop()
//    {
//        // we stop the webcam before the auto runs for now
//    }
//
//    @Override
//    public String getTelem(double time)
//    {
//        // we don't want to print out anything because we don't do auto mosaic yet
//        return "";
//    }
//
//    @Override
//    public void readPeriodicInputs(double time) {
//        // no reason for anything here in any situation
//    }
//
//    @Override
//    public void writePeriodicOutputs() {
//        // no reason for anything here in any situation
//    }
//}
