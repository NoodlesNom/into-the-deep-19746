package org.firstinspires.ftc.teamcode.autonomous.vision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.utils.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class VisionLibrary {
    private LinearOpMode opMode;
    OpenCvWebcam webcam;
    public TeamMarkerDetector teamMarkerDetector;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1;
    double fy = 1;
    double cx = 1;
    double cy = 1;

    // UNITS ARE METERS
    double tagsize = 0.166;

    static final double FEET_PER_METER = 3.28084;

    public VisionLibrary(LinearOpMode opMode)
    {
        this.opMode = opMode;
    }

    public void init() {
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                // Changed by ToddS webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.addData("Camera Error:",errorCode);
                opMode.telemetry.update();
                opMode.sleep(3000);
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    // First 20 AprilTags here:
    // https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf
    public SignalSleevePosition getSignalSleevePosition() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        int tag_id = -1;

        if(currentDetections.size() != 0) {
            tag_id = currentDetections.get(0).id;
        } else {
            return(SignalSleevePosition.UNKNOWN);
        }

        switch (tag_id % 3) {
            case 0:  return(SignalSleevePosition.ONE);
            case 1:  return(SignalSleevePosition.TWO);
            case 2:  return(SignalSleevePosition.THREE);
            default: return(SignalSleevePosition.UNKNOWN);
        }
    }

    public void stopVision() {
        webcam.stopStreaming();
    }
}