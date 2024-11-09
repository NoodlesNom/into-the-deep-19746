package org.firstinspires.ftc.teamcode.autonomous.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TeamMarkerDetector extends OpenCvPipeline {
    private Mat rawImage = new Mat();
    private Mat rawHSVImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat maskMat = new Mat();
    private Mat hierarchy  = new Mat();

    private List<Integer> areasToReturn = new ArrayList<>();

    private Scalar lowerYellow = new Scalar(15, 50, 50);
    private Scalar upperYellow = new Scalar(120, 255, 255);

    private AllianceColor allianceColor = AllianceColor.BLUE;
    private int desiredHeight = 0;
    private int desiredWidth = 0;
    private int originalHeight = 0;
    private int originalWidth = 0;
    private int area;
    private TeamMarkerPosition teamMarkerPosition = TeamMarkerPosition.UNKNOWN;

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(rawImage);

        Rect rectCrop;
        originalHeight = rawImage.height();
        originalWidth = rawImage.width();
        int firstX;
//        if (allianceColor.equals(AllianceColor.RED)) {
//            firstX = 0;
//            rectCrop = new Rect(firstX, originalHeight - (desiredHeight == 0 ?  originalHeight : desiredHeight),
//                    desiredWidth == 0 ?  originalWidth : desiredWidth, desiredHeight == 0 ? originalHeight : desiredHeight);
//        } else {
            firstX = originalWidth - (desiredWidth == 0 ?  originalWidth : desiredWidth); // original - (original-desired)
            rectCrop = new Rect(firstX, originalHeight - (desiredHeight == 0 ?  originalHeight : desiredHeight),
                    (desiredWidth == 0 ?  originalWidth : desiredWidth), desiredHeight == 0 ? originalHeight : desiredHeight);
        //}

        Imgproc.cvtColor(rawImage, workingMat, Imgproc.COLOR_BGR2HSV);

        Core.inRange(workingMat, lowerYellow, upperYellow, rawHSVImage);

        /*//Moves vertical bars from each side
        int firstX = (originalWidth - (desiredWidth == 0 ?  originalWidth : desiredWidth)) / 2;
        int width = originalWidth - (originalWidth - (desiredWidth == 0 ?  originalWidth : desiredWidth)) / 2;
        rectCrop = new Rect(firstX, originalHeight - (desiredHeight == 0 ?  originalHeight : desiredHeight),
                width, desiredHeight == 0 ? originalHeight : desiredHeight);*/

        workingMat = workingMat.submat(rectCrop);

        int erosion_size = 5;
        int dilation_size = 5;

        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * erosion_size + 1, 2 * erosion_size + 1));
        Imgproc.erode(workingMat, workingMat, element);

        Mat element1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * dilation_size + 1, 2 * dilation_size + 1));
        Imgproc.dilate(workingMat, workingMat, element1);

        Imgproc.blur(workingMat, workingMat, new Size(6, 6), new Point(-1, -1));

        Core.inRange(workingMat, lowerYellow, upperYellow, maskMat);

        List<MatOfPoint> yellowContours = new ArrayList<>();
        Imgproc.findContours(maskMat, yellowContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(maskMat, yellowContours, -1, new Scalar(255, 30, 30), 2);

        List<Rect> yellowStones = new ArrayList<>();
        List<Integer> areas = new ArrayList<>();

        for (MatOfPoint contour : yellowContours) {
            Rect rect = Imgproc.boundingRect(contour);
            area = rect.width *  rect.height;
            areas.add(area);
            //if (area > 2500) {
                yellowStones.add(rect);
            //}
        }

        double widthOfBlocks = desiredWidth / 3.0;
        double[] blockPositions = new double[]{firstX + widthOfBlocks / 2.0, firstX + widthOfBlocks * 3.0 / 2.0, firstX + widthOfBlocks * 5.0 / 2.0};
        boolean left = true;
        boolean center = true;
        boolean right = true;

        for (int i = 0; i < yellowStones.size(); i++) {
            if (blockPositions[0] > yellowStones.get(i).x
                    && blockPositions[0] < (yellowStones.get(i).x + yellowStones.get(i).width))
                left = false;
            if (blockPositions[1] > yellowStones.get(i).x
                    && blockPositions[1] < (yellowStones.get(i).x + yellowStones.get(i).width))
                center = false;
            if (blockPositions[2] > yellowStones.get(i).x
                    && blockPositions[2] < (yellowStones.get(i).x + yellowStones.get(i).width))
                right = false;
        }

        if (left) {
            teamMarkerPosition = TeamMarkerPosition.LEFT;
        } else if (center) {
            teamMarkerPosition = TeamMarkerPosition.CENTER;
        } else if (right) {
            teamMarkerPosition = TeamMarkerPosition.RIGHT;
        }

        areasToReturn = areas;

        Imgproc.line(rawHSVImage, new Point(0, rectCrop.y), new Point(rectCrop.width, rectCrop.y), new Scalar(100, 255, 0));
        Imgproc.line(rawHSVImage, new Point(firstX, 0), new Point(firstX, originalHeight), new Scalar(100, 255, 0));
        Imgproc.line(rawHSVImage, new Point(desiredWidth, 0), new Point(desiredWidth, originalHeight), new Scalar(100, 255, 0));

        return rawHSVImage;
    }

    public void setDesiredHeight(int desiredHeight) {
        this.desiredHeight = desiredHeight;
    }

    public void setDesiredWidth(int desiredWidth) {
        this.desiredWidth = desiredWidth;
    }

    public int getOriginalHeight() {
        return originalHeight;
    }

    public int getOriginalWidth() {
        return originalWidth;
    }

    public String getAreas() {
        return areasToReturn.toString();
    }

    public void update(AllianceColor color, int lowerHue, int lowerSaturation,
                       int lowerValue, int higherHue, int higherSaturation, int higherValue) {
        this.allianceColor = color;
        lowerYellow = new Scalar(lowerHue, lowerSaturation, lowerValue);
        upperYellow = new Scalar(higherHue, higherSaturation, higherValue);
    }

    public TeamMarkerPosition getTeamMarkerPosition() {
        return this.teamMarkerPosition;
    }
}
