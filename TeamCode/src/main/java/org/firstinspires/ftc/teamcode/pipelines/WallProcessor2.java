package org.firstinspires.ftc.teamcode.pipelines;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.sun.source.tree.Tree;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor.COLORS;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;


import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

public class WallProcessor2 implements VisionProcessor {
    // STATIC CONSTANTS
    public static Scalar blue = new Scalar(138,43,226,255);
    public static Scalar purple = new Scalar(138,43,226,255);
    public static Scalar red = new Scalar(255,0,0,255);
    public static Scalar green = new Scalar(0,255,0,255);
    public static Scalar white = new Scalar(255,255,255,255);
    public static Scalar yellow = new Scalar(196,180,84,255);

    public static Scalar green_lower = new Scalar(59.5, 38.3, 14.2);
    public static Scalar green_upper = new Scalar(195.5, 109.1, 167.2);
    public static Scalar white_lower = new Scalar(129, 113, 0);
    public static Scalar white_upper = new Scalar(255, 255, 255);
    public static Scalar yellow_lower = new Scalar(69.4, 0, 175.7);
    public static Scalar yellow_upper = new Scalar(255, 168.6, 255.0);
    public static Scalar purple_lower = new Scalar(0, 0, 45);
    public static Scalar purple_upper = new Scalar(255, 255, 120.4);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        constructMatrix();
    }



    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public enum COLORS {
        PURPLE (purple_lower, purple_upper, purple),
        GREEN (green_lower, green_upper, green),
        YELLOW (yellow_lower, yellow_upper, yellow),
        WHITE (white_lower, white_upper, white),
        NONE (white_lower, white_upper, white);

        public Scalar lower;
        public Scalar upper;
        public Scalar displayColor;

        COLORS(Scalar lower, Scalar upper, Scalar displayColor) {
            this.lower = lower;
            this.upper = upper;
            this.displayColor = displayColor;
        }
    }

    public class Pixel {
        public Point point;
        public COLORS color;
        public double col;
        public Pixel(Point point, COLORS color) {
            this.point = point;
            this.color = color;
        }

        public double getRawRow() {
            // return Math.ceil(point.y/45);
            return Math.ceil(point.y/38);
        }

        public double getRawCol() {
            return Math.ceil(point.x/38);
        }
        public void setCol(double col) {
            this.col = col;
        }
    }

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    // UNITS ARE METERS
    public static double TAG_SIZE = 0.166;

    // instance variables

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;
    Mat resized;
    Mat mat;

    double tagsizeX = TAG_SIZE;
    double tagsizeY = TAG_SIZE;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    boolean isRight;
    public boolean waiting = true;

    ArrayList<Pixel> pixels = new ArrayList<>();
    Telemetry telemetry;

    public WallProcessor2(Telemetry telemetry) {
        this.telemetry = telemetry;
        constructMatrix();
    }

//    @Override
//    public void init(Mat frame)
//    {
//        // Allocate a native context object. See the corresponding deletion in the finalizer
//    }

    @Override
    public void finalize()
    {
        // Delete the native context we created in the init() function
        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        //    Core.rotate(input, input, Core.ROTATE_180);

        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        //     labelPixels(input, purple_contours, COLORS.PURPLE);
        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, TAG_SIZE, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        if (detections.size() > 0) {
            for (AprilTagDetection d : detections) {
                if (d.id == 5) {
                    Imgproc.circle(input, d.corners[2], 5, new Scalar(255, 0, 0, 255), -1);
                    Imgproc.circle(input, d.corners[1], 5, new Scalar(0, 255, 0, 255), -1);
                    Imgproc.circle(input, d.corners[3], 5, new Scalar(0, 0, 255, 255), -1);
                    telemetry.addLine("corner 2: " + d.corners[2].toString());
                    telemetry.addLine("corner 1: " + d.corners[1].toString());
                    double inch_to_pixel =  Math.sqrt(Math.pow(d.corners[1].x-d.corners[2].x, 2) + Math.pow(d.corners[1].y-d.corners[2].y, 2))/2;
                    telemetry.addLine("size " + inch_to_pixel);




                    Orientation rot = Orientation.getOrientation(d.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                    double tanofangtle = (d.corners[2].y-d.corners[1].y)/(d.corners[2].x-d.corners[1].x);
                    double angle = Math.toDegrees(Math.atan(tanofangtle));
                    if (angle < 0) {
                        angle += 90;
                    } else {
                        angle -= 90;
                    }
                    telemetry.addLine("angle " + angle);
                    Point rotPoint = new Point(input.cols() / 2.0,
                            input.rows() / 2.0);
                    // Mat rotMat = Imgproc.getRotationMatrix2D(rotPoint, -angle, 1);
                    // Imgproc.warpAffine(input, input, rotMat, input.size(),
                    // Imgproc.WARP_INVERSE_MAP);
                    Rect crop = new Rect(0, 0, 0, 0);
                    // if (d.id == 1 || d.id ==4) {
                    int xStart = (int) ((int) d.corners[3].x-(3*inch_to_pixel));
                    int yStart = (int) Math.max(0, Math.min(input.size().height-0, (int) (8*inch_to_pixel)));
                    int xOver = (int) Math.max(0, Math.min(input.size().width-xStart, (int) (8*inch_to_pixel)));
                    int yOver = (int) Math.max(0, Math.min(input.size().height-yStart,(int) ((int)d.corners[1].y-(10*inch_to_pixel))));
                    telemetry.addData("input height", input.size().height);
                    telemetry.addData("xStart", xStart);
                    telemetry.addData("yShift", d.corners[3].x);
                    crop = new Rect(xStart, yStart, xOver, yOver);
                    // //} else if (d.id == 2) {
                    //     int xStart = 0;
                    //     int yStart = 0;
                    //     int xOver = (int) Math.max(0, Math.min(input.size().width-xStart, (int) ((int) d.corners[1].x+(4*inch_to_pixel))));
                    //     int yOver = (int) Math.max(0, Math.min(input.size().height-yStart,(int) ((int)d.corners[1].y-(2*inch_to_pixel))));
                    //     crop = new Rect(xStart, yStart, xOver, yOver);
                    // } else {
                    //     int xStart = (int)(d.corners[1].x-(5*inch_to_pixel));
                    //     int yStart = 50;
                    //     int xOver = (int) Math.max(0, Math.min(input.size().width-xStart, ((int) +(20*inch_to_pixel))));
                    //     int yOver = (int) Math.max(0, Math.min(input.size().height-yStart,(int) ((int)d.corners[1].y-(6*inch_to_pixel))));
                    //     crop = new Rect(xStart, yStart, xOver, yOver);
                    // }
                    input = new Mat(input, crop);
                    List<MatOfPoint> contours = getPixelsByColor(input, COLORS.YELLOW);
                    double maxVal = 0;
                    int maxValIdx = 0;
                    if (contours.size() == 0) {
                        telemetry.update();
                        resized = new Mat();

                        Imgproc.resize(mat, resized, new Size(mat.width() / 4, mat.height() / 4));
                        // Imgproc.putText(
                        //         resized,
                        //         location.toString(),
                        //         new Point(50, 75),
                        //         Imgproc.FONT_HERSHEY_PLAIN,
                        //         5,
                        //         new Scalar(255, 255, 255),
                        //         5
                        // );
                        return input;
                    }

                    for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
                    {
                        double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                        if (maxVal < contourArea)
                        {

                            maxVal = contourArea;
                            maxValIdx = contourIdx;
                        }
                    }

                    Moments m = Imgproc.moments(contours.get(maxValIdx));
                    //   telemetry.addData("detected width", contourRectMax.size.width);
                    //  telemetry.addData("detected height", contourRectMax.size.height);
                    double center = m.m10/m.m00;

                    if (center > input.width()/2) {
                        telemetry.addData("detected right", true);
                        isRight = true;

                    } else {
                        telemetry.addData("detected left", true);
                        isRight = false;
                    }
                    waiting = false;
                }}} else {
            waiting = true;
        }



        // Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2Lab);

        //      pixels = new ArrayList<>();
        //     //  input = getPixelsByColor(input, COLORS.WHITE);


        //     input = labelPixels(input, contours);
        //     if (detections.size() > 0 && pixels.size() > 0) {
        //    telemetry.addData("available", getAvailablePlaces());
        //ArrayList<Pixel> surrounding = getSurrounding(4, 2);
        //COLORS color = getMosaicColor(4, 2);
        // ArrayList<COLORS> colors = new ArrayList<>();
        // for (Pixel pixel : surrounding) {
        //     colors.add(pixel.color);
        // }
        // int current_score = getScore();
        // telemetry.addData("current score",  current_score);
        // telemetry.addData("mosaics", getMosaicColor(10., 2.));
        //  if (true) {
        //     Imgproc.putText(input, "Auto Placement: "+getAutoPixelLoc(), new Point(0, 75), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
        // }
        // //telemetry.addData("surrounding", color);
        //   Pixel next = getNextPixel();
        // telemetry.addData("next", next.getRawCol() + " " + next.getRawRow() + " " + next.color);
        // pixels.add(next);
        // Pixel next1 = getNextPixel();
        // telemetry.addData("next 1", next1.getRawCol() + " " + next1.getRawRow() + " " + next1.color);
        // pixels.add(next1);
        // int next_score = getScore();
        // telemetry.addData("score",  next_score);
        // drawHexagon(input, next.point.x-(next.getRawRow() % 2 == 0 ? 27 : 9), next.point.y-18, 18,next.color.displayColor);
        // Imgproc.circle(input, new Point(next.point.x-(next.getRawRow() % 2 == 0? 27 : 9), next.point.y-18), 5, next.color.displayColor, -1);
        // drawHexagon(input, next1.point.x-(next1.getRawRow() % 2 == 0 ? 27 : 9), next1.point.y-18, 18,next1.color.displayColor);
        // Imgproc.circle(input, new Point(next1.point.x-(next1.getRawRow() % 2 == 0 ? 27 : 9), next1.point.y-18), 5, next1.color.displayColor, -1);
        // Pixel leftClaw;
        // Pixel rightClaw;
        // if (getClawPos(next, next1)) {
        //     leftClaw = next1;
        //     rightClaw = next;
        // } else {
        //     leftClaw = next;
        //     rightClaw = next1;
        // }
        // Imgproc.putText(input, "Current Score: "+current_score, new Point(0, 15), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
        // Imgproc.putText(input, "Next Score: "+next_score, new Point(0, 30), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
        // Imgproc.putText(input, "Left Claw: "+leftClaw.color, new Point(0, 45), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
        // Imgproc.putText(input, "Right Claw: "+rightClaw.color, new Point(0, 60), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));


        telemetry.update();
        // } else {
        // telemetry.update();
        // }

        return input;
    }

    public List<MatOfPoint> getPixelsByColor(Mat input, COLORS color) {




        mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2Lab);
        //  Rect crop = new Rect(0, input.height()/2, input.width(), input.height()/2);
        //  mat = new Mat(mat, crop);


        Mat thresh = new Mat();

        Core.inRange(mat, color.lower, color.upper, thresh);
        List<MatOfPoint> contours = new ArrayList();

        Mat heirarchy = new Mat();
        Imgproc.findContours(thresh, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);




        return contours;
    }

    public Mat labelPixels(Mat input, List<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.04 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            Point[] points = approxCurve.toArray();
            int numVertices = points.length;

            Rect rect = Imgproc.boundingRect(contour);


            // Convert the MatOfPoint2f back to MatOfPoint
            MatOfPoint approxContour = new MatOfPoint();
            approxCurve.convertTo(approxContour, CvType.CV_32S);

            if (numVertices == 6 && (rect.width < 25 && rect.width > 15)) {
                COLORS color = getPixelColor(input, approxContour);
                //  Draw the approximate polygon
                List<MatOfPoint> drawContours = new ArrayList<>();
                drawContours.add(approxContour);
                Imgproc.drawContours(input, drawContours, 0, new Scalar(0, 255, 0), 2);
                Point point = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
                Pixel pixel = new Pixel(point, color);
                pixels.add(pixel);
                Imgproc.circle(input, point, 5, new Scalar(255, 0, 0, 255), -1);
                Imgproc.putText(input, color.toString(), new Point(points[0].x, points[0].y), Imgproc.FONT_HERSHEY_COMPLEX, 0.25, new Scalar(0, 0, 255));
                Imgproc.putText(input, pixel.getRawCol() + " " + pixel.getRawRow(), new Point(points[0].x, points[0].y+10), Imgproc.FONT_HERSHEY_COMPLEX, 0.25, new Scalar(0, 0, 255));
            }
        }
        return input;
    }

    public COLORS getPixelColor(Mat input, MatOfPoint contour) {

        Mat contourMat = new Mat(input, Imgproc.boundingRect(contour));
        Mat mat = new Mat();
        Imgproc.cvtColor(contourMat, mat, Imgproc.COLOR_RGB2Lab);
        //  Rect crop = new Rect(0, input.height()/2, input.width(), input.height()/2);
        //  mat = new Mat(mat, crop);
        for (COLORS color : COLORS.values()) {
            Mat thresh = new Mat();

            Core.inRange(mat, color.lower, color.upper, thresh);
            List<MatOfPoint> contours = new ArrayList();
            Mat heirarchy = new Mat();
            Imgproc.findContours(thresh, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


            if (contours.size() >0) {
                return color;
            }
        }



        return COLORS.WHITE;

    }

    public TreeMap<Double, TreeMap<Double, Pixel>> getPixels() {
        TreeMap<Double, TreeMap<Double, Pixel>> map = new TreeMap<>();
        for (Pixel pixel : pixels) {
            if (!map.containsKey(pixel.getRawRow())) {
                map.put(pixel.getRawRow(), new TreeMap<>());
            }
            pixel.setCol(map.get(pixel.getRawRow()).size()+1);
            //   Imgproc.putText(input, pixel.col + " " + pixel.getRawRow(), new Point(pixel.point.x, pixel.point.y+10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
            map.get(pixel.getRawRow()).put(pixel.getRawCol(), pixel);
        }
        //     telemetry.addData("pixels", map.toString());
        return map;
    }

    public Pixel getNextPixel() {
        TreeMap<Double, TreeMap<Double, Pixel>> map = getPixels();
        ArrayList<Double> available = getAvailablePlaces();
        for (int i = 0; i < available.size(); i++) {
            if (available.get(i) == null) {
                continue;
            }
            if (getMosaicColor(available.get(i), i+1) != COLORS.WHITE) {
                //        telemetry.addData("nextup", available.get(i) + " "+ i+1+ " "+ getMosaicColor(available.get(i), i+1));
                return new Pixel(new Point((i+1)*38, available.get(i)*38), getMosaicColor(available.get(i), i+1));
            }
            if (getSetlineByPixel(available.get(i), i+1)) {
                telemetry.addData("setline", available.get(i) + " "+ i+1+ " "+ getMosaicColor(available.get(i), i+1));
                return new Pixel(new Point((i+1)*38, available.get(i)*38), COLORS.WHITE);
            }
        }
        return new Pixel(new Point((available.size()+1)*38, available.get(available.size()-1)*38), COLORS.WHITE);
    }

    public ArrayList<Double> getMissingSlots(TreeMap<Double, Pixel> row) {
        ArrayList<Double> missing = new ArrayList<>();
        for (int i = 1; i < 7; i++) {
            if (!row.containsKey((double) i)) {
                missing.add((double) i);
            }
        }
        //    telemetry.addData("missing", missing.toString());
        return missing;
    }

//    public ArrayList<Double> getAvailablePlaces() {
//        TreeMap<Double, TreeMap<Double, Pixel>> map = getPixels();
//        ArrayList<Double> available = new ArrayList<>();
//        for (int i = 1; i < 7; i++) {
//            Entry<Double, TreeMap<Double, Pixel>> rowMap = map.firstEntry();
//            Double rowKey = rowMap.getKey();
//            TreeMap<Double, Pixel> row = rowMap.getValue();
//            while (map.containsKey(rowKey+1) && !map.get(rowKey+1).containsKey((double) i)) {
//                row = map.get(rowKey+1);
//                rowKey = rowKey+1;
//            }
//            available.add(rowKey);
//        }
//
//        telemetry.addData("available", available.toString());
//        return available;
//    }

    public ArrayList<Double> getAvailablePlaces() {
        TreeMap<Double, TreeMap<Double, Pixel>> map = getPixels();
        ArrayList<Double> available = new ArrayList<>();
        for (int i = 1; (i < 8); i++) {
            Double rowKey = map.firstKey();

            while (pixelsToColors(getFoundation(rowKey-1, i, false)).contains(COLORS.NONE)) {
                if (rowKey <= map.lastKey()) {
                    rowKey += 1;
                } else {
                    break;
                }
            }
            telemetry.addData("is8", rowKey+" "+pixelsToColors( getFoundation(rowKey-1, i, false)));

            if (map.containsKey(rowKey)&& map.get(rowKey).containsKey((double) i) && pixelsToColors( getFoundation(rowKey-1, i, false)).contains(COLORS.NONE)) {
                available.add(null);
            } else {
                available.add(rowKey-1);
            }
        }
        telemetry.addData("available", available);
        return available;
    }

    public ArrayList<COLORS> pixelsToColors(ArrayList<Pixel> pixels) {
        ArrayList<COLORS> colors = new ArrayList<>();
        for (Pixel pixel : pixels) {
            colors.add(pixel.color);
        }
        return colors;
    }

    public ArrayList<Pixel> getFoundation(double row, double col, boolean top) {
        TreeMap<Double, TreeMap<Double, Pixel>> map = getPixels();
        int shiftMult = row % 2 == 0 ? -1 : 1;
        ArrayList<Pixel> foundation = new ArrayList<>();
        if ((!top && map.containsKey(row+1)) || (top && map.containsKey(row-1))) {
            TreeMap<Double, Pixel> foundationRow = (top ? map.get(row-1) : map.get(row+1));
            if (shiftMult < 0) {
                if (foundationRow.containsKey(col-1)) {
                    foundation.add(foundationRow.get(col - 1));
                } else if (col == 1) {
                    foundation.add(new Pixel(new Point(0, 0), COLORS.WHITE));
                } else {
                    foundation.add(new Pixel(new Point(0, 0), COLORS.NONE));
                }
            }
            if (foundationRow.containsKey(col)) {
                foundation.add(foundationRow.get(col));
            } else if (col == 7) {
                foundation.add(new Pixel(new Point(0, 0), COLORS.WHITE));
            } else {
                foundation.add(new Pixel(new Point(0, 0), COLORS.NONE));
            }
            if (shiftMult > 0) {
                if (foundationRow.containsKey(col+1)) {
                    foundation.add(foundationRow.get(col+1));
                } else {
                    foundation.add(new Pixel(new Point(0, 0), COLORS.NONE));
                }
            }
        } else {
            foundation.add(new Pixel(new Point(0, 0), COLORS.WHITE));
            foundation.add(new Pixel(new Point(0, 0), COLORS.WHITE));
        }
        return foundation;
    }

    public ArrayList<Pixel> getSurrounding(double row, double col) {
        ArrayList<Pixel> surrounding = new ArrayList<>();
        TreeMap<Double, TreeMap<Double, Pixel>> map = getPixels();
        TreeMap<Double, Pixel> pixelRow = map.getOrDefault(row, new TreeMap<>());
        if (pixelRow.containsKey(col-1)) {
            surrounding.add(pixelRow.get(col-1));
        } else if (col == 1) {
            surrounding.add(new Pixel(new Point(0, 0), COLORS.WHITE));
        } else {
            surrounding.add(new Pixel(new Point(0, 0), COLORS.NONE));
        }
        surrounding.addAll(getFoundation(row, col, false));
        if (pixelRow.containsKey(col+1)) {
            surrounding.add(pixelRow.get(col+1));
        } else if (col == 1) {
            surrounding.add(new Pixel(new Point(0, 0), COLORS.WHITE));
        } else {
            surrounding.add(new Pixel(new Point(0, 0), COLORS.NONE));
        }
        surrounding.addAll(getFoundation(row, col, true));
        return surrounding;
    }

    public COLORS getMosaicColor(double row, double col) {
        ArrayList<Pixel> surrounding = getSurrounding(row, col);
        Map<COLORS, Double> colors = getSurroundingColorCount(surrounding);
        double surroundingColorPixelCount = 6-colors.getOrDefault(COLORS.WHITE, 0.0)-colors.getOrDefault(COLORS.NONE, 0.0);
        for (Pixel pixel : surrounding) {
            if (pixel.color != COLORS.WHITE && pixel.color != COLORS.NONE) {
                telemetry.addData("pixel", getSurroundingColorCount(getSurrounding(pixel.getRawRow(), pixel.getRawCol())));
                Map<COLORS, Double> scolors = getSurroundingColorCount(getSurrounding(pixel.getRawRow(), pixel.getRawCol()));
                double pnw =6-scolors.getOrDefault(COLORS.WHITE, 0.0)-scolors.getOrDefault(COLORS.NONE, 0.0);
                if (pnw != surroundingColorPixelCount-1) return COLORS.WHITE;
            }
        }


        if (surroundingColorPixelCount == 2) {

            telemetry.addData("colors", colors.toString());
            if (colors.containsKey(COLORS.PURPLE) && colors.get(COLORS.PURPLE) == 2) {
                return COLORS.PURPLE;
            }
            if (colors.containsKey(COLORS.GREEN) && colors.get(COLORS.GREEN) == 2) {
                return COLORS.GREEN;
            }
            if (colors.containsKey(COLORS.YELLOW) && colors.get(COLORS.YELLOW) == 2) {
                return COLORS.YELLOW;
            }
            if (colors.containsKey(COLORS.PURPLE) && colors.containsKey(COLORS.GREEN) && colors.get(COLORS.PURPLE) == 1 && colors.get(COLORS.GREEN) == 1) {
                return COLORS.YELLOW;
            }
            if (colors.containsKey(COLORS.PURPLE) && colors.containsKey(COLORS.YELLOW) && colors.get(COLORS.PURPLE) == 1 && colors.get(COLORS.YELLOW) == 1) {
                return COLORS.GREEN;
            }
            if (colors.containsKey(COLORS.GREEN) && colors.containsKey(COLORS.YELLOW) && colors.get(COLORS.GREEN) == 1 && colors.get(COLORS.YELLOW) == 1) {
                return COLORS.PURPLE;
            }

        } else if (surroundingColorPixelCount == 1) {
            telemetry.addData("colors1", colors.toString());
            if (colors.containsKey(COLORS.PURPLE) && colors.get(COLORS.PURPLE) == 1) {
                if (getRemainingPixels().get(COLORS.PURPLE) >2) {
                    return COLORS.PURPLE;
                } else {
                    return COLORS.GREEN;
                }
            }
            if (colors.containsKey(COLORS.GREEN) && colors.get(COLORS.GREEN) == 1) {
                if (getRemainingPixels().get(COLORS.GREEN) >2) {
                    return COLORS.GREEN;
                } else {
                    return COLORS.YELLOW;
                }
            }
            if (colors.containsKey(COLORS.YELLOW) && colors.get(COLORS.YELLOW) == 1) {
                if (getRemainingPixels().get(COLORS.YELLOW) >2) {
                    return COLORS.YELLOW;
                } else {
                    return COLORS.PURPLE;
                }
            }
        }
        return COLORS.WHITE;
    }

    public boolean getClawPos(Pixel pixel, Pixel pixel1) {
        TreeMap<Double, Pixel> pixelRow = getPixels().getOrDefault(pixel.getRawRow(), new TreeMap<>());
        TreeMap<Double, Pixel> pixel1Row = getPixels().getOrDefault(pixel1.getRawRow(), new TreeMap<>());
        int pixelpriority = 0; // > 1 is left < 1 is right
        int pixel1priority = 0;
        if (pixel.getRawRow() > pixel1.getRawRow()) {
            if (pixelRow.containsKey(pixel.getRawCol()-1) && pixelRow.containsKey(pixel.getRawCol()+1)) {
                return false;
            } else if (pixelRow.containsKey(pixel.getRawCol()-1)) {
                return false;
            } else if (pixelRow.containsKey(pixel.getRawCol()+1)) {
                return true;
            }
        } else if  (pixel.getRawRow() < pixel1.getRawRow()) {
            if (pixelRow.containsKey(pixel.getRawCol()-1) && pixelRow.containsKey(pixel.getRawCol()+1)) {
                return false;
            } else if (pixel1Row.containsKey(pixel1.getRawCol()-1)) {
                return true;
            } else if (pixel1Row.containsKey(pixel1.getRawCol()+1)) {
                return false;
            }
        } else {
            if (pixel.getRawCol() < pixel1.getRawCol()) {
                return false;
            } else if (pixel.getRawCol() < pixel1.getRawCol()) {
                return true;
            }
        }
        return false;

    }

    public boolean getAutoPixelLoc() {
//        TreeMap<Double, Pixel> bottom_row = getPixels().get(getPixels().lastKey());
//        telemetry.addData("Bottom Row Size: ", bottom_row.size());
//        if (bottom_row.size() == 1) {
//            return bottom_row.lastKey() % 2 == 0;
//        }
        return isRight;
    }

    public ArrayList<ArrayList<COLORS>> getMosaics() {
        ArrayList<ArrayList<COLORS>> mosaic = new ArrayList<>();
        ArrayList<Pixel> pixels_used = new ArrayList<>();
        for (Pixel p : pixels) {
            if (pixels_used.contains(p)) {
                continue;
            }
            ArrayList<Pixel> surrounding = getSurrounding(p.getRawRow(), p.getRawCol());
            Map<COLORS, Double> colors = getSurroundingColorCount(surrounding);
            double surroundingColorPixelCount = 6-colors.getOrDefault(COLORS.WHITE, 0.0)-colors.getOrDefault(COLORS.NONE, 0.0);
            for (Pixel pixel : surrounding) {
                if (pixel.color != COLORS.WHITE && pixel.color != COLORS.NONE) {
                    telemetry.addData("pixel", getSurroundingColorCount(getSurrounding(pixel.getRawRow(), pixel.getRawCol())));
                    Map<COLORS, Double> scolors = getSurroundingColorCount(getSurrounding(pixel.getRawRow(), pixel.getRawCol()));
                    double pnw =6-scolors.getOrDefault(COLORS.WHITE, 0.0)-scolors.getOrDefault(COLORS.NONE, 0.0);
                    //    if (pnw != surroundingColorPixelCount-1) return COLORS.WHITE;
                }
            }
            if (colors.containsKey(COLORS.PURPLE) && colors.get(COLORS.PURPLE) == 2 && p.color == COLORS.PURPLE) {
                mosaic.add(new ArrayList<>(Arrays.asList(COLORS.PURPLE, COLORS.PURPLE, COLORS.PURPLE)));
                for (Pixel up: getSurrounding(p.getRawRow(), p.getRawCol())) {
                    if (up.color != COLORS.WHITE && up.color != COLORS.NONE) {
                        pixels_used.add(up);
                    }
                }
            } else if (colors.containsKey(COLORS.GREEN) && colors.get(COLORS.GREEN) == 2 && p.color == COLORS.GREEN) {
                mosaic.add(new ArrayList<>(Arrays.asList(COLORS.GREEN, COLORS.GREEN, COLORS.GREEN)));
                for (Pixel up: getSurrounding(p.getRawRow(), p.getRawCol())) {
                    if (up.color != COLORS.WHITE && up.color != COLORS.NONE) {
                        pixels_used.add(up);
                    }
                }
            } else if (colors.containsKey(COLORS.YELLOW) && colors.get(COLORS.YELLOW) == 2 && p.color == COLORS.YELLOW) {
                mosaic.add(new ArrayList<>(Arrays.asList(COLORS.GREEN, COLORS.GREEN, COLORS.GREEN)));
                for (Pixel up: getSurrounding(p.getRawRow(), p.getRawCol())) {
                    if (up.color != COLORS.WHITE && up.color != COLORS.NONE) {
                        pixels_used.add(up);
                    }
                }
            } else if ((colors.containsKey(COLORS.PURPLE) && colors.get(COLORS.PURPLE) == 1) && (colors.containsKey(COLORS.YELLOW) && colors.get(COLORS.YELLOW) == 1) &&  p.color == COLORS.GREEN ) {
                mosaic.add(new ArrayList<>(Arrays.asList(COLORS.PURPLE, COLORS.YELLOW, COLORS.GREEN)));
                for (Pixel up: getSurrounding(p.getRawRow(), p.getRawCol())) {
                    if (up.color != COLORS.WHITE && up.color != COLORS.NONE) {
                        pixels_used.add(up);
                    }
                }
            }
        }
        return mosaic;
    }

    public boolean getSetlineByPixel(double row, double col) {
        double bottom_row = getPixels().lastKey()+1;
        double top_row = getPixels().firstKey();
        top_row = bottom_row - top_row;
        row = bottom_row - row;
        return row/3 > top_row/3;
    }

    public int getSetlines() {
        double bottom_row = getPixels().lastKey()+1;
        telemetry.addData("bottom keys", getPixels().keySet().toString());

        double top_row = getPixels().firstKey();
        top_row = bottom_row - top_row;
        telemetry.addData("top row", top_row);
        return (int) Math.floor(top_row/3);
    }

    public int getScore() {
        int score = 0;
        score += getMosaics().size()*10;
        score += getSetlines()*10;
        for (Pixel pixel : pixels) {
            score += 3;
        }
        return score;
    }

    public Map<COLORS, Double> getSurroundingColorCount(ArrayList<Pixel> surrounding) {
        Map<COLORS, Double> colors = new HashMap<>();
        for (Pixel pixel : surrounding) {
            if (colors.containsKey(pixel.color)) {
                colors.put(pixel.color, colors.get(pixel.color)+1);
            } else {
                colors.put(pixel.color, 1.0);
            }
        }
        return colors;
    }


    public Map<COLORS, Double> getRemainingPixels() {
        double purple = 5;
        double green = 5;
        double yellow = 5;
        for (Pixel pixel : pixels) {
            switch (pixel.color) {
                case PURPLE:
                    if (purple != 0) {
                        purple -= 1;
                    }
                    break;
                case GREEN:
                    if (green != 0) {
                        green -= 1;
                    }
                    break;
                case YELLOW:
                    if (yellow != 0) {
                        yellow -= 1;
                    }
                    break;
            }
        }
        Map<COLORS, Double> colors = new HashMap<>();
        colors.put(COLORS.PURPLE, purple);
        colors.put(COLORS.GREEN, green);
        colors.put(COLORS.YELLOW, yellow);
        return colors;
    }
    private static void drawHexagon(Mat img, double centerX, double centerY, int radius, Scalar color) {
        List<MatOfPoint> hexagons = new ArrayList<>();

        Point[] hexagon = new Point[6];

        for (int i = 0; i < 6; i++) {
            double angle = 2.0 * Math.PI / 6 * i;
            int x = (int) (centerX + radius * Math.sin(angle));
            int y = (int) (centerY + radius * Math.cos(angle));
            hexagon[i] = new Point(x, y);
        }

        hexagons.add(new MatOfPoint(hexagon));
        Imgproc.polylines(img, hexagons, true, color, 2);
    }
    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}