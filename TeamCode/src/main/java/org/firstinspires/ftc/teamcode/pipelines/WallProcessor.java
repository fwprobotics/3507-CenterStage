package org.firstinspires.ftc.teamcode.pipelines;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor.COLORS;
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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class WallProcessor extends OpenCvPipeline
{
    // STATIC CONSTANTS

    public static Scalar blue = new Scalar(7,197,235,255);
    public static Scalar red = new Scalar(255,0,0,255);
    public static Scalar green = new Scalar(0,255,0,255);
    public static Scalar white = new Scalar(255,255,255,255);

    public static Scalar green_lower = new Scalar(59.5, 38.3, 14.2);
    public static Scalar green_upper = new Scalar(195.5, 109.1, 167.2);
    public static Scalar white_lower = new Scalar(129, 113, 0);
    public static Scalar white_upper = new Scalar(255, 255, 255);
    public static Scalar yellow_lower = new Scalar(69.4, 0, 175.7);
    public static Scalar yellow_upper = new Scalar(255, 168.6, 255.0);
    public static Scalar purple_lower = new Scalar(0, 0, 0);
    public static Scalar purple_upper = new Scalar(255, 255, 120.4);

    public enum COLORS {
        PURPLE (purple_lower, purple_upper),
        YELLOW (yellow_lower, yellow_upper),
        WHITE (white_lower, white_upper);

        public Scalar lower;
        public Scalar upper;

        COLORS(Scalar lower, Scalar upper) {
            this.lower = lower;
            this.upper = upper;
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

    double tagsizeX = TAG_SIZE;
    double tagsizeY = TAG_SIZE;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();


    Telemetry telemetry;

    public WallProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
        constructMatrix();
    }

    @Override
    public void init(Mat frame)
    {
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize()
    {
        // Delete the native context we created in the init() function
        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert to greyscale

        //  input = getPixelsByColor(input, COLORS.WHITE);
        List<MatOfPoint> contours = getPixelsByColor(input, COLORS.WHITE);

        input = labelPixels(input, contours);
        //  labelPixels(input, purple_contours, COLORS.PURPLE);
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
            AprilTagDetection d = detections.get(0);
            Imgproc.circle(input, d.corners[3], 5, new Scalar(255, 0, 0, 255), -1);
            Imgproc.circle(input, d.corners[1], 5, new Scalar(0, 255, 0, 255), -1);
            telemetry.addLine("corner 3: " + d.corners[3].toString());
            telemetry.addLine("corner 1: " + d.corners[1].toString());

            Rect crop = new Rect((int) d.corners[3].x, (int)d.corners[3].y, (int) ((int) d.corners[1].x-d.corners[3].x), (int) ((int)d.corners[1].y-d.corners[3].y));
            input = new Mat(input, crop);
        }
        for(AprilTagDetection detection : detections)
        {

//            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
//            drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
//            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
//
            Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));

            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
        }

        telemetry.update();

        return input;
    }
    public List<MatOfPoint> getPixelsByColor(Mat input, COLORS color) {

        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);


        Mat mat = new Mat();
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

            if (numVertices == 6 && (rect.width < 25 && rect.width > 10)) {
                COLORS color = getPixelColor(input, approxContour);
                //  Draw the approximate polygon
                List<MatOfPoint> drawContours = new ArrayList<>();
                drawContours.add(approxContour);
                Imgproc.drawContours(input, drawContours, 0, new Scalar(0, 255, 0), 2);
                Point point = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
                Imgproc.circle(input, point, 5, new Scalar(255, 0, 0, 255), -1);
                Imgproc.putText(input, color.toString(), new Point(points[0].x, points[0].y), Imgproc.FONT_HERSHEY_COMPLEX, 0.25, new Scalar(0, 0, 255));
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