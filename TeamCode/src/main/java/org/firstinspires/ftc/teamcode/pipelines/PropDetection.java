package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Moments;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;

public class PropDetection implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public enum PropLocation {
        LEFT (0),
        RIGHT (13),
        CENTER(6),

        NONE(0);

        public int offset;
        PropLocation(int off) {
            offset = off;
        }
    }
    public static Scalar blue_lower = new Scalar(0, 130, 0);
    public static Scalar blue_upper = new Scalar(166, 137, 112);

    public static Scalar red_lower = new Scalar(0, 159, 149);
    public static Scalar red_upper = new Scalar(255, 255, 255);

    private Mat mat;
    private Mat thresh;
    private Mat heirarchy;
    private Mat resized;

    //  private int width;
    PropLocation location;
    private Telemetry telemetry;
    private Robot.AutoZoneColor autoZoneColor;
    public PropDetection(Robot.AutoZoneColor autoZoneColor, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.autoZoneColor = autoZoneColor;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Core.rotate(input, input, Core.ROTATE_180);
        mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2Lab);
        telemetry.addData("res", input.width()+input.height());
        Rect crop = new Rect(0, input.height()/3, input.width(), 2*input.height()/3);
        mat = new Mat(mat, crop);


        thresh = new Mat();
        if (autoZoneColor == Robot.AutoZoneColor.RED) {
            Core.inRange(mat, red_lower, red_upper, thresh);
        } else {
            Core.inRange(mat, blue_lower, blue_upper, thresh);
        }
        List<MatOfPoint> contours = new ArrayList();

        heirarchy = new Mat();
        Imgproc.findContours(thresh, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        double maxVal = 0;
        int maxValIdx = 0;
        if (contours.size() == 0) {
            location = PropLocation.NONE;
            telemetry.addData("Location", location);
            telemetry.update();
            resized = new Mat();

            Imgproc.resize(mat, resized, new Size(mat.width() / 4, mat.height() / 4));
            Imgproc.putText(
                    resized,
                    location.toString(),
                    new Point(50, 75),
                    Imgproc.FONT_HERSHEY_PLAIN,
                    5,
                    new Scalar(255, 255, 255),
                    5
            );
            return new Point(0, 0);
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

        double center = m.m10/m.m00;
        int width = input.width();
        if(!(autoZoneColor == Robot.AutoZoneColor.RED)) {
            if (center <= width/4) {
                location = PropLocation.LEFT;
            } else if (center <= (width/8)*5) {
                location = PropLocation.CENTER;
            } else {
                location = PropLocation.RIGHT;
            }
        } else {
            if (center <= (width/4)) {
                location = PropLocation.LEFT;
            } else if (center <= (width/8)*5) {
                location = PropLocation.CENTER;
            } else {
                location = PropLocation.RIGHT;
            }
        }

        telemetry.addData("Location", location);
        telemetry.update();
        Imgproc.circle(mat, new Point(m.m10/m.m00, m.m01/m.m00), 20, new Scalar(255, 0, 0), 40);
        Imgproc.line(mat, new Point((width/8)*5, 0), new Point((width/8)*5, 853), new Scalar(255, 0, 0), 5);
        Imgproc.line(mat, new Point((width/4), 0), new Point((width/4), 853), new Scalar(255, 0, 0), 5);

        //  Mat resized = new Mat();

        //  Imgproc.resize(mat, resized, new Size(mat.width()/4, mat.height()/4));
        Imgproc.putText(
                mat,
                location.toString(),
                new Point(50, 75),
                Imgproc.FONT_HERSHEY_PLAIN,
                5,
                new Scalar(255, 255, 255),
                5
        );
        return new Point(m.m10/m.m00, m.m01/m.m00);

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        float x = (float)((Point)userContext).x*scaleBmpPxToCanvasPx;
        float y = (float) ((float)((Point)userContext).y*scaleBmpPxToCanvasPx + onscreenHeight/2.0);
        canvas.rotate(180);
        canvas.drawCircle(x, y, 10, new Paint());
        canvas.drawLine((float) (onscreenWidth/8.0*5.0), 0, onscreenWidth/8*5, onscreenHeight, new Paint());
        canvas.drawLine((float) (onscreenWidth/4.0), 0, onscreenWidth/4, onscreenHeight, new Paint());
    }



    public PropLocation getLocation() {
        return location;
    }

}
