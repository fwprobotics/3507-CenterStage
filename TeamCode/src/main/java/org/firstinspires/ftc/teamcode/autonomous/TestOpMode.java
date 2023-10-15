package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.PropDetection;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class TestOpMode extends LinearOpMode {
    PropDetection pipeline;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        initCV();
        MecanumDrive drive =  new MecanumDrive(hardwareMap, new Pose2d(14, -64, Math.toRadians(90)));
        Lift lift = new Lift(hardwareMap, telemetry);
        Arm arm = new Arm(hardwareMap, telemetry);
        Carousel carousel = new Carousel(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Robot robot = new Robot(Robot.AutoZoneColor.RED, Robot.AutoZoneHalf.RIGHT, drive, arm, carousel, lift, intake);
        robot.arm.setState(Arm.ArmState.DRIVE);
        waitForStart();
        PropDetection.PropLocation location = readProp();
        Action autoAction = robot.createFieldActionSequence(new Pose2d(16, -64, Math.toRadians(90)))
                .dropPurplePixel(location)
                .dropYellowPixel(location)
                .park()
                .build();

        Actions.runBlocking(
                        autoAction
                );




    }

    private void initCV() {
        // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();

        visionPortalBuilder.setCamera(webcamName);
        pipeline = new PropDetection(Robot.AutoZoneColor.RED, telemetry);
        visionPortalBuilder.addProcessor(pipeline);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setAutoStopLiveView(true);
        visionPortal = visionPortalBuilder.build();
        visionPortal.setProcessorEnabled(pipeline, true);

        // Combines the above to create a webcam that we will use
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        //Sets our pipeline to view images through as the one we want
//        //(Boundary between regions 1 and 2, Boundary between 2 and 3, Far left, Far top, Far right, Far bottom, opmode, the side we're on)

//        webcam.setPipeline(pipeline);
//
//        // Turns on the webcam
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
//            }
//            //320 240
//
//            //This is needed so it knows what to do if something goes wrong
//            public void onError(int thing) {
//                telemetry.addData("error", thing);
//            }
//
//        });

    }

    public PropDetection.PropLocation readProp() {
        PropDetection.PropLocation location = pipeline.getLocation();
        visionPortal.setProcessorEnabled(pipeline, false);
        return location;
    }
}
