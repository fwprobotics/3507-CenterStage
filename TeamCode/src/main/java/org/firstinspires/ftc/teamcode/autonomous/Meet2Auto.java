package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.AutoZoneColor.RED;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.PropDetection;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Flippers;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Wacker;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.prefs.AbstractPreferences;

@Autonomous
public class Meet2Auto extends LinearOpMode {
    PropDetection pipeline;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.AutoZoneColor startColor = RED;
        Robot.AutoZoneHalf startHalf = Robot.AutoZoneHalf.NEAR;
        Pose2d startPose = new Pose2d(10, -64, Math.toRadians(-90));
        while (!gamepad1.a) {
            if (gamepad1.dpad_up) {
                startColor = RED;
                startHalf = Robot.AutoZoneHalf.NEAR;
                startPose = new Pose2d(10, -64, Math.toRadians(-90));
            } else if (gamepad1.dpad_down) {
                startColor = RED;
                startHalf = Robot.AutoZoneHalf.FAR;
                startPose = new Pose2d(-16-24, -64, Math.toRadians(-90));
            } else if (gamepad1.dpad_left) {
                startColor = Robot.AutoZoneColor.BLUE;
                startHalf = Robot.AutoZoneHalf.FAR;
                startPose = new Pose2d(-10-24, 64, Math.toRadians(90));
            } else if (gamepad1.dpad_right) {
                startColor = Robot.AutoZoneColor.BLUE;
                startHalf = Robot.AutoZoneHalf.NEAR;
                startPose = new Pose2d(15, 62, Math.toRadians(90));
            }
            telemetry.addData("autoPos", startColor+" "+startHalf);
            telemetry.update();
        }
        initCV(startColor);
        //10, -64
        MecanumDrive drive =  new MecanumDrive(hardwareMap, startPose);
        Lift lift = new Lift(hardwareMap, telemetry);
        Arm arm = new Arm(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Claw claw = new Claw(hardwareMap, telemetry);
        Flippers flippers = new Flippers(hardwareMap, telemetry);
        Lights lights = new Lights(hardwareMap, telemetry);
        Robot robot = new Robot(startColor, startHalf, drive, arm, claw, lift, intake, flippers, lights);
        waitForStart();
        PropDetection.PropLocation location = readProp();
      //  robot.carousel.setAutoStart(location);
     //   robot.arm.setState(Arm.ArmState.DRIVE);
        claw.setClawPosition(Claw.ClawPos.CLOSED, startColor == RED ? Claw.Claws.RIGHT: Claw.Claws.LEFT);
        Action autoAction = robot.createFieldActionSequence(startPose)
                .dropPurplePixel(location)
                .dropYellowPixel(location)
                .toStack(0)
                .toBackDrop(0)
                .park(location)
                .build();

        Actions.runBlocking(
                new ParallelAction(
                        autoAction,
                        arm.wristUpdateAction()
                ));




    }

    private void initCV(Robot.AutoZoneColor color) {
        // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(webcamName);
        pipeline = new PropDetection(color, telemetry);
        visionPortalBuilder.addProcessor(pipeline);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setAutoStopLiveView(true);
        visionPortalBuilder.setCameraResolution(new Size(640, 480));
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
