package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PropDetection;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class Camera {

    WebcamName webcamName;
    Telemetry telemetry;

    PropDetection pipeline;
    WallProcessor wallPipeline;
    boolean partnerPixel;
    VisionPortal visionPortal;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

    }

    public void initDetection(Robot.AutoZoneColor color) {
        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(webcamName);
        pipeline = new PropDetection(color, telemetry);
        visionPortalBuilder.addProcessor(pipeline);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setAutoStopLiveView(true);
        visionPortalBuilder.setCameraResolution(new Size(640, 480));
        visionPortal = visionPortalBuilder.build();
        visionPortal.setProcessorEnabled(pipeline, true);
        telemetry.addData("set camera", "true");
        telemetry.update();
    }

    public PropDetection.PropLocation readProp() {
        PropDetection.PropLocation location = pipeline.getLocation();
        visionPortal.setProcessorEnabled(pipeline, false);
        visionPortal.close();
        return location;
    }

    public Action scanWall() {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
                    visionPortalBuilder.setCamera(webcamName);
                    wallPipeline = new WallProcessor(telemetry);
                    visionPortalBuilder.addProcessor(pipeline);
                    visionPortalBuilder.enableLiveView(true);
                    visionPortalBuilder.setAutoStopLiveView(true);
                    visionPortalBuilder.setCameraResolution(new Size(640, 480));
                    visionPortal = visionPortalBuilder.build();
                    init = true;
                }
                if (wallPipeline.waiting) {
                    return true;
                }
                partnerPixel = wallPipeline.getAutoPixelLoc();
                return false;

            }
        };
    }

    public boolean pixelSlotMatches(Claw.Claws claw) {
        return (partnerPixel && claw == Claw.Claws.LEFT) || (!partnerPixel && claw == Claw.Claws.RIGHT);
    }
}
