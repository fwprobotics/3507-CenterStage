package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PropDetection;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class Camera {

    WebcamName webcamName;
    Telemetry telemetry;

    PropDetection pipeline;
    WallProcessor2 wallPipeline;
    boolean partnerPixel;
    VisionPortal visionPortal;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.telemetry = telemetry;

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
      //  visionPortal.close();
        return location;
    }

    public Action scanWall() {
        return new Action() {
            boolean init = false;
            ElapsedTime elapsedTime = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
                    visionPortalBuilder.setCamera(webcamName);
                    wallPipeline = new WallProcessor2(telemetry);
                    visionPortalBuilder.addProcessor(wallPipeline);
                    visionPortalBuilder.setLiveViewContainerId(0);
                    visionPortalBuilder.setAutoStopLiveView(true);
                    visionPortalBuilder.setCameraResolution(new Size(640, 480));
                    visionPortal = visionPortalBuilder.build();
                    elapsedTime.reset();
                    init = true;
                }
                if (wallPipeline.waiting) {
                    if (elapsedTime.seconds() < 6) {
                        return true;
                    } else {
                        partnerPixel = true;
                        visionPortal.setProcessorEnabled(wallPipeline, false);
                        return false;
                    }
                }
                partnerPixel = wallPipeline.getAutoPixelLoc();
                telemetry.log().add("detected: "+partnerPixel);
                visionPortal.setProcessorEnabled(wallPipeline, false);
                return false;

            }
        };
    }

    public boolean pixelSlotMatches(Claw.Claws claw) {
        telemetry.log().add("aligned: "+(!partnerPixel && claw == Claw.Claws.RIGHT));
        return (partnerPixel && claw == Claw.Claws.LEFT) || (!partnerPixel && claw == Claw.Claws.RIGHT);
    }
}
