package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.WallProcessor;

public class Flippers {

    public enum FlipperState {
        OPEN (0, 1),
        CLOSED (1, 0);

        public double leftPos;
        public double rightPos;

        FlipperState (double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

    Telemetry telemetry;

    Servo flipperRight;
    Servo flipperLeft;
    public Flippers(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        flipperRight = hardwareMap.get(Servo.class, "flipperRight");
        flipperLeft = hardwareMap.get(Servo.class, "flipperLeft");
    }

    public void setFlipperPosition(FlipperState state) {
        flipperRight.setPosition(state.rightPos);
        flipperLeft.setPosition(state.leftPos);
    }

    public Action flipperAction(FlipperState state) {
        return  telemetryPacket -> {
            setFlipperPosition(state);
            return false;
        };
    }

}
