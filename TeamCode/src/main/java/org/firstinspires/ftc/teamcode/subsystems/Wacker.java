package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wacker {

    Servo wackerServo;

    Telemetry telemetry;

    public enum WackerStates {
        SECOND (.97),
        TOP (.915),
        UP(0);

        double pos;

        WackerStates(double pos) {
            this.pos = pos;
        }
    }

    public Wacker(HardwareMap hardwareMap, Telemetry telemetry) {
        wackerServo = hardwareMap.servo.get("wacker");
        this.telemetry = telemetry;
    }

    public void setWackerState(WackerStates state) {
        wackerServo.setPosition(state.pos);
    }

    public Action wackerAction(WackerStates state) {
        return telemetryPacket -> {
            setWackerState(state);
            return false;
        };
    }
}
