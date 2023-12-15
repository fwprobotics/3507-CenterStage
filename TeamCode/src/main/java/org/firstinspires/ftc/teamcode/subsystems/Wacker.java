package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wacker {

    Servo wackerServo;

    Telemetry telemetry;

    public Wacker(HardwareMap hardwareMap, Telemetry telemetry) {
        wackerServo = hardwareMap.servo.get("wacker");
        this.telemetry = telemetry;
    }

    public void setWackerState(boolean up) {
        if (up) {
            wackerServo.setPosition(0.33);
        } else {
            wackerServo.setPosition(0.97);
        }
    }

    public Action wackerAction(boolean up) {
        return telemetryPacket -> {
            setWackerState(up);
            return false;
        };
    }
}
