package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lights {

    public enum LightStates {
        DEFAULT (RevBlinkinLedDriver.BlinkinPattern.BLUE),

        PURPLE (RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET),

        YELLOW (RevBlinkinLedDriver.BlinkinPattern.YELLOW),

        CYCLE (RevBlinkinLedDriver.BlinkinPattern.CONFETTI),

        PICKUP (RevBlinkinLedDriver.BlinkinPattern.BREATH_RED),

        RED (RevBlinkinLedDriver.BlinkinPattern.RED),
        RAINBOW (RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);

        RevBlinkinLedDriver.BlinkinPattern pattern;

        LightStates(RevBlinkinLedDriver.BlinkinPattern pattern) {
            this.pattern = pattern;
        }
    }

    Telemetry telemetry;
    RevBlinkinLedDriver blinkinLedDriver;
    DigitalChannel redLightRight;
    DigitalChannel redLightLeft;
    DigitalChannel greenLightRight;
    DigitalChannel greenLightLeft;



    public Lights(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        this.redLightRight = hardwareMap.get(DigitalChannel.class, "redLightRight");
        this.redLightLeft = hardwareMap.get(DigitalChannel.class, "redLightLeft");
        this.greenLightRight = hardwareMap.get(DigitalChannel.class, "greenLightRight");
        this.greenLightLeft = hardwareMap.get(DigitalChannel.class, "greenLightLeft");
        redLightRight.setMode(DigitalChannel.Mode.OUTPUT);
        redLightLeft.setMode(DigitalChannel.Mode.OUTPUT);
        greenLightRight.setMode(DigitalChannel.Mode.OUTPUT);
        greenLightRight.setMode(DigitalChannel.Mode.OUTPUT);
        setState(LightStates.RAINBOW);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);
    }

    public void setState(LightStates state){
        setPattern(state.pattern);
    }

    public void displayClawState(Claw claw) {
        if (claw.stateRight == Claw.ClawPos.CLOSED) {
            redLightRight.setState(true);
            greenLightRight.setState(false);
        } else {
            redLightRight.setState(false);
            greenLightRight.setState(true);
        }
        if (claw.stateLeft == Claw.ClawPos.CLOSED) {
            redLightLeft.setState(true);
            greenLightLeft.setState(false);
        } else {
            redLightLeft.setState(false);
            greenLightLeft.setState(true);
        }

        if (claw.stateLeft == Claw.ClawPos.CLOSED || claw.stateRight == Claw.ClawPos.CLOSED) {
            setState(LightStates.RED);
        }
    }

    public Action lightState(LightStates state) {
        return telemetryPacket -> {
            setState(state);
            return false;
        };
    }
}
