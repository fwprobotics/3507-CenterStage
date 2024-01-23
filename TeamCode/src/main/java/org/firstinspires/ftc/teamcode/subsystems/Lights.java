package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lights {

    public enum LightStates {
        DEFAULT (RevBlinkinLedDriver.BlinkinPattern.BLUE),

        PURPLE (RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET),

        YELLOW (RevBlinkinLedDriver.BlinkinPattern.YELLOW),

        CYCLE (RevBlinkinLedDriver.BlinkinPattern.CONFETTI),

        PICKUP (RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD),
        RAINBOW (RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);

        RevBlinkinLedDriver.BlinkinPattern pattern;

        LightStates(RevBlinkinLedDriver.BlinkinPattern pattern) {
            this.pattern = pattern;
        }
    }

    Telemetry telemetry;
    RevBlinkinLedDriver blinkinLedDriver;


    public Lights(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        setState(LightStates.RAINBOW);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);
    }

    public void setState(LightStates state){
        setPattern(state.pattern);
    }

    public Action lightState(LightStates state) {
        return telemetryPacket -> {
            setState(state);
            return false;
        };
    }
}
