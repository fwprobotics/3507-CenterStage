package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.PropDetection;

@Config
public class Carousel {

    public static double slot1Intake = 0.7;
    public static double slot1Output = 0.05;
    public static double slot2Intake = 0.13;
    public static double slot2Output = 0.8;

    public static double lock = .35;

    ServoImplEx servo;
    Telemetry telemetry;


    public enum CarouselStates{
        SLOT1 (slot1Intake, slot1Output),
        SLOT2 (slot2Intake, slot2Output),
        LOCK(lock, lock)
        ;

        public double intakeState;
        public double outputState;

        CarouselStates(double intakeState, double outputState) {
            this.intakeState = intakeState;
            this.outputState = outputState;
        }
    }

    public CarouselStates currentState = CarouselStates.SLOT1;

    public Carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(ServoImplEx.class, "carousel");
        this.telemetry = telemetry;
    }

    public void setState(CarouselStates state, boolean intake) {
        currentState = state;
        if (intake) {
            telemetry.addData("state type", "input");
            servo.setPosition(state.intakeState);
        } else {
            servo.setPosition(state.outputState);
        }
    }

    public void dropBoth() {
        servo.setPosition(CarouselStates.SLOT1.outputState);
        servo.setPosition(CarouselStates.SLOT2.outputState);
    }

    public void nextState(boolean intake) {
        switch (currentState) {
            case LOCK:
                currentState = CarouselStates.SLOT1;
                break;
            case SLOT1:
                currentState = CarouselStates.SLOT2;
                break;
            case SLOT2:
                currentState = CarouselStates.LOCK;
        }
        setState(currentState, intake);
    }

    public void setAutoStart(PropDetection.PropLocation location) {
        switch (location) {
            case LEFT:
                servo.setPosition(0.6);
                break;
            case CENTER:
                servo.setPosition(0.9);
                break;
            case RIGHT:
                servo.setPosition(1);
        }
    }

    public Action carouselStateAction(CarouselStates state, boolean intake) {
        return telemetryPacket -> {
            setState(state, intake);
            return false;
        };
    }
}
