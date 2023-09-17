package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Carousel {

    public static double slot1Intake = 30/210;
    public static double slot1Output = 1;
    public static double slot2Intake = 180/210;
    public static double slot2Output = 0;

    public static double lock = 105/210;

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
        if (intake) {
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
    }
