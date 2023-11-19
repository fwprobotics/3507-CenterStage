package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {

    public static double intake = 1;
    public static double wristIntake = .0;
    public static double drive = 0.5;
    public static  double limbo = 0.8;
    public static double drop = 0.45;//.3;//0.25;
    public static double wristDrop = .95;

    public enum ArmState {
        INTAKE (intake, wristIntake),
        LIMBO (limbo, 0),

        DRIVE(drive, 0),
        DROP (drop, wristDrop);

        public double armPos;
        public double wristPos;

        ArmState(double armPos, double wristPos) {

            this.armPos = armPos;
            this.wristPos = wristPos;
        }
    }

    public boolean isbusy () {
       boolean isbusy =  (servo.getPosition() >= .75 ||  servo.getPosition() <= .3);
       return  isbusy;
    }
    public ArmState currentState =ArmState.INTAKE;
    public ArmState currentWristState = ArmState.INTAKE;

    ServoImplEx servo;

    ServoImplEx wristServo;
    Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(ServoImplEx.class, "arm");
        wristServo = hardwareMap.get(ServoImplEx.class, "wrist");
        this.telemetry = telemetry;
        setState(ArmState.INTAKE);
    }

    public void setState(ArmState state) {
        this.currentState = state;
        servo.setPosition(state.armPos);
    }

    public void setWristState(ArmState state) {
        this.currentWristState = state;
        wristServo.setPosition(state.wristPos);
    }


    public class SetArmState implements Action {

        ArmState state;

        public SetArmState(ArmState state) {
            this.state = state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setState(this.state);
            return false;
        }
    }

    public Action armStateAction(ArmState state) {
        return new SetArmState(state);
    }
    public Action wristStateAction(ArmState state) {
        return telemetryPacket -> {
            setWristState(state);
            return false;
        };
    }


}
