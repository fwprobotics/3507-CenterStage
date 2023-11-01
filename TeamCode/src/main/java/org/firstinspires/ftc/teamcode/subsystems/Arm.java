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

    public static double intake = .9;
    public static double drive = 0.5;
    public static  double limbo = 0.8;
    public static double drop = 0.25;//.3;//0.25;

    public enum ArmState {
        INTAKE (intake),
        LIMBO (limbo),

        DRIVE(drive),
        DROP (drop);

        public double armPos;
        ArmState(double armPos) {
            this.armPos = armPos;
        }
    }

    public ArmState currentState =ArmState.INTAKE;

    ServoImplEx servo;
    Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(ServoImplEx.class, "arm");
        this.telemetry = telemetry;
        setState(ArmState.INTAKE);
    }

    public void setState(ArmState state) {
        this.currentState = state;
        servo.setPosition(state.armPos);
    }

    public void turnOff() {
        servo.setPwmDisable();
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


}
