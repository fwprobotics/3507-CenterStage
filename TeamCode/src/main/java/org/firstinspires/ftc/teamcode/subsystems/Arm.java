package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {

    public static double intake = 0;
    public static double drop = 1;

    public enum ArmState {
        INTAKE (intake),
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
    }

    public void setState(ArmState state) {
        this.currentState = state;
        servo.setPosition(state.armPos);
    }
}
