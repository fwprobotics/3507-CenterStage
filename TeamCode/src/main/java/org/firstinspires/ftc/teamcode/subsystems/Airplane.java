package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Airplane {
    @Config
            public static class AirplaneConfig {
        public static double rest = 1;
        public static double fire = 0;
    }

    public enum AirplaneStates {
        REST(AirplaneConfig.rest),
        FIRE (AirplaneConfig.fire);

        double pos;
        AirplaneStates(double pos) {
            this.pos = pos;
        }
    }

    Servo launcher;

    public Airplane(HardwareMap hardwareMap, Telemetry telemetry) {
        launcher = hardwareMap.servo.get("airplane");
    }

    public void setAirplaneState(AirplaneStates state) {
        launcher.setPosition(state.pos);
    }

}
