package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Airplane {
    @Config
            public static class AirplaneConfig {
        public static double speed = 1;
    }

    DcMotor launcher;

    public Airplane(HardwareMap hardwareMap, Telemetry telemetry) {
        launcher = hardwareMap.dcMotor.get("airplane");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        launcher.setPower(power);
    }

    public void setOn(boolean on) {
        setPower(on? AirplaneConfig.speed : 0);
    }
}
