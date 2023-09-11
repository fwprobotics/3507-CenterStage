package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    public static double liftPower = 0.05;
    enum LiftState {
        UP (100),
        DOWN (0);

        int height;
        LiftState(int height) {
            this.height = height;
        }
    }

    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor liftMotor;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.liftMotor = hardwareMap.dcMotor.get("liftMotor");
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeight(int height) {
        liftMotor.setTargetPosition(height);
        liftMotor.setPower(liftPower);
    }

    public Action setUp(LiftState state) {
        setHeight(state.height);
        return telemetryPacket -> liftMotor.isBusy();
    }


}
