package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

@Config
public class Lift {
    @Config
    public static class LiftConfig {
        public static double liftPower = 0.2;
        public static double kp = 0.001;
        public static double ki = 0;
        public static double kd = 0;
    }

    public enum LiftState {
        UP (-900),
        DOWN (0);

        public int height;
        LiftState(int height) {
            this.height = height;
        }
    }

    public static PIDController pid = new PIDController(LiftConfig.kp, LiftConfig.ki, LiftConfig.kd);

    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor liftMotor;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.liftMotor = hardwareMap.dcMotor.get("liftMotor");
        pid.setSetPoint(0);
//        liftMotor.setTargetPosition(0);
//        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setHeight(int height) {
        pid.setSetPoint(height);
    }

    public Action liftStateAction(LiftState state) {
        return telemetryPacket -> {
            setHeight(state.height);
            return false;
        };
    }

    public Action updateLiftAction() {
        return telemetryPacket -> {
            update();
            return true;
        };
    }
    public void manualControl(double invertedJoystick) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //if (liftMotor.getCurrentPosition() > 0 && liftMotor.getCurrentPosition() < LiftState.UP.height) {
            liftMotor.setPower(invertedJoystick * LiftConfig.liftPower);
            pid.setSetPoint(liftMotor.getCurrentPosition());
            telemetry.addData("lift pos", liftMotor.getCurrentPosition());
            telemetry.update();
     //   } else {
           // liftMotor.setPower(0);
       // }


    }

    public void update() {
        double output = pid.calculate(liftMotor.getCurrentPosition());
        liftMotor.setPower(output);
    }


}
