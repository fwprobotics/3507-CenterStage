package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

@Config
public class Lift {
    @Config
    public static class LiftConfig {
        public static double liftPower = 0.5;
        public static double kp = -0.001;
        public static double ki = 0;
        public static double kd =   0;

        public static double manualGain = 0.1;
    }

    public enum LiftState {
        UP (-750),
        MID (-450),
        LOW(-150),
        DOWN (0);

        public int height;
        LiftState(int height) {
            this.height = height;
        }
    }

    public static PIDController pid = new PIDController(LiftConfig.kp, LiftConfig.ki, LiftConfig.kd);
    public static double desiredPos = 0;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor liftMotor;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pid.setSetPoint(0);
        pid.setTolerance(10);

//        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setHeight(int height) {
        liftMotor.setTargetPosition(height);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LiftConfig.liftPower);
        this.desiredPos = height;
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
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          this.desiredPos += -invertedJoystick*LiftConfig.manualGain;
          liftMotor.setTargetPosition((int)Math.floor(this.desiredPos));
     //   } else {
           // liftMotor.setPower(0);
       // }


    }

    public void teleOpControl(Gamepad gamepad2) {
        if (gamepad2.dpad_down) {
            setHeight(LiftState.DOWN.height);
        }
        else if (gamepad2.dpad_left) {
            setHeight(LiftState.LOW.height);
        }
        else if (gamepad2.dpad_up) {
            setHeight(LiftState.MID.height);
        }
       else if (gamepad2.dpad_right) {
            setHeight(LiftState.UP.height);
        }
       if (-gamepad2.right_stick_y != 0) {
           manualControl(-gamepad2.right_stick_y);
       }
       telemetry.addData("currentHeight", liftMotor.getCurrentPosition());
       telemetry.addData("Desired Height (ticks)", liftMotor.getTargetPosition());
       telemetry.addData("Busy?", !pid.atSetPoint());

    }

    public void update() {
//        double output = pid.calculate(liftMotor.getCurrentPosition());
//        telemetry.addData("power", output);
//        liftMotor.setPower(output);
    }


}
