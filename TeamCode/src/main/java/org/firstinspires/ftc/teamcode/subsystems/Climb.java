package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climb {
    @Config
    public static class ClimbConfig {
        public static double winchSpeed = 0.6;
    }

    public enum HookStates {
        DOWN (0.07),
        UP (0.5), //1
        DROP (0.37);

        public double pos;

        HookStates(double pos) {
            this.pos = pos;
        }
    }

    public enum WinchStates {
        LIFT (500),
        RESET (0);

        public int pos;

        WinchStates(int pos) {
            this.pos = pos;
        }
    }

    Servo hook;
    DcMotor winch;

    Telemetry telemetry;

    public Climb(HardwareMap hardwareMap, Telemetry telemetry) {
        hook = hardwareMap.servo.get("climbHook");
        winch = hardwareMap.dcMotor.get("climbWinch");
        //   winch.setTargetPosition(0);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.telemetry = telemetry;
    }

    public void setHookState(HookStates state) {
        hook.setPosition(state.pos);
    }

    public void setWinchState(WinchStates state) {
        winch.setTargetPosition(state.pos);
        winch.setPower(ClimbConfig.winchSpeed);
    }

    public void setWinchPower(double power) {
        telemetry.addData("winch power", power);
        winch.setPower(power);
    }
}
