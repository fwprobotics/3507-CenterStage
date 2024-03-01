package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
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
        UP (1),
        DOWN (-1),
        OFF(0);

        public int pos;

        WinchStates(int pos) {
            this.pos = pos;
        }
    }

    CRServo hookLeft;
    CRServo hookRight;
    public boolean active = false;

    Telemetry telemetry;

    public Climb(HardwareMap hardwareMap, Telemetry telemetry) {
        hookLeft = hardwareMap.crservo.get("climbHookLeft");
        hookRight = hardwareMap.crservo.get("climbHookRight");
        //   winch.setTargetPosition(0);
        this.telemetry = telemetry;
    }

//    public void setHookState(HookStates state) {
//        hook.setPosition(state.pos);
//    }

    public void setWinchState(WinchStates winchState) {
        hookLeft.setPower(winchState.pos*1);
        hookRight.setPower(winchState.pos*1);
        if (winchState != WinchStates.OFF) {
            active = true;
        } else {
            active = false;
        }
    }

    public void setWinchPower(double power) {
        telemetry.addData("winch power", power);
    }
}
