package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public static double intakeSpeed = 0.8;

    DcMotor intakeMotor;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }

    public void manualControl(double rightTrigger, double leftTrigger) {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (rightTrigger > 0 ) {
            intakeMotor.setPower(rightTrigger * intakeSpeed);
        } else {
            intakeMotor.setPower(-leftTrigger*intakeSpeed*0.5);
        }
    }

    public Action intakeRunAction(int pos) {
        return telemetryPacket -> {
            intakeMotor.setTargetPosition(pos);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(0.1);
            return false;
        };
    }

    public Action intakeAction(int multiplier) {
        return telemetryPacket -> {
            intakeMotor.setPower(intakeSpeed*multiplier);
            return false;
        };

    }

    public void setState(boolean on) {
        intakeMotor.setPower(intakeSpeed);
    }
}
