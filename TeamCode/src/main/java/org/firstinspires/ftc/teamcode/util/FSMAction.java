package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
public interface FSMAction extends Action {

    boolean stateMachine();

    @Override
    default boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return stateMachine();
    }
}
