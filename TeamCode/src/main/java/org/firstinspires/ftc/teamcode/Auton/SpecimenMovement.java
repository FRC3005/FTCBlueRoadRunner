package org.firstinspires.ftc.teamcode.Auton;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpecimenMovement {
    private DcMotorEx specimen;

    public SpecimenMovement(HardwareMap hardwareMap) {
        specimen = hardwareMap.get(DcMotorEx.class, "spec");
        specimen.setDirection(DcMotor.Direction.FORWARD); // Adjust if needed
        specimen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimen.setPower(1.0);
        specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Move specimen up to "upPosition"
    public class MoveUp implements Action {
        private final int upPosition;
        private boolean initialized = false;

        public MoveUp(int upPosition) {
            this.upPosition = upPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                specimen.setTargetPosition(upPosition);
                specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                specimen.setPower(1.0);
                initialized = true;
            }
            int curPos = specimen.getCurrentPosition();
            packet.put("Specimen Pos", curPos);
            return specimen.isBusy();
        }
    }

    // Move specimen down to position 0
    public class MoveDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                specimen.setTargetPosition(0);
                specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                specimen.setPower(1.0);
                initialized = true;
            }
            int curPos = specimen.getCurrentPosition();
            packet.put("Specimen Pos", curPos);
            return specimen.isBusy();
        }
    }

    public Action moveUp(int position) {
        return new MoveUp(position);
    }

    public Action moveDown() {
        return new MoveDown();
    }
}
