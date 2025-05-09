package org.firstinspires.ftc.teamcode.Auton;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElbowMovement {
    private DcMotorEx elbow;

    public ElbowMovement(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow.setDirection(DcMotor.Direction.FORWARD); // Adjust if needed
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(1.0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public class ElbowToPosition implements Action {
        private int targetPosition;
        private boolean initialized = false;

        public ElbowToPosition(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                elbow.setTargetPosition(targetPosition);
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow.setPower(1.0);
                initialized = true;
            }
            int currentPos = elbow.getCurrentPosition();
            packet.put("Elbow Position", currentPos);
            if (elbow.isBusy()) {
                return true; // still moving
            } else {
                elbow.setPower(0);
                return false; // movement complete
            }
        }
    }

    public Action elbowToPosition(int position) {
        return new ElbowToPosition(position);
    }
}
