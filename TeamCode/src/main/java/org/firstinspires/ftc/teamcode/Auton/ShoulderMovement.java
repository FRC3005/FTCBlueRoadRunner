package org.firstinspires.ftc.teamcode.Auton;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOpParameters;

public class ShoulderMovement {
    private DcMotorEx shoulder;

    public ShoulderMovement(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulder.setDirection(TeleOpParameters.SHOULDER_DIRECTION);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public class ShoulderUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shoulder.setTargetPosition(3000); // Set target position for upward movement
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1.0);
                initialized = true;
            }
            double pos = shoulder.getCurrentPosition();
            packet.put("ShoulderPos", pos);
            if (shoulder.isBusy()) {
                return true; // Still moving
            } else {
                shoulder.setPower(0);
                return false; // Movement complete
            }
        }
    }

    public Action shoulderUp() {
        return new ShoulderUp();
    }

    public class ShoulderDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shoulder.setTargetPosition(0); // Set target position for downward movement
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(-1.0);
                initialized = true;
            }
            double pos = shoulder.getCurrentPosition();
            packet.put("ShoulderPos", pos);
            if (shoulder.isBusy()) {
                return true; // Still moving
            } else {
                shoulder.setPower(0);
                return false; // Movement complete
            }
        }
    }

    public Action shoulderDown() {
        return new ShoulderDown();
    }

    public class ShoulderToPosition implements Action {
        private int targetPosition;
        private boolean initialized = false;

        public ShoulderToPosition(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                shoulder.setTargetPosition(targetPosition);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setPower(1.0);
                initialized = true;
            }
            double pos = shoulder.getCurrentPosition();
            packet.put("ShoulderPos", pos);
            if (shoulder.isBusy()) {
                return true; // Still moving
            } else {
                shoulder.setPower(0);
                return false; // Movement complete
            }
        }
    }

    public Action shoulderToPosition(int position) {
        return new ShoulderToPosition(position);
    }
}
