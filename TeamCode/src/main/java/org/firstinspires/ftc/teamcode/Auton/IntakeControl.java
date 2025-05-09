package org.firstinspires.ftc.teamcode.Auton;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeControl {
    private DcMotorEx intakeMotor;

    public IntakeControl(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD); // Adjust as needed
        intakeMotor.setPower(0);
    }

    // Action to run intake forward
    public class IntakeIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intakeMotor.setPower(1.0);
                initialized = true;
            }
            // Running continuously, return true to keep running, false to stop
            return true;
        }
    }

    // Action to run intake backward
    public class IntakeOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intakeMotor.setPower(-1.0);
                initialized = true;
            }
            return true;
        }
    }

    // Action to stop intake motor
    public class IntakeStop implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intakeMotor.setPower(0);
                initialized = true;
            }
            return false; // Action completes immediately
        }
    }

    public Action intakeIn() {
        return new IntakeIn();
    }

    public Action intakeOut() {
        return new IntakeOut();
    }

    public Action intakeStop() {
        return new IntakeStop();
    }

    // Optional direct control methods for TeleOp

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
