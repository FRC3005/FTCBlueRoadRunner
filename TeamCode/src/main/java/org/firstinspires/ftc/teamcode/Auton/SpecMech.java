package org.firstinspires.ftc.teamcode.Auton;

/*
    How to use Spec Mech:

    SpecMech spec = new SpecMech(hardwareMap);
    sequence.add(spec.prepIntake());
*/

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.TeleOpParameters;
import org.firstinspires.ftc.teamcode.Auton.AutonSettings;

public class SpecMech {
    private DcMotorEx specimen;
    private CRServo specLServo, specRServo;

    public SpecMech(HardwareMap hardwareMap) {
        specimen = hardwareMap.get(DcMotorEx.class, "spec");
        specimen.setDirection(TeleOpParameters.SPECIMEN_DIRECTION);
        specimen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimen.setTargetPosition(0);
        specimen.setPower(1.0);
        specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specLServo = hardwareMap.get(CRServo.class, "specLServo");
        specRServo = hardwareMap.get(CRServo.class, "specRServo");
    }

    public Action moveToState(String state) {
        int specimenTarget;

        switch (state.toLowerCase()) {
            case "home":
                specimenTarget = AutonSettings.SPECIMEN_HOME;
                break;
            case "wall":
                specimenTarget = AutonSettings.SPECIMEN_WALL;
                break;
            case "high":
                specimenTarget = AutonSettings.SPECIMEN_HIGH;
                break;
            default:
                throw new IllegalArgumentException("Unknown specimen state: " + state);
        }

        return moveToSpecimenPosition(specimenTarget);
    }

    public Action moveToSpecimenPosition(int target) {
        return packet -> {
            specimen.setTargetPosition(target);
            specimen.setPower(1.0);

            packet.put("Specimen Pos", specimen.getCurrentPosition());

            if (!specimen.isBusy()) {
                specimen.setPower(0);
                return false;
            }

            return true;
        };
    }

    public Action prepIntake() {
        return new SequentialAction(
                moveToSpecimenPosition(AutonSettings.SPECIMEN_WALL),
                (TelemetryPacket packet) -> {
                    specLServo.setPower(1.0);
                    specRServo.setPower(-1.0);
                    return false;
                },
                moveToSpecimenPosition(AutonSettings.SPECIMEN_WALL_LIFT),
                (TelemetryPacket packet) -> {
                    specLServo.setPower(0);
                    specRServo.setPower(0);
                    return false;
                }
        );
    }

    public Action scoreHigh() {
        return new SequentialAction(
                moveToSpecimenPosition(AutonSettings.SPECIMEN_HIGH),
                moveToSpecimenPosition(AutonSettings.SPECIMEN_HIGH_CLIP),
                new SleepAction(1.0),
                packet -> {
                    specLServo.setPower(-1.0);
                    specRServo.setPower(1.0);
                    return false;
                },
                new SleepAction(0.5),
                packet -> {
                    specLServo.setPower(0);
                    specRServo.setPower(0);
                    return false;
                },
                moveToSpecimenPosition(AutonSettings.SPECIMEN_HOME)
        );
    }
    private Action stopSpecimenServos() {
        return packet -> {
            specLServo.setPower(0);
            specRServo.setPower(0);
            return false;
        };
    }

}

