package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "4 Basket Auto", group = "Autonomous")
public class Spec3 extends LinearOpMode {

    private MecanumDrive drive; // Declare drive as an instance variable

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -62, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, initialPose); // Initialize drive

        waitForStart();

        executeAutonomous();
    }

    private void executeAutonomous() {
        // Define trajectories
        TrajectoryActionBuilder path1 = Clipping1();
        TrajectoryActionBuilder path2 = PushingIn(path1);
        TrajectoryActionBuilder path3 = Clipping2(path2);
        TrajectoryActionBuilder path4 = pickingup2(path3);







        Actions.runBlocking(
                new SequentialAction(
                        path1.build(),
                        path2.build(),
                        path3.build()
                )
        );
    }

    private TrajectoryActionBuilder Clipping1() {
        return drive.actionBuilder(new Pose2d(-23, -62, Math.toRadians(0)))
                .strafeTo(new Vector2d(0, -30));     //Clipped the first one

    }

    private TrajectoryActionBuilder PushingIn(TrajectoryActionBuilder previousPath) {
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -40)) //Started pushing 2 In
                .splineToLinearHeading(new Pose2d(35, -34, Math.toRadians(0)), 0)
                .strafeTo(new Vector2d(35, -9))
                .strafeTo(new Vector2d(45, -9))
                .strafeTo(new Vector2d(45, -48))
                .splineToLinearHeading(new Pose2d(56, -9, Math.toRadians(0)), 0)
                .strafeTo(new Vector2d(56, -48))
                .strafeTo(new Vector2d(40, -48))
                .turn(Math.toRadians(-90)); //Finished pushing 2 in and ready to start clipping
    }

    private TrajectoryActionBuilder Clipping2(TrajectoryActionBuilder previousPath){
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(40, -62)) //Got the first one
                .strafeTo(new Vector2d(0, -50)) //Leaving after getting the speci
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0, -32)); //At the bar to score the speci
    }
    private TrajectoryActionBuilder pickingup2(TrajectoryActionBuilder previousPath) {
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(40, -62)); //Picking up a second one

    }
}
