package org.firstinspires.ftc.teamcode.Autons;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner..geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

/*
 * This is an example of a more complex autonomous using Road Runner.  This
 * demonstrates:
 * - Driving to a point
 * - Turning
 * - Driving to another point
 * - Doing a delay
 * - Driving to a final point
 *
 * This assumes you have your SampleMecanumDrive setup and are using the
 * trajectory sequence builder.
 */
@Autonomous(name = "RoadRunner Autonomous", group = "RoadRunner")
public class Basket extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system.  You will need to change the name here to
        // whatever you called your drive class.
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the robot's starting position.  This is VERY important!
        // You'll want to set it to whereever you start the robot on the field.
        // The pose is specified as (x, y, heading) in inches and radians.
        // The origin is at the center of the field, with +x being forward, +y being to the left, and +heading
        // being counterclockwise.  *MAKE SURE YOU SET THIS TO THE CORRECT STARTING POSITION!!!*
        Pose2d startPose = new Pose2d(-12, -63, Math.toRadians(90)); // Example starting pose - adjust as needed
        drive.setPoseEstimate(startPose);

        // Build the trajectory sequence.  This is a list of actions the robot will perform, in order.
        // Road Runner uses a builder pattern, which is a bit different than the
        // traditional way of creating objects.  You chain method calls together
        // to define the sequence.
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-12, -30))    // Drive forward
                .turn(Math.toRadians(90))         // Turn 90 degrees counterclockwise
                .lineTo(new Vector2d(24, -30))     // Drive to the side
                .waitSeconds(2)                  // Wait for 2 seconds
                .lineTo(new Vector2d(24, -12))    // Drive forward
                .build();

        // Send telemetry message to signify match auto started.
        telemetry.addData("Status", "Initialized.  Ready to start!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).
        waitForStart();

        runtime.reset();

        // Run the trajectory sequence.
        drive.followTrajectorySequence(trajectorySequence);

        // Add any final actions here.  For example, you might want to drop off a game piece.
        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
        sleep(2000);  //optional pause at end
    }
}
