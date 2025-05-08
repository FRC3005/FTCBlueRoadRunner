    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.robochargers.MecanumDrivetrain;

    @TeleOp(name="TeleOp", group="Iterative Opmode")

    public class TeleOp_Main extends OpMode
    {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private MecanumDrivetrain drivetrain;

        private DcMotorEx shoulder = null;
        private DcMotorEx elbow = null;
        private DcMotorEx specimen = null;
        private DcMotorEx intake = null;
        private CRServo specLServo =  null;
        private CRServo specRServo = null;
        private CRServo climberLServo = null;
        private CRServo climberRServo = null;

        private boolean lastLeftButton = false;
        private boolean lastRightButton = false;
        private boolean lastXButton = false;
        private boolean lastArrowUPButton = false;
        private boolean lastArrowDOWNButton = false;
        private int climberToggle = 0;

        int shoulderTarget = 0;
        int elbowTarget = 0;
        int climberTarget = 0; // Any Number Smaller Than 0 or Larger Than 255 Will Stop The Climber

        private double speedMulti = 1.0;

        enum SpecimenState {
            HOME,
            WALL,
            WALL_LIFT,
            LOW_CLIP,
            LOW_EJECT,
            HIGH,
            HIGH_CLIP,
            HIGH_EJECT
        }

        SpecimenState specimenState = SpecimenState.HOME;

        /*
        * Code to run ONCE when the driver hits INIT
        */
        @Override
        public void init() {
            telemetry.addData("Status", "Initializing");
            telemetry.update();

            drivetrain = new MecanumDrivetrain(hardwareMap, DriveParametersTemplate.class);
            
            // shoulder init
            shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
            shoulder.setDirection(DcMotor.Direction.REVERSE);
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setTargetPosition(0);
            shoulder.setPower(1.0);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // elbow init
            elbow = hardwareMap.get(DcMotorEx.class, "elbow");
            elbow.setDirection(DcMotor.Direction.FORWARD);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setTargetPosition(0);
            elbow.setPower(1.0);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // specimen init
            specimen = hardwareMap.get(DcMotorEx.class, "spec");
            specimen.setDirection(DcMotor.Direction.REVERSE);
            specimen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            specimen.setTargetPosition(0);
            specimen.setPower(1.0);
            specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            // intake init
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);

            // specLServo & specRServo
            specLServo = hardwareMap.get(CRServo.class, "specLServo");
            specRServo = hardwareMap.get(CRServo.class, "specRServo");

            // climberLServo & climberRServo
            climberLServo = hardwareMap.get(CRServo.class, "climberLServo");
            climberRServo = hardwareMap.get(CRServo.class, "climberRServo");

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }

        //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
        @Override
        public void init_loop() {
        }

        //Code to run ONCE when the driver hits PLAY
        @Override
        public void start() {
            runtime.reset();
        }

        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
        @Override
        public void loop() {
            drivetrain.updateHeading();

            drivetrain.runMotorsFieldOriented(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x * speedMulti);
            // drivetrain.runMotorsMixed(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

            if (gamepad1.right_stick_button){
                drivetrain.zeroHeading();
            }
            
            //telemetry stuff
            telemetry.addData("Shoulder Pos", shoulder.getCurrentPosition());
            telemetry.addData("Elbow Pos", elbow.getCurrentPosition());
            telemetry.addData("Specimen Pos", specimen.getCurrentPosition());
            telemetry.addData("Speed Multi", speedMulti);
            
            //Arm Target Postions
            int shoulderStartPos = 0;
            int shoulderIntakePos = 400;
            int shoulderScoreHighPos = 1870;
            int shoulderHoverPos = 400;
            int elbowStartPos = 0;
            int elbowHoverPos = 950;
            int elbowScoreHighPos = 0;
            int elbowIntakePos = 1300;

            int specimenHome = 0;
            int specimenWall = 0;
            int specimenWallLift = 450;
            int specimenLowClip = 0;
            int specimenLowEject = 0;
            int specimenHigh = 1150;
            int specimenHighClip = 750;
            int specimenHighEject = 750;
            int specimenTolerance = 10;

            //Speed Controls
            if (
                (shoulderTarget == shoulderIntakePos && elbowTarget == elbowIntakePos)
                ||
                (specimenState == SpecimenState.WALL)
                )
            {
                speedMulti = 0.5;
            }
            else if (
                (shoulderTarget == shoulderHoverPos
                && elbowTarget == elbowHoverPos)
                )
            {
                speedMulti = 0.65; 
            }
            else {
                speedMulti = 1.0;
            }

            // Shoulder & Elbow Controll
            if (gamepad1.a){
                shoulderTarget = shoulderStartPos;
                elbowTarget = elbowStartPos;
            }
            
            if (shoulderTarget == shoulderHoverPos
            && elbowTarget == elbowHoverPos
            && gamepad1.right_trigger > 0.5){
                shoulderTarget = shoulderIntakePos;
                elbowTarget = elbowIntakePos; 
            }
            if (shoulderTarget == shoulderIntakePos
            && elbowTarget == elbowIntakePos
            && gamepad1.right_trigger < 0.5){
                shoulderTarget = shoulderHoverPos;
                elbowTarget = elbowHoverPos;
            }
            
            if (gamepad1.y){
                shoulderTarget = shoulderScoreHighPos;
                elbowTarget = elbowScoreHighPos;
            }
            
            if (gamepad1.b){
                shoulderTarget = shoulderHoverPos;
                elbowTarget = elbowHoverPos;
            }
            
            // Servo Intake
            if (gamepad1.right_trigger > 0.5 && shoulderTarget != shoulderStartPos){  /// Intake
                intake.setPower(1.0);
            }
            else if (gamepad1.left_trigger > 0.5 && shoulderTarget != shoulderStartPos){  // Outtake
                intake.setPower(-1.0);
            }
            else {
                if (shoulderTarget != shoulderStartPos) {
                    intake.setPower(0.1);   
                }
                else {
                    intake.setPower(0); 
                }
            }

            // Specimen State Controls 
            if (specimenState == SpecimenState.HOME) {
                specimen.setTargetPosition(specimenHome);
                specLServo.setPower(0.0);
                specRServo.setPower(0.0);

                if (gamepad1.right_bumper && !lastRightButton){
                    specimenState = SpecimenState.WALL;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.WALL;
                }
            }    
            else if (specimenState == SpecimenState.WALL){
                specimen.setTargetPosition(specimenWall);
                specLServo.setPower(1.0);
                specRServo.setPower(-1.0);

                if (gamepad1.x){
                    specimenState = SpecimenState.HOME;
                }
                if (gamepad1.right_bumper && !lastRightButton){
                    specimenState = SpecimenState.HIGH;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.WALL_LIFT;
                }
            }
            else if (specimenState == SpecimenState.WALL_LIFT){
                specimen.setTargetPosition(specimenWallLift);
                specLServo.setPower(0.0);
                specRServo.setPower(0.0);

                if (gamepad1.x){
                    specimenState = SpecimenState.HOME;
                }
                if (gamepad1.right_bumper && !lastRightButton){
                    specimenState = SpecimenState.HIGH;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.LOW_CLIP;
                }
            }
            else if (specimenState == SpecimenState.LOW_CLIP){
                specimen.setTargetPosition(specimenLowClip);
                specLServo.setPower(0.0);
                specRServo.setPower(0.0);

                if (Math.abs(specimen.getCurrentPosition() - specimenLowClip) <= specimenTolerance){
                    specimenState = SpecimenState.LOW_EJECT;
                }
                if (gamepad1.x){
                    specimenState = SpecimenState.HOME;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.HOME;
                }
            }
            else if (specimenState == SpecimenState.LOW_EJECT){
                specimen.setTargetPosition(specimenLowEject);
                specLServo.setPower(-1.0);
                specRServo.setPower(1.0);

                if (gamepad1.x){
                    specimenState = SpecimenState.HOME;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.HOME;
                }
            }
            else if (specimenState == SpecimenState.HIGH){
                specimen.setTargetPosition(specimenHigh);
                specLServo.setPower(0.0);
                specRServo.setPower(0.0);

                if (gamepad1.x){
                    specimenState = SpecimenState.HOME;
                }
                if (gamepad1.right_bumper && !lastRightButton){
                    specimenState = SpecimenState.HIGH_CLIP;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.WALL_LIFT;
                }
            }
            else if (specimenState == SpecimenState.HIGH_CLIP){
                specimen.setTargetPosition(specimenHighClip);
                specLServo.setPower(0.0);
                specRServo.setPower(0.0);

                if (Math.abs(specimen.getCurrentPosition() - specimenHighClip) <= specimenTolerance){
                    specimenState = SpecimenState.HIGH_EJECT;
                }
            }
            else if (specimenState == SpecimenState.HIGH_EJECT){
                specimen.setTargetPosition(specimenHighEject);
                specLServo.setPower(-1.0);
                specRServo.setPower(1.0);

                if (gamepad1.x){
                    specimenState = SpecimenState.HOME;
                }
                if (gamepad1.left_bumper && !lastLeftButton){
                    specimenState = SpecimenState.WALL;
                }
                if (gamepad1.right_bumper && !lastRightButton){
                    specimenState = SpecimenState.HOME;
                }
            }

            if (climberToggle == 1 && climberTarget < 255) { // Climber Is Going Up
                climberTarget += 1;
                climberLServo.setPower(1.0);
                climberRServo.setPower(-1.0);
            }

            if (climberToggle == 0) { // Climber Is Not Moving
                climberTarget += 0;
                climberLServo.setPower(0);
                climberRServo.setPower(0);
            }

            if (climberToggle == -1 && climberTarget > 0) { // Climber Is Going Down
                climberTarget -= 1;
                climberLServo.setPower(-1.0);
                climberRServo.setPower(1.0);
            }

            if (climberTarget <= 0) {
                climberToggle = 0;           
            }
            if  (climberTarget >= 255) {
                climberToggle = 0;
            }

            if (gamepad1.dpad_up && !lastArrowUPButton && climberToggle != 1) {
                climberToggle += 1;
            }
            if (gamepad1.dpad_down && !lastArrowDOWNButton && climberToggle != -1) {
                climberToggle -= 1;  
            }

            shoulder.setTargetPosition(shoulderTarget);
            elbow.setTargetPosition(elbowTarget);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Climber Position", climberTarget);
            telemetry.addData("Climber Toggle", climberToggle);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gyro Orientation", drivetrain.getHeading());
            telemetry.addData("Drive Joystick Raw", gamepad1.left_stick_y);
            telemetry.addData("Strafe Joystick Raw", gamepad1.left_stick_x);
            telemetry.addData("Rotate Joystick Raw", gamepad1.right_stick_x);
            telemetry.update();

            lastLeftButton = gamepad1.left_bumper; 
            lastRightButton = gamepad1.right_bumper;
            lastXButton = gamepad1.x;
            lastArrowUPButton = gamepad1.dpad_up;
            lastArrowDOWNButton = gamepad1.dpad_down;
        }

        //Code to run ONCE after the driver hits STOP
        @Override
        public void stop() {
        }

    }
