package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FULL_DRIVE")
public class CenterStage_Full_23114 extends LinearOpMode {
    // variables
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = .5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final int        MAX_LIFT_HEIGHT         = 400;

    private ElapsedTime     ClawTime = new ElapsedTime();    // sets up a timer function
    private ElapsedTime     LiftTime = new ElapsedTime();
    private ElapsedTime     DroneTime = new ElapsedTime();
    private ElapsedTime     IntakeTime = new ElapsedTime();
    private ElapsedTime     IntakeServoTime = new ElapsedTime();
    private ElapsedTime     ReverseIntakeTime = new ElapsedTime();
    private double          frontLeftPower = 0;     // declare motor power variable
    private double          backLeftPower = 0;      // declare motor power variable
    private double          frontRightPower = 0;    // declare motor power variable
    private double          backRightPower = 0;     // declare motor power variable
    private double          denominator = 1;        // declare motor power calculation variable
    private int               precision = 2;          // chassis motor power reduction factor 1 = full 2 = half power 3 = third power 4 = quarter power
    private double          liftPower = 0.7;	// declare lift motor power variable *******
    private boolean         isClosed1 = false;	// variable for claw1 state
    private boolean         isClosed2 = false;	// variable for claw2 state
    private boolean         E_DoubleClose = false;	// ??
    private int             ClawInput = 0;		// claw ativation button stepping variable
    private int             LiftTarget = 0;		// target postion for lift encoder values
    private boolean         BeganPressed = false;	// ??
    private boolean         IntakeRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
         DcMotor Fleft = hardwareMap.dcMotor.get("Fleft");
        DcMotor Bleft = hardwareMap.dcMotor.get("Bleft");
        DcMotor Fright = hardwareMap.dcMotor.get("Fright");
        DcMotor Bright = hardwareMap.dcMotor.get("Bright");
        Servo Claw1 = hardwareMap.servo.get("Claw1");
        Servo Claw2 = hardwareMap.servo.get("Claw2");
        Servo Drone = hardwareMap.servo.get("Drone");
        DcMotor LeftLiftMotor = hardwareMap.dcMotor.get("Lift1");    // Lift Motors *******
        DcMotor RightLiftMotor = hardwareMap.dcMotor.get("Lift2");    // Lift Motors *******
        DcMotor ArmRotationMotor = hardwareMap.dcMotor.get("ArmRotationMotor");
        DcMotor IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        Servo IntakeServo = hardwareMap.servo.get("IntakeServo");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set lift motors to use encoders
		
        RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

	    // Set motor behavior to hold postion when stopped
		
        Fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Lift Motors behavior 
		
        RightLiftMotor.setTargetPosition(0);
        LeftLiftMotor.setTargetPosition(0);
        RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	    // Set servo positions		
		
        Claw1.setPosition(0);
        Claw2.setPosition(0);
        IntakeServo.setPosition(0);

// Beginning of code to interpret operator controlled movements

        waitForStart();

        while (opModeIsActive()) {

            // check for driving input

            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            telemetry.update();

            // calculate motor movement math and adjust according to turbo or manual precision mode selection

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if(gamepad1.left_bumper){
                precision = 1;            // set default speed to full power - TURBO MODE
            }
            else if(gamepad1.right_bumper){
                precision = 4;            // set speed to 1/4 power
            }
            else{precision = 2;}		// reset default speed to half power

            denominator = denominator * precision;
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

    	    // check for claw movement input and cycle through claw stages

            switch(ClawInput){
                case 0:				//If the answer is 0 it runs the code here
                    if(gamepad2.b && ClawTime.milliseconds() > 300 && isClosed1){ //This checks to see if it has been more than 300 milliseconds since the "a" button has been pressed
                        ClawTime.reset();//This resets the timer to 0
                        Claw1.setPosition(0);//this opens claw 1
                        isClosed1 = false;
                        ClawInput++;//This adds one to the variable to keep track of times the "a" button has been pressed
                    }
		        case 1:			   	//If the answer is 1 it runs the code here
                    if(gamepad2.b && ClawTime.milliseconds() > 300 && isClosed2){ //This checks to see if it has been more than 300 milliseconds since the "a" button has been pressed
                        ClawTime.reset();//This resets the timer to 0
                        Claw2.setPosition(0);//this opens claw 2
                        isClosed2 = false;
                        ClawInput++;//This adds one to the variable to keep track of times the "a" button has been pressed
                    }
                case 2:				//If the answer is 2 it runs the code here
                    if(gamepad2.b && ClawTime.milliseconds() > 300){ //This checks to see if the "a" button has been pressed on gamepad1
                        ClawTime.reset();//This starts the timer since the "a" button was just pressed
                        Claw1.setPosition(1);//this closes claw 1
                        Claw2.setPosition(1);//this closes claw 2
                        isClosed1 = true;
                        isClosed2 = true;
                        E_DoubleClose = true;
                        ClawInput = 0;//This adds one to the variable to keep track of times the "a" button has been pressed
                    }
		    }
            if(gamepad2.right_bumper && !E_DoubleClose && ClawTime.milliseconds() > 300){
                Claw1.setPosition(1);
                Claw2.setPosition(1);
                E_DoubleClose = true;
                ClawTime.reset();
            }
            else if(gamepad2.right_bumper && E_DoubleClose && ClawTime.milliseconds() > 300){
                Claw1.setPosition(0);
                Claw2.setPosition(0);
                E_DoubleClose = false;
                ClawTime.reset();
            }

            // check for lift movement input

            if(gamepad2.a && LiftTime.seconds() > 1.0){	// Move lift to preset level 1
                LiftTarget = 100;
                LiftTime.reset();
            }
            else if (gamepad2.x && LiftTime.seconds() > 1.0){	// Move lift to preset level 2
                LiftTarget = 200;
                LiftTime.reset();
            }
            else if (gamepad2.y && LiftTime.seconds() > 1.0){	// Move lift to preset level 2
                LiftTarget = 350;
                LiftTime.reset();
            }

            if(gamepad2.dpad_up && RightLiftMotor.getCurrentPosition() < MAX_LIFT_HEIGHT - 10){
                LiftTarget = LiftTarget + 10;
            }
            if(gamepad2.dpad_down && RightLiftMotor.getCurrentPosition() > 10){
                LiftTarget = LiftTarget - 10;
            }
            if(LiftTarget > RightLiftMotor.getCurrentPosition()){liftPower = 0.7;}     // ?
                else{liftPower = 0.3;} 
		    
	    	// issue lift power for movement

		    if(!(LiftTarget > MAX_LIFT_HEIGHT)){
                RightLiftMotor.setTargetPosition(LiftTarget);
                LeftLiftMotor.setTargetPosition(LiftTarget);
                RightLiftMotor.setPower(liftPower);
                LeftLiftMotor.setPower(liftPower);
            }
   		    // Drone launching

            if(gamepad1.b){
                DroneTime.reset();
                BeganPressed = true;
            }
            if(!gamepad1.b && DroneTime.seconds() > 3.0 && BeganPressed == true){
                Drone.setPosition(1);
            }
            else{BeganPressed = false;}
                    
		    // issue chassis power for movement

            if(gamepad1.a && !IntakeRunning){
                IntakeMotor.setPower(1);
                IntakeRunning = true;
                IntakeTime.reset();
            }
            if(gamepad1.a && IntakeTime.seconds() > 0.25){
                IntakeMotor.setPower(0);
                IntakeRunning = false;
                IntakeTime.reset();
            }

    		// Intake code
		    
	        if(gamepad1.x){
                if(ReverseIntakeTime.seconds() > 0.25){
                    IntakeMotor.setPower(-1);
                }
                else{IntakeMotor.setPower(0);
                }
            }
            else{
                ReverseIntakeTime.reset();
            }

            if(gamepad2.left_bumper) {
                IntakeServo.setPosition(1);
                IntakeServoTime.reset();
            }
            if(gamepad2.left_bumper && IntakeServoTime.seconds() > 1.0){
                IntakeServo.setPosition(0);
                IntakeServoTime.reset();
            }
		    // issue chassis power for movement
            
            Fleft.setPower(frontLeftPower);
            Bleft.setPower(backLeftPower);
            Fright.setPower(frontRightPower);
            Bright.setPower(backRightPower);
        }
    }
}
