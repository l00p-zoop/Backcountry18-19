package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class DriveUtilities {

    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    //Declare Motors and servos
    DcMotor motor1, motor2, motor3, motor4, liftmotor, rotatemotor, linac, extendmotor;
    Servo hookservo, mservo;
    LinearOpMode opMode;
    //declare constants
    private static final double     ROBOT_DIAMETER          = 16;
    private static final double     DIST_BETWEEN_EDGES      = 12;
    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;
    private static final double     ACCELERATION_GRADIENT   = 1.0/600;
    private static final double     DECELERATION_GRADIENT   = 1.0/2000;//rate of acceleation and decelleration of robot
    private static final double     MINIMUM_POWER           = 0.1;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public DriveUtilities(HardwareMap hardwareMap, LinearOpMode opMode) {
        //Initialize motors and variables
        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");
        liftmotor = hardwareMap.get(DcMotor.class, "lm");
        rotatemotor = hardwareMap.get(DcMotor.class, "rm");
        extendmotor = hardwareMap.get(DcMotor.class, "em");
        linac = hardwareMap.get(DcMotor.class, "lc");
        sensorRange = hardwareMap.get(DistanceSensor.class, "dst");
        mservo = hardwareMap.get(Servo.class,"ms");

        //Set encoder modes
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.opMode = opMode;

    }
    //Function to move the robot on a speed gradient
    private double getPower(double targetPower, int curPos, int endPos, int startingPos){
        double rampUpPower = Math.abs(curPos - startingPos) * ACCELERATION_GRADIENT;
        double rampDownPower = Math.abs(endPos - curPos) * DECELERATION_GRADIENT;
        opMode.telemetry.addData("rampDownPower", rampDownPower);
        opMode.telemetry.addData("rampUpPower", rampUpPower);
        //opMode.telemetry.update();
        double clippedPower = Math.min(Math.min(targetPower, rampUpPower), rampDownPower);
        opMode.telemetry.addData("clipped power ", clippedPower);
        return Math.max(MINIMUM_POWER, clippedPower);  //Ensure power is high enough to always move the robot
        //return targetSpeed;
    }



    public void driveFB(double speed, double inches, double timeout){
        //function to move the robot Forwards and Backwards when called in autonomous programs
        drive(speed,speed,inches,-inches,timeout,motor2,motor3);
    }
    public void driveLR(double speed, double inches, double timeout){
        //function to move the robot Left and Right when called in autonomous programs
        drive(speed,speed,inches,-inches,timeout,motor1,motor4);
    }

    public void rotate(double degrees){//Function to facilitate rotating the robot
        double inches = degrees * (ROBOT_DIAMETER*Math.PI)/360;
        drive(.3,.3,-inches,-inches,4,motor1,motor4);


    }

    //Function to move the robot in an arcing motion
    public void arc(double degrees, double radius ,double speed, String side, String type) {//Radius is measured from inner wheel
        //Function to let the robot move in an arcing motion
        double circNear = 2 * radius * Math.PI;
        double circFar = 2 * (radius + DIST_BETWEEN_EDGES) * Math.PI;

        double arcNear = circNear * (degrees / 360);
        double arcFar = circFar * (degrees / 360);

        double innerSpeed = ((arcNear * speed) / arcFar);

        opMode.telemetry.addData("left: ", speed);
        opMode.telemetry.addData("right: ", innerSpeed);
        opMode.telemetry.update();


        //Move the robot depending on which direction you specify

        if (side == "L") {
            if (type == "BLFR") {
                drive(innerSpeed, speed, arcFar, -arcNear, 4,motor2,motor3);
            } else {
                drive(innerSpeed, speed, arcFar, -arcNear, 4,motor1,motor4);
            }
        } else {
            if (type == "BLFR") {
                drive(speed, innerSpeed, arcNear, -arcFar, 4,motor2,motor3);
            } else {
                drive(speed, innerSpeed, arcNear, -arcFar, 4,motor1,motor4);
            }
            opMode.sleep(1000);
        }
    }


    public void logPositions(){
        //Prints the motor's encoder positions when called
        opMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                motor1.getCurrentPosition(),
                motor4.getCurrentPosition());
        opMode.telemetry.update();
    }
    private void drive(double Lspeed, double Rspeed,
                           double leftInches, double rightInches,
                           double timeoutS, DcMotor m1, DcMotor m2 ) {
        //Function to facilitate encoder driving
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active


            // Determine new target position, and pass to motor controller

            int leftstart = m1.getCurrentPosition();
            int rightstart = m2.getCurrentPosition();
            newLeftTarget = m1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = m2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            m1.setTargetPosition(newLeftTarget);
            m2.setTargetPosition(newRightTarget);

            //Move motors to position
            m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            //m1.setPower(Math.abs(Rspeed));
            //m2.setPower(Math.abs(Lspeed));

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (m1.isBusy() && m2.isBusy())) {


                m1.setPower(getPower(Math.abs(Lspeed), m1.getCurrentPosition(), newLeftTarget,leftstart));
                m2.setPower(getPower(Math.abs(Rspeed), m2.getCurrentPosition(), newRightTarget, rightstart));

                opMode.telemetry.addData("RmotorPower " ,getPower(Math.abs(Rspeed), m2.getCurrentPosition(), newRightTarget, rightstart));
                opMode.telemetry.addData("LmotorPower ", getPower(Math.abs(Lspeed), m1.getCurrentPosition(), newLeftTarget,leftstart));



                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        m1.getCurrentPosition(),
                        m2.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            m1.setPower(0);
            m2.setPower(0);

            // Turn off RUN_TO_POSITION
            m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }



