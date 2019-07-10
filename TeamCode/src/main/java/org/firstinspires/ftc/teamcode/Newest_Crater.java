/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "eh", group = "Concept")
@Disabled
public class Newest_Crater extends LinearOpMode {
    //Vuforia setup
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUKN4dL/////AAABmZL4EHQ+ukIzpu89viYzaR4/jtXdbGjrssYSj0jYOGUszrxyjIZgyw6bnlbe3u5cMKxWpBP4i8HPWkAKoqmaJmRHrv8Ho7vPazLYJ65ZoLevtugk9OihzarMGR1FSjcThK3Tx3k2zpgCwaZWGopNS4ObXr5mFqdZkLmE11hEj7QV/sX886rFKB6Q9ySF1L4eYhwZmsArJjf5BGgenPwg0/Ou8yhB3VXk5++EAbLQwwlSFYZnQkS8h9TeGHe7LL5CbPgYptx+qo8iLJqqx6cqkabRHyVv1rzTxeJw42wFpHAono6FcMrWOiuKZs2xBiKKlUlw2qI8cl6sHbhiRPcOtLO3X5p7f0+fInRqOY4pakdS";
    //Declaring COnstants
    static final double     DIST_BETWEEN_EDGES      = 12;
    static final double     ROBOT_DIAMETER          = 16;
    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;
    private VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();

    private TFObjectDetector tfod;

    private DistanceSensor sensorRange;
    //Creating Motors
    DcMotor motor1, motor2, motor3, motor4, liftmotor, rotatemotor, linac;
    Servo hookservo, mservo;
    //Establishing Variables
    boolean chkloop = true;
    boolean drploop = true;
    boolean timecheck = true;
    double inchCount = 21.5;
    double tickCount = 0;
    double doubleCount = 0;
    double revCount = 0;
    double movetimem = 0;
    double movetimes = 0;
    @Override
    public void runOpMode() {

        //Initializing motors, servos, and our distance sensor
        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");
        liftmotor = hardwareMap.get(DcMotor.class, "lm");
        rotatemotor = hardwareMap.get(DcMotor.class, "rm");
        linac = hardwareMap.get(DcMotor.class, "lc");
        sensorRange = hardwareMap.get(DistanceSensor.class, "dst");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        hookservo = hardwareMap.get(Servo.class, "hs");
        mservo = hardwareMap.get(Servo.class,"ms");

        //Setting encoder modes
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initVuforia();
        //Initializing vuforia and tensorflow
        mservo.setPosition(0.05);
        hookservo.setPosition(-.1);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("Path0", "Starting at %7d :%7d",
                motor1.getCurrentPosition(),
                motor4.getCurrentPosition());
        telemetry.update();
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            //Looks for particles
            runtime.reset();
            while (chkloop) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() != 0) {
                        int goldMineralX = -1;
                        //find Gold Particle location in respect to the camera
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1) {
                          if(goldMineralX > 0 && goldMineralX < 150){
                              //Identifies if it is in the left position
                              telemetry.addData("Position = ", "Left");
                              telemetry.update();
                              leftmove();
                              chkloop = false;
                          } else if(goldMineralX >= 150 && goldMineralX <= 475){
                              //Identifies if it is in the center position
                              telemetry.addData("Position = ", "Center");
                              telemetry.update();
                              centermove();
                              chkloop = false;
                          } else if(goldMineralX > 475){
                              //Identifies if it is in the right position
                              telemetry.addData("Position = ", "Right");
                              telemetry.update();
                              rightmove();
                              chkloop = false;
                          } else if(runtime.seconds() >=10){
                              //Backup function if it takes more than 10 seconds to identify
                              telemetry.addData("Position = ", "Timeout");
                              telemetry.update();
                              centermove();
                              chkloop = false;
                          }
                        } else if (runtime.seconds() >= 10){
                            //Backup function if it takes more than 10 seconds to identify
                            telemetry.addData("Position = ", "Timeout");
                            telemetry.update();
                            leftmove();
                            chkloop = false;
                        }
                      }
                      telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

    }
    public void centermove() {
        //Function for if the gold is in the central position
        //Drops the robot
        droprobot();
        //Updates the console
        telemetry.addData("Movement", " Done");
        telemetry.update();
        //Pushes particle and moves to depot
        BLFRDrive(.4,.5,12,-12,3);
        FLBRDrive(.3,.3,5,-5,3);
        BLFRDrive(.4,.5,7.5,-7.5,3);
        /*BLFRDrive(.5,.5,-7.5,7.5,3);
        FLBRDrive(.3,.3,12,-12,3);
        finalmove();*/
    }
    public void leftmove(){
        //Function for if the gold is in the left position
        //Drops the robot
        droprobot();
        telemetry.addData("Movement", " Done");
        telemetry.update();
        //Pushes particle and moves to depot
        BLFRDrive(.4,.5,12,-12,3);
        FLBRDrive(.3,.3,18,-18,2);
        BLFRDrive(.4,.5,10,-10,3);
        /*BLFRDrive(.5,.5,-10,10,3);
        finalmove();*/


    }
    public void rightmove(){
        //Function for if the gold is in the right position
        //Drops the robot
        droprobot();
        telemetry.addData("Movement", " Done");
        telemetry.update();
        //Pushes particle and moves to depot
        BLFRDrive(.4,.5,12,-12,3);
        FLBRDrive(.3,.3,-9,9,2);
        BLFRDrive(.4,.5,7.5,-7.5,3);
        /*BLFRDrive(.5,.5,-7,7,3);
        FLBRDrive(.3,.3,30,-30,2);
        finalmove();*/


    }
    public void timeout(){
        //drops the robot in an emergency
        droprobot();
        telemetry.addData("Movement", " Done");
        telemetry.update();
    }
    public void finalmove(){

        //Unused function
        arc(140,17,.4,"R", "FLBR");
        FLBRDrive(.3,.3,33,-33,5);
        mservo.setPosition(1);
        sleep(1000);
        FLBRDrive(.3,.3,-5,5,3);
        BLFRDrive(.3,.3,5,-5,3);
        FLBRDrive(.3,.3,-80,80,8);
    }



    public void droprobot(){//Function to drop the robot
        liftmotor.setPower(.2);
        // motor2.setPower(.15);
        while (drploop) {
            if (sensorRange.getDistance(DistanceUnit.CM) < 4.2) {
                //continue dropping until the wheels are touching the ground
                drploop = false;
            }
        }

        //Let the lift continue to raise just enough to the point where the whole weight of the robot is resting on the wheels
        telemetry.addData("Robot:","Dropped");
        telemetry.update();
        sleep(750);
        liftmotor.setPower(0);//stop the lift
        //Disengage hook
        hookservo.setPosition(.35);

        motor1.setPower(-.15);
        motor4.setPower(.15);
        sleep(300);//strafe out from the lander just a tiny bit
        motor1.setPower(0);
        motor4.setPower(0);
    }
    public void rotate(double degrees){//Function to facilitate rotating the robot
        double inches = degrees * (ROBOT_DIAMETER*Math.PI)/360;
        FLBRDrive(.3,.3,-inches,-inches,4);

    }
    public void arc(double degrees, double radius ,double speed, String side, String type){//Radius is measured from inner wheel
        //Function to let the robot move in an arcing motion
        double circNear = 2*radius*Math.PI;
        double circFar = 2*(radius + DIST_BETWEEN_EDGES)*Math.PI;

        double arcNear = circNear * (degrees / 360);
        double arcFar = circFar * (degrees / 360);

        double innerSpeed = ((arcNear*speed)/arcFar);

        telemetry.addData("left: ", speed);
        telemetry.addData("right: ", innerSpeed);
        telemetry.update();

        //sleep(2000);

        //Move the robot depending on which direction you specify

        if(side == "L"){
            if(type =="BLFR") {
                BLFRDrive(innerSpeed, speed, arcFar, -arcNear, 4);
            } else {
                FLBRDrive(innerSpeed, speed, arcFar, -arcNear, 4);
            }
        }else {
            if(type == "BLFR") {
                BLFRDrive(speed, innerSpeed, arcNear, -arcFar, 4);
            } else {
                FLBRDrive(speed, innerSpeed, arcNear, -arcFar, 4);
            }
            sleep(1000);
        }
        //run inner motor for arcNear inches at innerSpeed power




    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    //Function to program the front left and back right motor
    public void FLBRDrive(double lspeed,double rspeed,
                          double leftInches, double rightInches,
                          double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motor4.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            motor1.setTargetPosition(newLeftTarget);
            motor4.setTargetPosition(newRightTarget);


            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motor1.setPower(Math.abs(lspeed));
            motor4.setPower(Math.abs(rspeed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor1.isBusy() && motor4.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motor1.getCurrentPosition(),
                        motor4.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor1.setPower(0);
            motor4.setPower(0);

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }//Function to make motor programming easier

    public void BLFRDrive(double Lspeed, double Rspeed,
                          double leftInches, double rightInches,
                          double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motor3.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            motor2.setTargetPosition(newLeftTarget);
            motor3.setTargetPosition(newRightTarget);

            //Move motors to position
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motor2.setPower(Math.abs(Rspeed));
            motor3.setPower(Math.abs(Lspeed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor2.isBusy() && motor3.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor2.setPower(0);
            motor3.setPower(0);

            // Turn off RUN_TO_POSITION
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }
}
