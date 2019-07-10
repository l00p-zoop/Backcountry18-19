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
@Autonomous(name = "Depot Auto", group = "Concept")
//@Disabled
public class New_New_Depot extends LinearOpMode {
    //Setting up vuforia and TensorFlow
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUKN4dL/////AAABmZL4EHQ+ukIzpu89viYzaR4/jtXdbGjrssYSj0jYOGUszrxyjIZgyw6bnlbe3u5cMKxWpBP4i8HPWkAKoqmaJmRHrv8Ho7vPazLYJ65ZoLevtugk9OihzarMGR1FSjcThK3Tx3k2zpgCwaZWGopNS4ObXr5mFqdZkLmE11hEj7QV/sX886rFKB6Q9ySF1L4eYhwZmsArJjf5BGgenPwg0/Ou8yhB3VXk5++EAbLQwwlSFYZnQkS8h9TeGHe7LL5CbPgYptx+qo8iLJqqx6cqkabRHyVv1rzTxeJw42wFpHAono6FcMrWOiuKZs2xBiKKlUlw2qI8cl6sHbhiRPcOtLO3X5p7f0+fInRqOY4pakdS";

    //Establish constants

    private VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();

    private TFObjectDetector tfod;

    private DistanceSensor sensorRange;

    private DriveUtilities drive;
    //Declare Motors
    DcMotor  liftmotor, rotatemotor;
    Servo hookservo, mservo;
    //Establishing Constants
    boolean chkloop = true;
    boolean drploop = true;

    @Override
    public void runOpMode() {

        //Initialize motors - the rest is done in the DriveUtilities class
        drive = new DriveUtilities(hardwareMap, this);
        liftmotor = hardwareMap.get(DcMotor.class, "lm");
        rotatemotor = hardwareMap.get(DcMotor.class, "rm");

        sensorRange = hardwareMap.get(DistanceSensor.class, "dst");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        mservo = hardwareMap.get(Servo.class,"ms");
        hookservo = hardwareMap.get(Servo.class, "hs");

        //Set encoder modes

        initVuforia();

        //Initialize vuforia and tensorflow
        mservo.setPosition(0.05);
        hookservo.setPosition(-.1);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        drive.logPositions();
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        //Start of the autonomous
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            runtime.reset();
            while (chkloop) {
                //Locate the position of the gold mineral
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() != 0) {
                        int goldMineralX = -1;

                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1) {
                          if(goldMineralX > 0 && goldMineralX < 150){
                              //If the gold mineral is in the left position
                              telemetry.addData("Position = ", "Left");
                              telemetry.update();
                              //Starts left position function
                              leftmove();
                              chkloop = false;
                          } else if(goldMineralX >= 150 && goldMineralX <= 475){
                              //Identifies if the gold mineral is in the canter position
                              telemetry.addData("Position = ", "Center");
                              telemetry.update();
                              //calls the center position function
                              centermove();
                              chkloop = false;
                          } else if(goldMineralX > 475){
                              //If the gold mineral is in the right position
                              telemetry.addData("Position = ", "Right");
                              telemetry.update();
                              //Calls the right position function
                              rightmove();
                              chkloop = false;

                          }
                        }else if (runtime.seconds() >= 10){
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
        //Drops the robot
        droprobot();
        telemetry.addData("Movement", " Done");
        telemetry.update();
        //Push the particle
        drive.driveFB(.6,33,3);
        drive.driveFB(.6,-20,3);
        drive.driveLR(.6,14,5);
        //Function that moves robot to crater
        finalToCrater();


    }
    public void leftmove(){
        //Drop the robot
        droprobot();
        //Push the particle
        drive.driveFB(.6,12,3);
        drive.driveLR(.6,14,5);
        drive.driveFB(.6,22,3);
        drive.driveFB(.6,-22,3);
        //Function that moves robot to crater
        finalToCrater();

    }
    public void rightmove(){
        //Drop the robot
        droprobot();
        //Push the particle
        drive.driveFB(.6,12,3);
        drive.driveLR(.6,-11,4);
        drive.driveFB(.6,12,3);
        drive.driveFB(.6,-13.5,3);
        drive.driveLR(.6,27,6);
        //Function that moves robot to crater
        finalToCrater();

        
    }
    public void finalToCrater(){
        telemetry.addData("Final", "Moving");
        telemetry.update();
        //Rotate robot and arm
        drive.rotate(34);
        drive.rotatemotor.setPower(-.4);
        sleep(1500);
        drive.rotatemotor.setPower(0);
        //move into the depot
        drive.driveLR(.6,22,4);
        drive.driveFB(.6,30,5);
        //Drop the team marker
        mservo.setPosition(.6);
        sleep(200);
        //Reverse into the crater
        drive.driveFB(.6,-56,6);


    }




        //Function to drop the robot
    public void droprobot(){
        liftmotor.setPower(.2);
        // motor2.setPower(.15);
        while (drploop) {
            if (sensorRange.getDistance(DistanceUnit.CM) < 4.3) {
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

        drive.motor1.setPower(-.15);
        drive.motor4.setPower(.15);
        sleep(300);//strafe out from the lander just a tiny bit
        drive.motor1.setPower(0);
        drive.motor4.setPower(0);
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

}
