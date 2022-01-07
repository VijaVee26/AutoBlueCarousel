/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;


@Autonomous(name="Auto(BLUE): Drop Carousel BLUE", group="COMPETITION")

public class AutoCompetitionDropCarouselBlue extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareAlphaIntelli robot   = new HardwareAlphaIntelli();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    // set the count per motor rotation
    static final double     TETRIX_CPMR   = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     ANDY_CPMR     = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     goBilda_CPMR  = 537 ;    // eg: TETRIX Motor Encoder

    //set the drive speed
    static final double     TURN_SPEED              = 0.3;
    double     DRIVE_SPEED             = 0.5;
    public ModernRoboticsI2cColorSensor colorSensor = null;
    @Override

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        
        robot.lFrontDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        robot.lBackDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        robot.rFrontDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        robot.rBackDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.lFrontDrive.getCurrentPosition(),
                robot.lBackDrive.getCurrentPosition(),
                robot.rFrontDrive.getCurrentPosition());
        robot.rBackDrive.getCurrentPosition();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.dump.setPower(robot.DUMP_FRWD_POSN_MIN);
        //Shipping hub
        encoderDrive(DRIVE_SPEED, "T", 3.4, "B", 25, 100000);
        encoderDrive(DRIVE_SPEED, "T", 3.4, "R", 17, 100000);
        encoderDrive(DRIVE_SPEED, "T", 3.4, "B", 3.5, 100000);
        encoderSlideUpDwn(DRIVE_SPEED, "T", 2.0, "UP", 17, 100000);
        telemetry.addData("Path", "Moving to Dump");
        telemetry.update();
        sleep(1000);
        robot.dump.setPower(0.5);
        //robot.dump.wait();
        sleep(2500);
        robot.dump.setPower(robot.DUMP_FRWD_POSN_MIN);
        //robot.dump.wait();
        sleep(2500);

        // turn towards carousel
        if (opModeIsActive()) robot.rotateToHeading(140, 0.4);

        // drive to the carousel
        if (opModeIsActive()) robot.driveDistance(0.4, -39);

        // rotate carousel until duck comes off
        if (opModeIsActive()) carousel.move(Carousel.BLUE);
        sleep(2500);
        if (opModeIsActive()) carousel.stopMotors();

        // back up to clear the carousel
        if (opModeIsActive()) robot.driveDistance(0.4, 30);

        // turn 45 degrees left
        if (opModeIsActive()) robot.rotateToHeading(90, 0.2);

        // drive forward to the box
        if (opModeIsActive()) robot.driveDistance(0.4, -24);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Pace this loop so jaw action is reasonable speed.
            sleep(2500);
        }
    }
}

