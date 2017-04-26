/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/

@TeleOp(name="UltraUpgradedTeleOp2Players", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class UltraUpgradedTeleOp2Players extends LinearOpMode
{
    DcMotor wheelR;
    DcMotor wheelL;
    DcMotor intake;
    DcMotor launcher;
    Servo intakeServo;
    Servo beaconPresser;
    Servo sensorServo;
    LightSensor legoLightSensor;      // Primary LEGO Light sensor,
    OpticalDistanceSensor lightSensor;   // Alternative MR ODS sensor
    UltrasonicSensor rangeSensor;
    LegacyModule board;
    ElapsedTime runtime;
    double launcherPower = -0.34;
    boolean buttonOn = false;

    @Override
    public void runOpMode() {
        wheelR = hardwareMap.dcMotor.get("wheelR");
        wheelL = hardwareMap.dcMotor.get("wheelL");
        intake = hardwareMap.dcMotor.get("launcher");
        launcher = hardwareMap.dcMotor.get("intake");
        intakeServo =  hardwareMap.servo.get("servo_1");
        beaconPresser = hardwareMap.servo.get("servo_2");
        sensorServo = hardwareMap.servo.get("servo_3");
        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction


        board = hardwareMap.legacyModule.get("Legacy Module 1");
        board.enable9v(4, true);

        rangeSensor = hardwareMap.ultrasonicSensor.get("sensor_ultrasonic");

        runtime = new ElapsedTime();

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        legoLightSensor = hardwareMap.lightSensor.get("sensor_light");                // Primary LEGO Light Sensor
        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        waitForStart();

        sensorServo.setPosition(0);
        beaconPresser.setPosition(1);

        intake.setPower(0.5);
        //wheels
        while (opModeIsActive()) {

            check();
            //shooter
            if (gamepad2.a) {
                launcher.setPower(launcherPower);
                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 3) {
                    check();
                }

                intakeServo.setPosition(1);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 2) {
                    check();
                }
                wheelL.setPower(0);
                wheelR.setPower(0);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 3) {
                    check();
                }
                intakeServo.setPosition(0);
                launcher.setPower(0);
            }
        }
    }
    public void check() {
        float throttle = -gamepad1.left_stick_y;
        float direction = -gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        wheelR.setPower(right);
        wheelL.setPower(left);

        if (gamepad1.dpad_left) {
            beaconPresser.setPosition(0.02);
        }
        if (gamepad1.dpad_right) {
            beaconPresser.setPosition(0.98);
        }
        if (gamepad2.b && !buttonOn) {
            launcherPower += 0.01;
            buttonOn = true;
        }
        if (gamepad2.x && !buttonOn) {
            launcherPower -= 0.01;
            buttonOn = true;
        }
        if (!gamepad2.b && !gamepad2.x) {
            buttonOn = false;
        }
        telemetry.addData("launcher power: ", -launcherPower);
        telemetry.addData("light sensor measurement:", legoLightSensor.getLightDetected());
        telemetry.addData("ods measurement:", lightSensor.getLightDetected());
        telemetry.addData("ultrasonic measurement: ", rangeSensor.getUltrasonicLevel());
        telemetry.addData("sensor servo position: ", sensorServo.getPosition());
        telemetry.update();
    }
}
