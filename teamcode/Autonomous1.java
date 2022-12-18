/* Copyright (c) 2021 FIRST. All rights reserved.
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


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Autonomous 1", group="Autonomous")
public class Autonomous1 extends LinearOpMode {


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        AutonomousRobotActions robot = new AutonomousRobotActions();

        DcMotor greenMotor = hardwareMap.get (DcMotor.class, "greenMotor");
        DcMotor blackMotor = hardwareMap.get (DcMotor.class, "blackMotor");
        DcMotor armMotor = hardwareMap.get (DcMotor.class, "armMotor");

        Servo rightHandServo = hardwareMap.get (Servo.class, "rightHandServo");
        Servo leftHandServo = hardwareMap.get (Servo.class, "leftHandServo");



        DigitalChannel centerTouchSensor  = hardwareMap.get (DigitalChannel.class, "centerTouchSensor");

        boolean codeHasRun = false;


        //region telemetry
        // Wait for the game to start (driver presses PLAY)
        resetRuntime();
        waitForStart();
        telemetry.update();
        //endregion
        if (opModeInInit()){

            greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            centerTouchSensor.setMode(DigitalChannel.Mode.INPUT);

            greenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // run until the end of the match (driver presses STOP)
        if(opModeIsActive()) {
            robot.moveForward(5300, greenMotor, blackMotor);
            greenMotor.setPower(1.0);
            blackMotor.setPower(1.0);
            while(opModeIsActive() && greenMotor.isBusy()){
                idle();
            }

            robot.turnLeft(700 , greenMotor, blackMotor);
            greenMotor.setPower(0.8);
            blackMotor.setPower(1.0);
            greenMotor.setPower(1.0);
            while (opModeIsActive() && greenMotor.isBusy()){
                idle();
            }

            robot.moveForward(500, greenMotor, blackMotor);
            greenMotor.setPower(0.8);
            blackMotor.setPower(1.0);
            greenMotor.setPower(1.0);
            while (opModeIsActive() && greenMotor.isBusy()){
                idle();
            }



            telemetry.update();
            greenMotor.setPower(0);
            blackMotor.setPower(0);
        }
//        stop();
//        else if (opModeIsActive() && codeHasRun){
//            stop();
//        }

        while (opModeIsActive())
        {
            if (gamepad1.b || gamepad2.b){
                stop();
            }
        }


    }

    public void moveForward(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor) {
        _greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);

        telemetry.addData("Acton executed", "Forward " + duration + " at " + power + " power");
    }

    public void moveBackwards(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor) throws InterruptedException {
        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);

        telemetry.addData("Acton executed", "Backwards " + duration + " at " + power + " power");
    }

    public void turnLeft(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor){

        _greenMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _blackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);

        telemetry.addData("Acton executed", "Turned Left " + duration + " at " + power + " power");
    }

    public void turnRight(double power, int duration, DcMotor _greenMotor, DcMotor _blackMotor) throws InterruptedException {
        _greenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _blackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _greenMotor.setPower(power);
        _blackMotor.setPower(power);

        _greenMotor.setTargetPosition(duration);
        _blackMotor.setTargetPosition(duration);

        _greenMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _blackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _greenMotor.setPower(0);
        _blackMotor.setPower(0);

        telemetry.addData("Acton executed", "Turn Right " + duration + " at " + power + " power");
    }

    public void armUp(double power, int duration, DcMotor _armMotor) throws InterruptedException {
        _armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _armMotor.setPower(power);

        _armMotor.setTargetPosition(duration);

        _armMotor.setPower(0);

        telemetry.addData("Acton executed", "Arm Up " + duration + " at " + power + " power");
    }

    public void armDown(double power, int duration, DcMotor _armMotor) throws InterruptedException {
        _armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _armMotor.setPower(power);

        _armMotor.setTargetPosition(duration);

        _armMotor.setPower(0);

        telemetry.addData("Acton executed", "Arm Down " + duration + " at " + power + " power");
    }

    public void closeHand(Servo _rightHandServo, Servo _leftHandServo) throws InterruptedException{

    }



    public void stop(DcMotor _greenMotor, DcMotor _blackMotor) {
        _greenMotor.setPower(0);
        _blackMotor.setPower(0);

        telemetry.addData("Acton executed", "Stopped Drive Motors");
    }

    public void stopAll(DcMotor _greenMotor, DcMotor _blackMotor, DcMotor _armMotor, Servo _leftHandServo, Servo _rightHandServo){
        _greenMotor.setPower(0);
        _blackMotor.setPower(0);
        _armMotor.setPower(0);

        telemetry.addData("Acton executed", "Stopped All Motors");
    }
    


}
