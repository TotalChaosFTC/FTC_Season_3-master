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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SDV: Telop Tank", group="SDV")
//@Disabled
public class SDVTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor right;
    public DcMotor left;
    public Servo grab1;
    public Servo grab2;
    public Servo grab3;
    public Servo grab4;
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {
        double leftJoy;
        double rightJoy;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYydEH3/////AAAAGXMATUMRIE4Pv8w0T+lHxs5Vah12gKSD60BBnydPYF3GeoUEUBpr9Q4NXikGwa+wLuElb3hZH2ujmFnni6yudqsshk91NxEEeOBZBscu60T3JbZVW05gvgAbxrAQgQRbMomuW3rFL/KhLVeOL+pb0k0DJEAsgTcoL7dahj1z/9tfrZC0vFDIW4qXsnzmjXRyT1MWXc8odL8npQI+FJZoyh8gpfGs6iuY6ZCi+QkjdlRIpsZnozIPCN5S9K1Zv8/3CnOmBz50I7x+fiZM9Soj3jbShvKQyfHRMTYX4b1DAspwJ6ekaU10UxtUeijN2pjfRv8jE857LRDmrBsuO6YBrlI9C49idhYLXADg8DlegTq4 ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //left  = hardwareMap.get(DcMotor.class, "left");
        //right = hardwareMap.get(DcMotor.class, "right");
        grab1 = hardwareMap.get(Servo.class, "1");
        grab2 = hardwareMap.get(Servo.class, "2");
        grab3 = hardwareMap.get(Servo.class, "3");
        grab4 = hardwareMap.get(Servo.class, "4");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)


            double position = 0;
            leftJoy = gamepad1.left_stick_y/10;
            rightJoy = gamepad1.right_stick_y/10;
            left.setPower(leftJoy);
            right.setPower(rightJoy);
            /*grab1.setPosition(-position);
            grab2.setPosition(position);
            grab3.setPosition(-position);
            grab4.setPosition(position);
             if(gamepad1.x){
                 position = position + 20;
                grab1.setPosition(-position);
                grab2.setPosition(position);
                grab3.setPosition(-position);
                grab4.setPosition(position);
             }
            else if(gamepad1.b){
                position = position - 20;
                grab1.setPosition(-position);
                grab2.setPosition(position);
                grab3.setPosition(-position);
                grab4.setPosition(position);
            }*/

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.addData("Vu Foria has gotten a value","");
                telemetry.update();

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

            }
            else {
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }

            telemetry.addData("left",  "%.2f", leftJoy);
            telemetry.addData("right", "%.2f", rightJoy);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
        }
    }
}
