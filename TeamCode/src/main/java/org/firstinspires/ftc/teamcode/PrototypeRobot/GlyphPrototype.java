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

package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name="GlyphPrototype", group="SDV")
//@Disabled
public class GlyphPrototype extends OpMode {

    /* Declare OpMode members. */
    public DcMotor glyphleft;
    public DcMotor glyphright;
   // public DcMotor rb;
    //public DcMotor lb;
    //public DcMotor gr;
    //public DcMotor gl;
   // public Servo jk;
    //public Servo grab2;
    public Servo rgservo;
    public Servo lgservo;
    double rservoposition = 0;
    double lservoposition = 0;



    @Override
    public void init() {
        glyphleft  = hardwareMap.get(DcMotor.class, "gl");
        glyphright = hardwareMap.get(DcMotor.class, "gr");
        //lb = hardwareMap.get(DcMotor.class, "lb");
       // rb = hardwareMap.get(DcMotor.class, "rb");
        rgservo = hardwareMap.get(Servo.class , "rgs");
        lgservo = hardwareMap.get(Servo.class , "lgs");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        rgservo.setPosition(0);
        lgservo.setPosition(0);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        double leftJoy;
        double rightJoy;

        double position = 0;
        leftJoy = gamepad1.left_stick_y*0.5;
        rightJoy = gamepad1.right_stick_y*0.5;
        lf.setPower(leftJoy);
        lb.setPower(leftJoy);
        rf.setPower(rightJoy);
        rb.setPower(rightJoy);
        */

        if(gamepad1.right_bumper){
            glyphright.setPower(0.5);
        }
        else if(gamepad1.left_bumper){
            glyphleft.setPower(0.5);
        }
        else{
            glyphright.setPower(0.0);
            glyphleft.setPower(0.0);;
        }

        if(gamepad1.right_trigger > 0){
            rservoposition = rservoposition + 0.1;
            rgservo.setPosition(rservoposition);
            lservoposition = lservoposition -0.1;
            lgservo.setPosition(lservoposition);
        }
        else if (gamepad1.left_trigger > 0){
            rservoposition = rservoposition - 0.1;
            rgservo.setPosition(rservoposition);
            lservoposition = lservoposition + 0.1;
            lgservo.setPosition(lservoposition);
        }
        telemetry.update();

        // Pause for 40 mS each cycle = update 25 times a second.
    }

    @Override
    public void stop() {
    }
}
