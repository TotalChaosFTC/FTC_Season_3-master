package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.util.Vector;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public abstract class NewBaseAutoOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armTwist;
    Servo jewelKnock;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;
    private ElapsedTime runtime = new ElapsedTime();
    //navx values
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = -45.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    ColorSensor color;
    Vector<Step> steps;
    Step currentStep;
    int counts = 0;
    int currentStepIndex;
    final static int ATREST = 0;
    final static int WAITFORRESETENCODERS = 1;
    final static int WAITFORCOUNTS = 2;
    final static int FINISHED = 3;
    final static int DETECTCOLOR = 4;
    final static int WAITFORTURN = 5;
    final static int TURNTOANGLE = 6;
    final static int JEWELKNOCK = 7;
    final static int VUCHECK = 8;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 3.6;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    //Start step types
    final static int MOVE = 1;
    final static int RIGHT = 2;
    final static int LEFT = 3;
    final static int RANGE = 4;
    final static int WAITFORTOUCH = 5;
    final static int BACK = 6;
    final static int BLUE = 7;
    final static int RED = 8;
    final static int WAIT = 9;
    final static int MOVEARM = 10;
    final static int FORWARD = 11;
    final static int BACKWARD = 12;
    final static int NONE = 13;
    final static int VUFORIA = 14;
    //
    final static int BLOCKN = 0;
    final static int BLOCKC = 1;
    final static int BLOCKL = 2;
    final static int BLOCKR = 3;
    int block = BLOCKN;
    int counter = 0;

    public class Step {
        public double distance;
        public double leftPower;
        public double rightPower;
        public int rightCounts;
        public int leftCounts;
        public int sweeperDirection;
        public double armPosition;
        public double turnAngle;
        public int sType;
        public Step(double dist, double left, double right, int stepType, int direction, int angle) {
            distance = dist;
            sType = stepType;
            if (stepType == MOVE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = right;
            }
            else if (stepType == RIGHT) {
                turnAngle = angle;
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = right;

            }
            else if(stepType == LEFT){
                turnAngle = angle;
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = -right;
            }
            else if(stepType == BACK){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == BLUE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == RED){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == WAIT){
                leftPower = 0;
                rightPower = 0;
            }
            else if (stepType == MOVEARM){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = right;
            }
            else{
                armPosition = dist;
            }
            sweeperDirection = direction;
        }

        public int convertDistance(double distance){
            double  rotations = distance / CIRCUMFERENCE;
            double counts = ENCODER_CPR * rotations * GEAR_RATIO;
            return (int) counts;
        }
    }

    public void initializeNavX( double angle){

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setSetpoint(angle);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        navx_device.zeroYaw();

    }
    public void init()
    {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        color = hardwareMap.colorSensor.get("cs");
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("navx"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        jewelKnock =  hardwareMap.servo.get("jk");
        steps = new Vector<Step>();
        initSteps();
        currentStep = steps.get(0);
        currentStepIndex = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYydEH3/////AAAAGXMATUMRIE4Pv8w0T+lHxs5Vah12gKSD60BBnydPYF3GeoUEUBpr9Q4NXikGwa+wLuElb3hZH2ujmFnni6yudqsshk91NxEEeOBZBscu60T3JbZVW05gvgAbxrAQgQRbMomuW3rFL/KhLVeOL+pb0k0DJEAsgTcoL7dahj1z/9tfrZC0vFDIW4qXsnzmjXRyT1MWXc8odL8npQI+FJZoyh8gpfGs6iuY6ZCi+QkjdlRIpsZnozIPCN5S9K1Zv8/3CnOmBz50I7x+fiZM9Soj3jbShvKQyfHRMTYX4b1DAspwJ6ekaU10UxtUeijN2pjfRv8jE857LRDmrBsuO6YBrlI9C49idhYLXADg8DlegTq4 ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

    }
// Create all doubles for sensors ect..

    public abstract void initSteps();
    @Override
    public void loop(){
        if (state == ATREST){
            if(currentStep.sType == WAIT){
                if (counts == 400){
                    currentStepIndex = currentStepIndex + 1;
                    counts = 0;
                    if (currentStepIndex >= steps.size()) {
                        state = FINISHED;
                    } else {
                        currentStep = steps.get(currentStepIndex);
                        state = ATREST;

                    }
                }
                else{
                    counts = counts+1;                }
            }
            else if (currentStep.sType ==  MOVE || currentStep.sType == BACK){
                resetEncoders();
                state = WAITFORRESETENCODERS;
                initializeNavX(TARGET_ANGLE_DEGREES);

            }
            else if (currentStep.sType == RIGHT || currentStep.sType == LEFT){
                state = TURNTOANGLE;
                initializeNavX(currentStep.turnAngle);
                setMotorPower(currentStep.leftPower,currentStep.rightPower);


            }
            else if(currentStep.sType == MOVEARM){
                jewelKnock.setPosition(45);
                state = JEWELKNOCK;
            }
            else if(currentStep.sType == VUFORIA){
                state = VUCHECK;
            }
        }
        else if( state == WAITFORRESETENCODERS) {
            if (areEncodersReset()){
                setMotorPower(currentStep.leftPower, currentStep.rightPower);
                state = WAITFORCOUNTS;
            }
        }
        else if (state== WAITFORCOUNTS) {
            if (areCountsReached(currentStep.leftCounts, currentStep.rightCounts)) {
                setMotorPower(0, 0);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }
            }
            else {
                setMotorPower(currentStep.leftPower, currentStep.rightPower);
                navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
                try {
                    if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                        if ( yawPIDResult.isOnTarget() ) {
                            setMotorPower(currentStep.leftPower, currentStep.rightPower);

                        } else {
                            double output = yawPIDResult.getOutput();
                            if ( output < 0 ) {
                                setMotorPower(currentStep.leftPower - output, currentStep.rightPower + output);
                            } else {
                                setMotorPower(currentStep.leftPower + output, currentStep.rightPower - output);

                            }
                        }
                    }
                }
                catch(InterruptedException e){

                }

            }
        }
        else if (state == TURNTOANGLE){

            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            double yaw = navx_device.getYaw();
            if (Math.abs(yaw - currentStep.turnAngle) >= 2) {
                telemetry.addData("Current yaw: ", yaw);
                telemetry.update();
                try {
                    if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                        if (!yawPIDResult.isOnTarget() ) {
                            double output = yawPIDResult.getOutput();
                            if ( output < 0 ) {
                                setMotorPower(output, -output);
                            } else {
                                setMotorPower(-output,output);
                            }
                        }
                    } else {
                        Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    }
                } catch (InterruptedException e) {

                }
            }
            else {
                setMotorPower(0,0);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }
            }

        }
        else if (state == JEWELKNOCK){
            if (counter < 50){
                counter ++;
            }
            else {
                state = DETECTCOLOR;
            }
        }
        else if (state == DETECTCOLOR){
            if (color.red()>2){
                resetEncoders();
                state = WAITFORRESETENCODERS;
                initializeNavX(TARGET_ANGLE_DEGREES);
            }
            else if (color.blue()>2){
                resetEncoders();
                state = WAITFORRESETENCODERS;
                initializeNavX(TARGET_ANGLE_DEGREES);
                currentStep.distance = currentStep.distance *-1;
                currentStep.leftPower = currentStep.leftPower*-1;
                currentStep.rightPower = currentStep.rightPower*-1;
            }
        }
        else if(state ==  VUCHECK){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuForia is getting no readings","");
                telemetry.update();
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER){
                telemetry.addData("VuMark", "Center");
                telemetry.update();
                block = BLOCKC;
            }
            else if(vuMark ==  RelicRecoveryVuMark.LEFT){
                telemetry.addData("VuMark", "Left");
                telemetry.update();
                block = BLOCKL;
            }
            else if(vuMark ==  RelicRecoveryVuMark.RIGHT){
                telemetry.addData("VuMark", "Right");
                telemetry.update();
                block = BLOCKR;
            }
            telemetry.update();

        }
        else if (state == FINISHED){
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    @Override
    public void stop(){
        state = ATREST;
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public boolean areEncodersReset() {
        return leftFront.getCurrentPosition() == 0 &&
                rightFront.getCurrentPosition() == 0 &&
                leftBack.getCurrentPosition() == 0 &&
                rightBack.getCurrentPosition() == 0;
    }

    public boolean areCountsReached(int leftCounts, int rightCounts) {
        return ( Math.abs(leftFront.getCurrentPosition()) >= Math.abs(leftCounts) &&
                Math.abs(rightFront.getCurrentPosition()) >= Math.abs(rightCounts) &&
                Math.abs(leftBack.getCurrentPosition())>= Math.abs(leftCounts) &&
                Math.abs(rightBack.getCurrentPosition()) >= Math.abs(rightCounts) );

    }

    public void setMotorPower(double leftPower,double rightPower){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
    }
}