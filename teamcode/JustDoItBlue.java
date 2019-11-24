package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="JustDoItBlue", group ="Concept")

public class JustDoItBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront_drive = null;
    private DcMotor leftFront_drive = null;
    private DcMotor leftBack_drive = null;
    private DcMotor rightBack_drive = null;
    //Servo2 is the grabber
    private Servo Servo2 = null;
    //Servo1 is the arm
    private Servo Servo1 = null;
    //Servo0 is the extender
    private Servo Servo0 = null;
    //next two are foundation hooks
    private Servo Servo3 = null;
    private Servo Servo4 = null;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;


    @Override public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        rightFront_drive = hardwareMap.get(DcMotor.class, "rightFront_drive");
        leftFront_drive = hardwareMap.get(DcMotor.class, "leftFront_drive");
        leftBack_drive = hardwareMap.get(DcMotor.class, "leftBack_drive");
        rightBack_drive = hardwareMap.get(DcMotor.class, "rightBack_drive");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo0 = hardwareMap.get(Servo.class, "Servo0");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");
        rightFront_drive.setDirection(DcMotor.Direction.REVERSE);
        leftFront_drive.setDirection(DcMotor.Direction.FORWARD);
        leftBack_drive.setDirection(DcMotor.Direction.FORWARD);
        rightBack_drive.setDirection(DcMotor.Direction.REVERSE);
        //reset all servos
        Servo3.setPosition(0.52);
        Servo4.setPosition(0.25);
        Servo2.setPosition(1);
        waitForStart();
        Servo1.setPosition(0.95);
        //move forward
        forward(0.75);
        waitfor(0.91667);
        brakemotors();
        //grab
        Servo2.setPosition(0);
        waitfor(0.75);
        //retract arm
        Servo1.setPosition(0.05);
        waitfor(1.5);
        //strafe
        strafeleft(2.5);
        waitfor(2.75);
        //push forward
        forward(1);
        waitfor(0.5);
        brakemotors();
        //grab foundation
        Servo3.setPosition(0.3);
        Servo4.setPosition(0.8);
        waitfor(0.5);
        //pull back
        forward(-1);
        waitfor(2);
        brakemotors();
        //place block
        Servo1.setPosition(1);
        waitfor(0.5);
        Servo2.setPosition(1);
        //ungrab
        Servo3.setPosition(0.52);
        Servo4.setPosition(0.25);
        waitfor(1);
        Servo1.setPosition(0.05);
        waitfor(1.5);
        //wheeeeeeeee to parking spot and do parallel parking
        straferight(0.5);
        waitfor(3.5);
        brakemotors();
    }
    private void forward(double power){
        rightFront_drive.setPower(power);
        leftFront_drive.setPower(power);
        leftBack_drive.setPower(power);
        rightBack_drive.setPower(power);

    }
    private void straferight(double power){
        rightFront_drive.setPower(-power);
        leftFront_drive.setPower(power);
        leftBack_drive.setPower(-power);
        rightBack_drive.setPower(power);

    }
    private void strafeleft(double power){
        rightFront_drive.setPower(power);
        leftFront_drive.setPower(-power);
        leftBack_drive.setPower(power);
        rightBack_drive.setPower(-power);

    }
    private void brakemotors(){
        leftFront_drive.setPower(0);
        rightFront_drive.setPower(0);
        leftBack_drive.setPower(0);
        rightBack_drive.setPower(0);

    }
    private void waitfor(double waittime){
        runtime.reset();
        while (runtime.time() < waittime){

        }

    }

}
