package com.qualcomm.ftcrobotcontroller.opmodes.customcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

public class ETAutonomousBase extends LinearOpMode {

  public static final double DIST_FACTOR = 3.4;
  public static final double MOVE_FIRST_DISTANCE = 100 / DIST_FACTOR;
  public static final double MOVE_POWER = -0.25;
  public static final double MOVE_SECOND_DISTANCE = 24 / DIST_FACTOR;
  public static final double REVERSE_FIRST_DISTANCE = -27 / DIST_FACTOR;

  final static int ENCODER_CPR = 1440;            // Encoder counts per revolution
  final static double GEAR_RATIO = 1;             // Gear ratio
  final static double WHEEL_DIAMETER = 2.625;     // Diameter of wheel
  final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
  final static int ERROR_THRESHOLD = 10;
  public static final int TURN_DELAY = 800;

  // Declare hardware devices
  protected DcMotor right;
  protected DcMotor left;
  protected Servo mainRaise;
  protected Servo climberL;
  protected Servo climberR;

  // Variables
  protected static boolean isTargetSet = false;
  protected static double counts = 0;
  protected static int stage;
  protected boolean loopBreaker;

  // CONTROL STAGES
  final static int STAGE_MOVE_FIRST = 1;
  final static int STAGE_MOVE_SECOND = 2;
  final static int STAGE_TURN_FIRST = 3;
  //final static int STAGE_TURN_SECOND = 4;
  final static int STAGE_REVERSE_FIRST = 5;
  final static int STAGE_STOP = 100;

  public void etInit() throws InterruptedException {
    right = hardwareMap.dcMotor.get("Right");
    left = hardwareMap.dcMotor.get("Left");
    left.setDirection(DcMotor.Direction.REVERSE);
    mainRaise = hardwareMap.servo.get("MainRaise");
    climberR = hardwareMap.servo.get("ClimberR");
    climberL = hardwareMap.servo.get("ClimberL");
    climberL.setDirection(Servo.Direction.REVERSE);
    mainRaise.setPosition(0.52);
    climberR.setPosition(0);
    climberL.setPosition(0);

  }
  public void etSetup() throws InterruptedException {

    // Reset all variables to initial value
    mainRaise.setPosition(0);
    // Stage = first stage
    stage = STAGE_MOVE_FIRST;
    // Loop is not broken
    loopBreaker = false;
    // Target is not set
    isTargetSet = false;
    // Counts is 0
    counts = 0;
    waitOneFullHardwareCycle();
  }
  public void etLoop() throws InterruptedException {
    
  }

  public void etBreakLoop() throws InterruptedException {
    loopBreaker = true;
  }

  @Override
  public void runOpMode()  throws InterruptedException {
    etInit();
    waitForStart();
    etSetup();
    //int i = 0;
    while(opModeIsActive()) {
      //telemetry.addData("etLoop ", i++);
      etLoop();
      if(loopBreaker) {
        break;
      }
      waitOneFullHardwareCycle();
    }
    //this.stop();
  }

  protected double getCountsForDistance(double distance) {
    double rotations = distance / CIRCUMFERENCE;
    return ENCODER_CPR * rotations * GEAR_RATIO;
  }

  protected void turn(double rightPower, double leftPower) throws InterruptedException {
    right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    waitOneFullHardwareCycle();
    right.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    left.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    right.setPower(rightPower);
    left.setPower(leftPower);
    telemetry.addData("Started Turning leftSetPower0, power:", left.getPower());
    sleep(TURN_DELAY);

  }

  protected boolean hasArrived(double cts) {
    //telemetry.addData("cts", cts);
    telemetry.addData("Left Position", left.getCurrentPosition());
    telemetry.addData("Right Position", right.getCurrentPosition());
    telemetry.addData("cts", cts);
    double absCts = Math.abs(cts);


    int rightErrorMargin = Math.abs(Math.abs(right.getCurrentPosition()) - (int) absCts);
    int leftErrorMargin = Math.abs(Math.abs(right.getCurrentPosition()) - (int) absCts);
    if (leftErrorMargin < ERROR_THRESHOLD && rightErrorMargin < ERROR_THRESHOLD) {
      isTargetSet = false;
      return true;
    }
    return false;

  }

  protected void stopRobot() throws InterruptedException {
    right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    waitOneFullHardwareCycle();
    etBreakLoop();
  }

  protected synchronized void  setTarget(double power) throws InterruptedException {
    if(isTargetSet)
      return;

    //telemetry.addData("setTarget ", distance);

    //double cts = getCountsForDistance(distance);
    right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    waitOneFullHardwareCycle();
    telemetry.addData("Reset Encoders", "Done");

    right.setTargetPosition((int) counts);
    left.setTargetPosition((int) counts);
    right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    right.setPower(power);
    left.setPower(power);
    isTargetSet = true;
    //return cts;
  }
}