import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.remote.ev3.RMISampleProvider;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

import java.awt.Color;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
// Eric Xu 730095427
// Loi Pham 730095087

@SuppressWarnings("unused")

public class circleObstacle {

	public static void main(String[] args) {

		EV3MediumRegulatedMotor motorA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor motorB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3MediumRegulatedMotor motorC = new EV3MediumRegulatedMotor(MotorPort.C);
		EV3TouchSensor touchsensor1 = new EV3TouchSensor(SensorPort.S1);
		EV3TouchSensor touchsensor2 = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S3);
		
		EV3ColorSensor cs = new EV3ColorSensor(SensorPort.S4);
		//SampleProvider colorprovider = cs.getColorIDMode();
		//float[] colorsample = new float[colorprovider.sampleSize()];
		SensorMode touch = touchsensor1.getTouchMode();
		SensorMode touch2 = touchsensor2.getTouchMode();
		SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode(); // start ultrasonic
		float[] sample_sonic = new float[sonic.sampleSize()];
		   
	    motorA.synchronizeWith(new EV3MediumRegulatedMotor[] {motorB}); //sync tire speed
	    motorA.setSpeed(500);
	    motorB.setSpeed(500);
	   
	    //START
	    Button.waitForAnyPress();
	    
		float[] sample_touch = new float[touch.sampleSize()];
		float[] sample_touch2 = new float[touch2.sampleSize()]; 
	    while((sample_touch[0] != 1) && (sample_touch2[0] != 1)) //move forward until touch
	    {
	      motorA.forward();
	      motorB.forward();
	      touch.fetchSample(sample_touch,0);
	      touch2.fetchSample(sample_touch2,0);
	    }
	    
	    int starttohit = motorA.getTachoCount()-267; //distance from start to hitpoint
	    System.out.println(starttohit);
	    
	    motorA.setSpeed(600);
	    motorB.setSpeed(600);
	    motorA.stop(true);
	    motorB.stop(true);
	    motorA.resetTachoCount();
	    motorB.resetTachoCount();
	    //beep
	    Sound.beepSequenceUp();
	    Sound.beepSequence();
	    //move backwards 20cm - 7.62cm
	    while(motorA.getTachoCount() > -267){
		    motorA.backward();
		    motorB.backward();
	    }
	    
	    int left = motorB.getTachoCount(); //position of right wheel at 20cm
	    int right = motorA.getTachoCount(); //position of left wheel at 20cm
	    
	    motorA.resetTachoCount();
	    motorB.resetTachoCount();
	    motorA.stop(true);
	    motorB.stop(true);
	    
	    //rotate 360 to right and move forward 7in or 17.78cm
	    motorA.rotate(360);
	    motorB.rotate(-240);
	    motorA.resetTachoCount();
	    motorB.resetTachoCount();
	    motorA.setSpeed(500);
	    motorB.setSpeed(500);
	    while(motorA.getTachoCount() < 370)
	    {
	    	motorA.forward();
	    	motorB.forward();
	    }
	    motorA.stop(true);
	    motorB.stop(true);
	    motorC.setSpeed(200);
	    motorC.rotate(90);
	    
	    
	    //wall following
	    SampleProvider colorprovider = cs.getColorIDMode();
	  	float[] colorsample = new float[colorprovider.sampleSize()];
	  	while(colorsample[0] != 2)
	    {
	  		motorA.forward();
	    	motorB.forward();
	    	
			
	    	System.out.println("Sonic: " + sample_sonic[0]);
	    	sonic.fetchSample(sample_sonic, 0);
	    	
	    	if(sample_sonic[0] < 0.08)
	    	{
	    		motorA.setSpeed(100);
	    		motorB.setSpeed(40);
	    	}
	    	if(sample_sonic[0] > 0.08 && sample_sonic[0] < 0.1)
	    	{
	    		motorA.setSpeed(100);
	    		motorB.setSpeed(100);
	    	}
	    	if(sample_sonic[0] > 0.1)
	    	{
	    		motorB.setSpeed(80);
	    		motorA.setSpeed(40);
	    	}
	    	colorprovider.fetchSample(colorsample, 0);
	    	
	    }
	    
	  	motorA.setSpeed(600);
	  	motorB.setSpeed(600);
	    motorA.stop(true);
	    motorB.stop(true);
	    motorA.resetTachoCount();
	    motorB.resetTachoCount();
	    
	    // turn right
	    motorB.rotate(-200);
	    motorA.rotate(360);
	    
	    // move same distance as beginning
	    while (motorA.getTachoCount() < starttohit)
	    {
	    	motorA.forward();
		    motorB.forward();
	    }
	    //color detection
	    motorA.stop(true);
	    motorB.stop(true);
	    motorA.close();
	    motorB.close();
	    motorC.close();
	    touchsensor1.close();
	    touchsensor2.close();
	    ultrasensor.close();
	}
}