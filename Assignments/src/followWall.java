import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.Sound;
import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
// Eric Xu 730095427
// Loi Pham 730095087

@SuppressWarnings("unused")

public class followWall
{
    
  public static void main(String[] args)
  {
	  
    EV3MediumRegulatedMotor motorA = new EV3MediumRegulatedMotor(MotorPort.A);
    EV3MediumRegulatedMotor motorB = new EV3MediumRegulatedMotor(MotorPort.B);
    EV3MediumRegulatedMotor motorC = new EV3MediumRegulatedMotor(MotorPort.C);
	EV3TouchSensor touchsensor1 = new EV3TouchSensor(SensorPort.S1);
	EV3TouchSensor touchsensor2 = new EV3TouchSensor(SensorPort.S2);
	EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S3);
	
	SensorMode touch = touchsensor1.getTouchMode();
	SensorMode touch2 = touchsensor2.getTouchMode();
	
	SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode(); // start ultrasonic
	
	
	float[] sample_sonic = new float[sonic.sampleSize()];
	
	   
    motorA.synchronizeWith(new EV3MediumRegulatedMotor[] {motorB}); //sync tire speed
    
    motorA.setSpeed(500);
    motorB.setSpeed(500);
   
    Button.waitForAnyPress();
    
    
	float[] sample_touch = new float[touch.sampleSize()];
	float[] sample_touch2 = new float[touch2.sampleSize()]; //move forward until touch
	
    while((sample_touch[0] != 1) && (sample_touch2[0] != 1)) 
    {
      motorA.forward();
      motorB.forward();
      touch.fetchSample(sample_touch,0);
      touch2.fetchSample(sample_touch2,0);
      
    }
    

    motorA.setSpeed(300);
    motorB.setSpeed(300);
    
    motorA.stop(true);
    motorB.stop(true);
    
    motorA.resetTachoCount();
    
    while(motorA.getTachoCount() > -270) // go back 
    {
	    motorA.backward();
	    motorB.backward();
    }
    
    motorA.resetTachoCount();
    motorB.resetTachoCount();
    
    motorA.stop(true);
    motorB.stop(true);
    
    try {
		Thread.sleep(1000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    
    motorA.rotate(360);
    //motorB.rotate(-720,true);
    

    //turn 
    
    motorA.setSpeed(100);
    motorB.setSpeed(100);
    

    while(sample_sonic[0] < 1)
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
    }
    
    motorA.resetTachoCount();
    motorB.resetTachoCount();
    
	motorB.setSpeed(200);
	motorA.setSpeed(200);
	
	
	Delay.msDelay(600);
    motorA.stop(true);
    motorB.stop(true);
    
    try {
		Thread.sleep(1000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    
    motorB.rotate(60);
    while(motorA.getTachoCount() < 1505)
    {
    	motorA.forward();
    	motorB.forward();
    }
    //keep moving within this range until you can't measure anymore
    
    
    
    /*
    motorA.stop(true);
    motorB.stop(true);
	Sound.beep();
    Button.waitForAnyPress();
	    */
    
    
	//Run until the center/enter button is pressed
    
  
    motorA.close();
    motorB.close();
    touchsensor1.close();
    touchsensor2.close();
    ultrasensor.close();
    
}
}