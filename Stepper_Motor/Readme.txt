In this project

STM32F030xx is interfaced with Stepper Motor. 
A rotate function has been created which will take parameters: RPM, No_of_Steps, Direction as an input.

The function will give exact output for RPMs:
10, 20, 30, 40, 50, 60, 80, 90, 100, 120, 150, 160, 180, 200

Use the frequency as 48MHz

/*
 * Configurations:
 * Enable Timer 3 and Timer 6
 * Timer will generate PWM Pulses and Timer 6 will count the number of PWM pulses as an Interrupt
 */

/*
	 Logic:

		1 min - 200 rotations
		1 sec - 200 / 60;

		Stpper Motor Calculations:

		Stepping Rate = (Step Counts/ 60) * Desired RPM		// Step Counts = 200
				  = (200/60)* 200				// RPM = 200
				  = 666.67 Hz

		Stepping Time = 1/Stepping Rate = 1.5 msecs			// delay = 1.5 * 10^(-3) inbtw 2 PWM pulses

		=> 200 RPM  == 666.67 Hz == or 1.5 msec PWM pulses
		for x RPM   == (666.67/x)*200 == or x/ (1.5 msec * 200)  PWM Pulses = delay

		Timer formula: (ARR + 1)*(PSC + 1) = (frequency)*(delay)*(CNT + 1)

		(ARR + 1)*(PSC + 1) = ((frequency)*(delay)*200) / (Required RPM)
		delay = ((48 * 10^6) * (1.5 * 10*(-3)) * 200) / (Required RPM)
			  = 14400000/ RPM

*/


This library is made to ease from HAL_Delay Concept.