# **ADAS Features For Saving Driver**

## **Overview**

This project implements advanced driver-assistance systems (ADAS) to enhance vehicle safety by integrating various features such as Lane Keeping Assist (LKAS), Adaptive Cruise Control (ACC), Lane Collision Warning (LCW), and Automatic Emergency Braking (AEB). The system aims to improve the driving experience by helping prevent accidents, ensuring smoother driving, and reducing the likelihood of human errors on the road.

## **Features**
Adaptive Cruise Control (ACC): Maintains a safe following distance from the car ahead by automatically adjusting the vehicle's speed.
Lane Keeping Assist System (LKAS): Steers the vehicle back into the lane if it drifts unintentionally.
Lane Collision Warning (LCW): Alerts the driver if the vehicle is at risk of leaving its lane.
Automatic Emergency Braking (AEB): Detects obstacles in front and applies the brakes if necessary to prevent collisions.
Speed Control Based on Distance: Automatically adjusts the vehicle’s speed depending on the proximity to other vehicles or obstacles.

## **Pin Configuration**
TRIG_PIN: GPIO_PIN_9 (for triggering the ultrasonic sensor)
ECHO_PIN: GPIO_PIN_8 (for receiving the echo from the ultrasonic sensor)

## **Constants**
### Speed Constants
1- Stop_Distance: 15 cm
2- Low_Distance: 30 cm
3- Medium_Distance: 40 cm
4- High_Distance: 50 cm
5- ADLowSpeed: 200
6- ADMediumSpeed: 250
7- ADHighSpeed: 300
8- LowSpeed: 100
9- MediumSpeed: 200
10- HighSpeed: 350
11- VeryHighSpeed: 400
12- TSpeed: 300
12- RLSpeed: 350
12- LimitSpeed: 300
### Control Constants
- TURN_ON_AUTOPILOT ('O')
- TURN_OFF_AUTOPILOT ('o')
- TURN_ON_ACC ('D')
- TURN_OFF_ACC ('d')
- TURN_ON_LKAS ('K')
- TURN_OFF_LKAS ('k')
- TURN_ON_AEB ('A')
- TURN_OFF_AEB ('a')
- TURN_ON_LCW ('C')
- TURN_OFF_LCW ('c')

## **Key Functions**
### Speed Control
- Car_vSpeed(uint16_t Distance): Adjusts the vehicle's speed based on the distance measured by the ultrasonic sensor.
- Car_vFroward(), Car_vStop(), Car_vReverse(), Car_vTRight(), Car_vTLeft(): Functions to control the vehicle's movement in different directions (forward, reverse, turning left or right).

### **Movement Control**
- Move_vForward(), Move_vReverse(): Move the vehicle forward or reverse with the designated speed.
- Move_vRight(), Move_vLeft(): Move the vehicle to the right or left.
- Move_vRF(), Move_vLF(): Move the vehicle forward at a right or left angle.
- Move_vRR(), Move_vLR(): Move the vehicle in reverse with a right or left angle.

## **Autonomous Features**
- Auto_vPilot(uint16_t distance_car): Main function that activates the autopilot, including ACC, LKAS, and AEB.
- ACC(uint16_t distance_car): Adaptive Cruise Control - adjusts the car's speed based on distance to the car ahead.
- LKAS(): Lane Keeping Assist System - corrects the car's steering to stay within the lane.
- LCW(): Lane Collision Warning - warns and adjusts the vehicle’s position if lane departure is detected.
- AEB(uint16_t distance_car): Automatic Emergency Braking - activates the brakes to prevent a collision if the distance is too short.

## **Ultrasonic Sensor**
- ULTRASONIC_u16GetDistance(): Measures the distance to nearby objects using an ultrasonic sensor to adjust speed or trigger braking.

## **How It Works**
- The ADAS system operates in real-time by continuously monitoring the vehicle’s surroundings and making automatic adjustments to speed, steering, and braking. The vehicle uses an ultrasonic sensor to detect nearby obstacles and other vehicles. Based on this data, it adjusts the vehicle’s speed using Adaptive Cruise Control (ACC), keeps the vehicle within its lane using Lane Keeping Assist (LKAS), warns the driver with Lane Collision Warning (LCW), and automatically applies the brakes if an obstacle is detected too close using Automatic Emergency Braking (AEB).

## *Main System Features*
- Autopilot: The car can be put in autopilot mode, where it actively controls speed, steering, and braking to maintain safety and comfort.
- Adaptive Cruise Control: When activated, the system adjusts the car's speed based on the distance to the vehicle ahead, maintaining a safe gap.
- Lane Keeping Assist: If the system detects unintended lane departure, it steers the vehicle back into the lane.
- Lane Collision Warning: Warns the driver if the vehicle is drifting out of the lane without signaling.
- Automatic Emergency Braking: Automatically applies the brakes if an obstacle is detected too close.

## **UART Commands for Control**
- O: Turn on Autopilot.
- o: Turn off Autopilot.
- K: Activate Lane Keeping Assist System (LKAS).
- k: Deactivate Lane Keeping Assist System (LKAS).
- D: Activate Adaptive Cruise Control (ACC).
- d: Deactivate Adaptive Cruise Control (ACC).
- A: Activate Automatic Emergency Braking (AEB).
- a: Deactivate Automatic Emergency Braking (AEB).
- C: Activate Lane Collision Warning (LCW).
- c: Deactivate Lane Collision Warning (LCW).
- F: Move forward.
- B: Move in reverse.
- L: Turn left.
- R: Turn right.
- W: Move right forward.
- Y: Move left forward.
- r: Move right in reverse.
- X: Move left in reverse.

## **System Initialization**
The system initializes by configuring necessary peripherals (GPIO, Timer, UART) and then enters a continuous loop to monitor the environment and respond to incoming commands.

## **Conclusion**
The ADAS Features For Saving Driver project enhances driver safety by providing advanced driving assistance functions. These features, including adaptive cruise control, lane keeping, and automatic braking, aim to reduce the risk of accidents and ensure a safer driving experience.
