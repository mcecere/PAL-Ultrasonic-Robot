# PAL-Ultrasonic-Robot
// Personal Project to develop a motorized platform that will follow the user around using an ultrasonic beacon mounted on the user's remote control device, and turret based tracking on the mobile platform.
Based on a PIC32MX device, and a custom designed Ultrasonic driver constructed from Serial RS-232 Port driver, specifically Maxim232 device.
Uses 4 timers in the PIC to control:
timer 1: generates 40KHz Ultrasonic drive signal, of programmable frequency and also programable burst sizes. It can send one 40KHz pulse, or 3, or continuous.

timer 2: 3 channels of RC servoe control on a 20ms time schedule. used for mobile platform steering, platform speed control, and controlling position of ultrasonic direction finding turret servo.

timer 3: ADC acquisition control, 50uS for 50mS generating 1000 data points.

Timer 4: Main control loop @ 100ms. initiates US pulses, and powers overall state machine in PIC



