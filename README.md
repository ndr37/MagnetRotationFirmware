This code drives a custom rig to rotate a GMW c-frame magnet mounted on GMW's magnet rotation platform. The set up is straight forward, a timing belt is coupled to the magnet in one location and then serpentined through several posts around a stepper motor with a timing pulley. The stepper motor uses the DRV8825 stepper motor driver by Pololu. Stepper motor driver libraries by laurb9: <a href="https://github.com/laurb9/StepperDriver">StepperDriver</a>.

True angular position is carefully kept track of using a quadrature encoder coupled to the magnet with a rubber drive wheel. Prototype code for the quadrature encoder by <a href"http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino-with-solution/">Dr Rainer Hessmer</a>.

