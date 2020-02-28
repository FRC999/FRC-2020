# Meeting Minutes  2/22/20

## Intake

All subsystems + commands have been written
Split into three parts (all victors):
Intake (bit that extends) has a pneumatics to deploy + 1 motor to pull
Magazine holds ammo: has 1 motor
Loader: shoves ammo from magazine into shooter 2 motor
**TODO:**

* Shuffleboard assignments PETER is on this

## Shooter

Still needs some parts
**MOTORS**
Has "shooter" motor that fires (A miniSim) (*will* be a Victor)
Pan than rotates the turret: A 775 (w/ more magic encoder)
Tilt that adjusts range (red fangs): A Johnson motor (w/ magic encoder) (code written in ColorWheel)
**TODO**

* Get turret parameters: CALUM is working in this
  * Zero point location on Falcon chassis
  * Full rotation counter
  * Full throw of johnson motor
* Test calum's zero adjustment code

## Climber

All commands have been written: all motors and required system are setup in the subsystem
HOWEVER: Brake is not yet extant

## DriveTrain

Written
**TODO**

* Test the triple class rewrite
* FalconBot pid parameters
* Test left wall following
* Test right wall following: PETER and THE DOCTOR are doing this

## Control Panel

Has not been examined for a while: Evaluate
THE DOCTOR is working on this

## Vision Tracking

Currently: we have detection
We need to train it on a proper target
BILLY and JACK are on this
