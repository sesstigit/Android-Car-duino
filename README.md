# ANDROID CAR DUINO

This is the source code for an Android app called "Carduino".  The app runs on a device such as a mobile phone attached to the car.  The app is the brains of the autonomous car, controlling how it drives.  It sends commands to the car via bluetooth to control steering and acceleration.  The app uses information from the car sensors (received via bluetooth), as well as from the mobile phone camera, to take observations from the environment.  The code in this appimplements an OODA loop (observe, orient, decide, act), common in robotics, to go from observations to actions.

The repository from the original authors is [here], and an article about their autonomous car is titled [The world's first Autonomous android vehicle](https://platis.solutions/blog/2015/06/29/worlds-first-android-autonomous-vehicle/).

![Alt text](Diagram.png?raw=true "You are now looking at the android app")

Instructions for building and running the app are on the [wiki](https://github.com/sesstigit/Android-Car-duino/wiki) of this repository.

