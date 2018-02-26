# ANDROID CAR DUINO

This is the source code for an Android app called "Carduino".  The app runs on an Android device such as a mobile phone attached to the car.  It is the brains of the autonomous car, controlling how it drives by sending steering and acceleration commands to the car via bluetooth.  The app uses information from the car sensors (received via bluetooth), as well as from the mobile phone camera, to take observations from the environment.  The code implements an OODA loop (observe, orient, decide, act), common in robotics, to go from observations to actions.

Instructions for building and running the app are on the [wiki](https://github.com/sesstigit/Android-Car-duino/wiki) of this repository.

The repository from the original authors is [here](https://github.com/Petroula/Android-Car-duino), and an article about their autonomous car is titled [The world's first Autonomous android vehicle](https://platis.solutions/blog/2015/06/29/worlds-first-android-autonomous-vehicle/).

![Alt text](car_architecture_androidapp.png?raw=true "You are now looking at the android app")


