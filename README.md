# CARLA Ego Vehicle - Simulation to Xlsx

# Summary
 - [Introduction](#Introduction)
 - [Goal](#Goal)
 - [Requirements](#Requirements)
   - [Installed software](#Installed-software)
     - [CARLA](#CARLA)
     - [Python3](#Python3) 
     - [Pip3](#Pip3)
   - [Libraries](#Libraries)
     - [Pandas](#Pandas)
     - [Requirements.txt](#Requirements.txt)
 - [How to use the software](#How-to-use-the-software)
 - [What can be improved](#What-can-be-improved)
 - [Conclusion](#Conclusion)

# Introduction

- Internship work for the conclusion of my Software Application Development course at [TESP](http://www.ipg.pt/website/candidatos_tesp_geral.aspx)
- This project was developed during my professional internship at [Capgemini Engineering](https://capgemini-engineering.com/pt/pt-pt/), where I had the opportunity to participate in the V2X initiative at the Embedded and Software Critical Systems unit.

# Goal 

The goals of this project are:
- To be able to control a car in CARLA manually and to be able to visualize in the car's view over the other cars a box to delimit the cars and their corresponding identification;
- Calculate the distance to the other cars in the simulation;
- Calculate other data ("angle,PE,etc");
- Generate at the end of the simulation an Excel file with all the data of the vehicles separately and graphics of the paths traveled by them.

# Requirements

The steps below were used with Ubuntu 20.04

## Installed software

### CARLA

In order to run the software it is necessary to have CARLA 0.9.11 installed and the simulator open.

- [CARLA version 0.9.11](https://github.com/carla-simulator/carla/releases/tag/0.9.11)
- [How to run CARLA](https://carla.readthedocs.io/en/latest/start_quickstart/)

### Python3.7

Update OS repositories

```
$ sudo apt update
```

Add new repository
```
$ sudo apt install software-properties-common
$ sudo add-apt-repository ppa:deadsnakes/ppa
```

Install Python 3.7
```
$ sudo apt-get install python3.7
```

### Pip3

Install Pip3
```
$ sudo apt-get -y install python3-pip
```

## Libraries

### Pandas

Install Pandas
```
$ sudo apt install python3.7-distutils
$ python3.7 -m pip install -U pandas --user
```

### Requirements.txt

 1. Get the "requirements.txt" file from the repository.
 2. Inside the CARLA folder you downloaded go to the folder /PythonAPI/examples and paste the "requirements.txt" file there.
 3. Right mouse click inside folder and click "Open in terminal".
 4. Paste the following code and execute it.

```
$ python3.7 -m pip install -r requirements.txt
```

# How to use the software

 1. Go to CARLA folder you have downloaded.
 2. Right mouse click inside folder and click "Open in terminal".
 3. Run CARLA simulator typing the next code.
 ```
 $ ./CarlaUE4.sh
 ```
 4. Inside the CARLA folder you downloaded go to the folder /PythonAPI/examples and paste the file "mymc.py" there.
 5. Right mouse click inside folder and click "Open in terminal".
 6. Paste the following code to execute the software.
 ```
 $ python3.7 mymc.py
 ```
 7. When the software opens you have access to manual control of one car while the second car is on autopilot.
 8. To control the car use the keys (W,A,S,D) and (Q) to change the gear.
 9. You can see a box delimiting the other cars and the ID of the cars on top of them.
 10. When you want to end the execution just click on the ESC key.
 11. At the end of the execution a ZIP will be generated with all the information from both cars that can be used in the project [CARLA-2DCarsPath](https://github.com/FranciscoG001/CARLA-2DCarsPath)

# What can be improved

 - In the future the car bounding box can be adapted to the size of each car.
 - The string that identifies the cars can always be on top of the car in the pygame view.

# Conclusion

This software emerged as part of Capgemini's V2X initiative project, with the ultimate goal of providing data from two cars during an execution in CARLA, to later be used in projects already carried out by other project members.

In this project I had the help and collaboration of [Frederico Martins](https://github.com/fredpedroso), [Daniel Sader](https://github.com/danielpontello) and [Rédi Vildo]() from Capgemini.
