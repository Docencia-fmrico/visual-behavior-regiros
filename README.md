[[_TOC_]]
# Visual Behavior Regiros

## Creators:
Marina <surname> <br />
Juan Miguel Valverde ... <br />
Daniel Quinga ... <br />
David Duro Aragonés <br />

## Index
   Global Objetives
   Follow Ball
      Introduction
      Objetive
      What we have done?
   Follow Person
      Introduction
      Objetive
      What we have done?
   Follow with priority
      Introduction
      Objetive
      What we have done?
   Implementations
   
## 0. Global Objetives: 
The main objetive of this project is to make a robot capa
   
## 1. Follow Ball

### 1.0. Introduction
   The implementation of this program is based on a behavior tree, which changes state depending on whether it detects a ball. To locate the ball, it is mainly used      in a color filter.
   
### 1.1. Objetive
   Our main objective was to create a conditioning program called ifball, which in addition to detecting the ball, would also filter its image to return the speed corresponding to the distance and rotation of it. To do this, various means have been used.
   
### 1.2. What we have done?
   The ifball program returns true if it detects the ball and returns false if it does not detect it. Now, not only does it return true or false, but when filtering to detect the ball it also gets the distance and rotation of it. With the help of a PID we are able to transmit a speed according to the distance, in addition to an angular speed according to its rotation. These speeds taken from the PID are passed to the move program, which simply sends the kobuki to move with that speed. If ifball returned false, in addition, it would return an angular velocity so that the kobuki rotates on itself until the ball is detected.
   
#### 1.2.1 Some problems
   When launching the program in the simulator it worked perfectly, since the filtering of the image was super sharp and without noise. On the other hand, when launching the program in the kobuki and performing the filtering, noise was sneaked in that the program itself could interpret as the position of the ball. To correct this, by filtering the image and seeing a pixel that could be the ball, we do a kind of focus. We take a square of the image from the detected pixel and filter that piece, if we count more than x pixels that may belong to the ball, we return the corresponding speed. It should also be added that this does not always work because the color filtering is not perfect and having objects of the same color as the ball can lead to failures.
   
## 2. Follow Person
   
### 2.0. Introduction
   The implementation of this program is based on a behavior tree, which changes state depending on whether it detects a person. To locate the person, it is mainly used bounding boxes.

### 2.1. Objetive
   Our main objective was to create a conditioning program called ifperson, which in addition to detecting a person, would also filter its bounding box to return the speed corresponding to the distance and rotation of it. To do this, various means have been used.
   
### 2.2. What we have done?
   
   
## 3. Follow with priority
   
### 3.0. Introduction

### 3.1. Objetive

### 3.2. What we have done?
   
   
## 4. Implementations
   Finally we have created a visual-behavior.yaml configuration file, to be able to change basic parameters such as kobuki speed, turning distance, etc.
