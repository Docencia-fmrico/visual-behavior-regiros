# Visual Behavior Regiros

## Creators:
Marina Wiesenberg Bustillo <br />
Juan Miguel Valverde García <br />
Daniel Quinga López <br />
David Duro Aragonés <br />

## Index
   - Global Objetives
   - Follow Ball 
      - Introduction 
      - Objetive 
      - What have we done? 
         - Some problems 
         - Behavior Tree
      - Working in simulator
   - Follow Person 
      - Introduction 
      - Objetive 
      - What have we done? 
         - Behavior Tree
      - Working in simulator
   - Follow with priority 
      - Introduction 
      - Objetive 
      - What have we done? 
         - Behavior Tree
      - Working in simulator
   - Implementations 
   
## 0. Global Objetives: 
The main objetive of this project is to make a robot capable of following a ball, a person and both at the same time.
   
## 1. Follow Ball

### 1.0. Introduction
   The implementation of this program is based on a behavior tree, which changes its state depending on whether it detects a ball or not. To locate the ball, it is mainly used a color-filtered image.
   
### 1.1. Objetive
   Our main objective was to create a condition program called ifball, which in addition to detecting the ball, would also filter its image to return the speed corresponding to the distance and rotation of the object. To do so, several means have been used.
   
### 1.2. What have we done?
   The ifball program returns true if it detects the ball and returns false if it does not. Now, not only does it return true or false, but also, the distance and rotation of the ball using the filtered image. With the help of a PID we are able to transmit a speed according to the distance, as well as an angular depending of its rotation. These speeds taken from the PID are passed to the move program, which simply sends the kobuki the order to move with that speed. If ifball returns false, it would also return an angular velocity so that the kobuki rotates on itself until the ball is detected.
   
#### 1.2.1. Some problems
   When launching the program in the simulator it worked perfectly, since the filtering of the image was super sharp and without noise. On the other hand, when launching the program in the kobuki and performing the filtering, appears enough noise that the program itself could interpret as the position of the ball. To correct this, by filtering the image and seeing a pixel that could be the ball, we do kind of a focus. We take a square of the image from the detected pixel and filter that piece, if we count more than x pixels that may belong to the ball, so we return the corresponding speed. It is worth mentioning that this does not always work because the color filtering is not perfect and having objects of a similar color to the ball's can lead to erratic readings.

#### 1.2.2. Behavior Tree
   The behavior tree used for this implementatios is the following one:

   ![Follow ball BT](https://i.postimg.cc/qvHpRFxf/bt-ifball.png "Follow ball BT")

## 2. Follow Person
   
### 2.0. Introduction
   The implementation of this program is based on a behavior tree, which changes state depending on whether it detects a person. To locate the person, it is mainly used bounding boxes.

### 2.1. Objetive
   Our main objective was to create a condition program called ifperson, which in order to detect a person, it would filter its bounding box and return the speed corresponding to the distance and rotation of it. To do so, severals means have been used.
   
### 2.2. What have we done?
   In a similiar manner to that used in the follow ball program, ifperson binds two images, one with the bounding boxes, and the other one, capable of getting the depth of the object, therefore, its distance to the camera. Checking if the bounding box detected is that of a person and getting its middle pixel, we can finds the distance of that pixel and its position. With that information and adjusting it with the PID we get the linear and angular speed needed to approach the person.

#### 2.2.1. Behavior Tree
   The behavior tree used for this implementatios is the following one:

   ![Follow Person BT](https://i.postimg.cc/3JvK3hpC/bt-if-person.png "Follow person BT")

### 2.3. Working in simulator
   [Operation](https://urjc-my.sharepoint.com/:v:/g/personal/da_quinga_2020_alumnos_urjc_es/Ed7FduVtF61FijoWPuvrN4UBaSExEh2OFEv1WyQdnvhRlA?e=ohRvtC)
   
## 3. Follow with priority
   
### 3.0. Introduction
   The implementation of this program is based on a simple behavior tree, which changes state depending on whether it detects a ball or a person. The behavior tree is similar to follow ball or follow person, the principal difference is the priority implementation, it may sound difficult, but its very easy, in the conditional parte of our tree, if it detects the ball it returns the speeds according to its distance, but on the other hand if it does not detect it, it goes to look for the person, if it detects it would do the same as the ball, this would be repeated infinitely times until it detects one or the other.
   
### 3.1. Objetive
   Our main objective was to create a node capable of working with both ifball and ifperson programs, with the implementation of another Behavior Tree, in order to be able to follow a ball or a person at the same time. 

### 3.2. What have we done?
   We have created a new behavior tree that uses the same nodes as the previous ones, thanks to how we did ifball and ifperson the implementation of both programs is simple. It is just needed a program for the ros node that register the behavior tree and the nodes and ticks the root.
   
#### 3.2.1. Behavior Tree
   The behavior tree used for this implementatios is the following one:

   ![Follow Both BT](https://i.postimg.cc/vmgRtLz7/followboth-bt.png "Follow both BT")
   
### 3.3. Working in simulator
   [Operation](https://urjc-my.sharepoint.com/:v:/g/personal/da_quinga_2020_alumnos_urjc_es/ETjmVZh4gMpKjt-Cqy7RXS4B224AEJzS1rOufhIQg5uI5g?e=FfZimJ)

## 4. Implementations
   Finally we have created a visual-behavior.yaml configuration file, to be able to change basic parameters such as kobuki speed, turning distance, etc.
