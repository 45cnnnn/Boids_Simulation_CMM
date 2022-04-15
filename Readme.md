## Assignment 3 - Boids

## Introduction

In this assignment, I implement a particle system for simulating the behavior of birds based on *Boids*[[Reynolds(1987)]](https://medium.com/swlh/boids-a-simple-way-to-simulate-how-birds-flock-in-processing-69057930c229). A short introduction from the author is available [online](https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html).

![figure: overview](videos/interface.png)
<p align="center">Figure 1: Overview</p>

As shown in Figure 1, different behaviors(*Freefall, Cohesion, Alignment, Separation, Leading*), integration schemes(*Explicit Euler, Symplectic Euler and Explicit midpoint) can be chosen from the interface. You can also set different parameters to test the behavior of  boids.

If the embedded videos can not be played well, you can check them in corresponding Youtube link or in the folder */videos.*

## Basic Time Integration
### [Freefall](https://youtu.be/qYt27zo-Buo)
Boids are randomly initialized and then fall down because of gravity.
<iframe width="560" height="315" src="https://www.youtube.com/embed/qYt27zo-Buo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Circular Motion
In this case, circular motion are implemented to test different integration schemes.  When step size `h` is small, all three schemes perform well. As `h` getting bigger, **Explicit Euler** and **Midpoint** start to diverge, while **Symplectic Euler** remains stable.
<iframe width="560" height="315" src="https://www.youtube.com/embed/PwRrlLuJ37k" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Flocking
### Cohesion
Boids have the tendency to stay close to their neighbors. There is an attraction force makes the bird move toward the average position of neighboring birds. Cohesion range can be set with the slider-bar.
<iframe width="560" height="315" src="https://www.youtube.com/embed/Dmw8gu9sAZ4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
### Alignment
In addition to stay close to the neighbors, each birds also want to match the average direction of other birds.
<iframe width="560" height="315" src="https://www.youtube.com/embed/0KBYlTj5tyA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
### Separation
As we can see, when "Cohesion" and "Alignment" are both implemented, all birds will finally converge into one point. So we need to separate the birds if they are too close to each other to avoid collision.
<iframe width="560" height="315" src="https://www.youtube.com/embed/8rFneyfA9W4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
### Collision Avoidance
In this case, a collision avoidance strategy is implemented to avoid obstacle. When the bird flies towards the obstacle and the distance from the obstacle is less than a threshold, it starts to dodge. This threshold can be different from (usually smaller than) "cohesion/alignment range", because birds can communicate by smell, birdsong or something with their peers, but not with the obstacle. If the bird's velocity is too fast, it will hit the obstacle.
<iframe width="560" height="315" src="https://www.youtube.com/embed/Yx_ezrE5MsI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Collaborative and Adversarial Behaviors
### Leader Following
All red birds all folloing the leader green birds (which is control by mouse). There is a maximun distance from witch the bird can follow the leader. This distance ususally larger than "cohesion/alignment range", since the leader is more attractive. The attractive force of the leader is inversely proportional to the distance.

<iframe width="560" height="315" src="https://www.youtube.com/embed/KCzpANXjRC0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>