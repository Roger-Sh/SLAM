# SLAM - Simultaneous localization and mapping

## Introduction 

这份笔记是针对汉诺威大学Claus Brenner教授的课程SLAM 整理而来，笔记内的某些专业词汇可能是德语，该笔记仅供个人学习使用。
该课程分为以下几个模块，课程所需的编程环境为python2.7.

- Unit A: Roboter, Sensor, Arena,Landmark, Motion Model, 
- Unit B: LS-Square, correkt the trajektory, Iterative-Closest-Point
- Unit C: Filtering, Probability-Distribution, Base-Filter, Kalmann-Filter (One-Dimention)
- Unit D: Multivariate Normal Distribution, Kalmann-Filter (Muilti-Dimention),  Extended-Kalmann-Filter(Non-Linear)
- Unit E: Particle Filter
- Unit F: Mapping, Extended-Kalmann-Filter SLAM
- Unit G: Particle Filter SLAM 

## Unit A 

Unit A 的内容是对一个乐高小车的运动进行建模，首先利用乐高小车的电机编码器返回的数据计算出小车的运动轨迹，其次利用乐高小车搭载的LiDAR传感器获得周围环境的深度信息，检测出Landmark所在的位置。

### 1. Lego Car and Landmark

Fig A-1 Lego Car:

<img src="https://i.imgur.com/0uQWKGA.png" width="500px" />

Fig A-2 Arena and Landmark:

<img src="https://i.imgur.com/DBH8449.png" width="500px" />


### 2. Motor Control 

下图展示了乐高小车的运动模型，由两个电机带动左右履带，履带的位移差距可以让小车进行转向。

Fig A-3 Motion:

<img src="https://i.imgur.com/YErhk8h.png" width="500px" />

小车的编码器可以实时显示出
