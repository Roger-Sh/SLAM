# SLAM - Simultaneous localization and mapping

## Introduction 

这份笔记是针对汉诺威大学Claus Brenner教授的课程SLAM 整理而来，笔记内的某些专业词汇可能是德语，该笔记仅供个人学习使用。
该课程分为以下几个模块，课程所需的编程环境为python2.7. 在github页面正确显示公式需要安装一个插件: 
[MathJax Plugin for Github](https://chrome.google.com/webstore/detail/mathjax-plugin-for-github/ioemnmodlmafdkllaclgeombjnmnbima)

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

<div align=center><img src="https://i.imgur.com/0uQWKGA.png" width="600px" /> </div>
<div align=center> Fig A-1 Lego Car </div>

<div align=center><img src="https://i.imgur.com/DBH8449.png" width="600px" /> </div>
<div align=center> Fig A-2 Arena and Landmark </div>

### 2. Motor Control 

下图展示了乐高小车的运动模型，由两个电机带动左右履带，履带的位移差距可以让小车进行转向。

<div align=center><img src="https://i.imgur.com/YErhk8h.png" width="600px" /> </div>
<div align=center> Fig A-3 Motion </div>

小车的编码器可以表示电机的角度，经过换算，1 tick 约为 0,349 mm 履带前进的距离。下图表示了一个电机的log-file中的内容。

<div align=center><img src="https://i.imgur.com/lMD8jBt.png" width="600px" /> </div>
<div align=center> Fig A-4 Motor log file </div>

其中，第0列 'M' 表示Motor, 第1列表示时间戳 (time stamp), 第2列表示左轮Motor的编码， 第6列表示右轮Motor的编码，第10列表示第三个motor的编码，这里我们只需要左轮和右轮的motor即可。slam\_01\_a， slam\_01\_b，slam\_01\_c 这三个程序展示了如何通过lego\_robot 这个class来读取robot4\_motors.txt 。

下一步是建立小车转弯时的运动模型。

<div align=center><img src="https://i.imgur.com/p6utG7D.png" width="600px" /> </div>
<div align=center> Fig A-5 Motion Model</div>

$$ 
\begin{align}
 r &= \alpha \cdot (R + w) \nonumber \\\\ 
 l &= \alpha \cdot R \nonumber \\\\
 r - l &= \alpha \cdot w \nonumber \\\\
 \alpha &= (r-l)/w \nonumber \\\\
 R &= l/\alpha \nonumber
\end{align}
\tag{1.1}
$$

