# SLAM - Simultaneous localization and mapping

## Introduction 

这份笔记是针对汉诺威大学Claus Brenner教授的课程SLAM 整理而来，笔记内的某些专业词汇可能是德语，该笔记仅供个人学习使用。课程所需的编程环境为python2.7. 在github页面正确显示公式需要安装一个插件: 
[MathJax Plugin for Github](https://chrome.google.com/webstore/detail/mathjax-plugin-for-github/ioemnmodlmafdkllaclgeombjnmnbima),
教程视频： [SLAM Lectures](https://www.youtube.com/playlist?list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm)，Repo：[SLAM](https://github.com/ipa-fog-ws/SLAM)，该课程分为以下几个模块：

- Unit A: Roboter, Sensor, Arena,Landmark, Motion Model, 
- Unit B: LS-Square, correkt the trajektory, Iterative-Closest-Point
- Unit C: Filtering, Probability-Distribution, Base-Filter, Kalmann-Filter (One-Dimention)
- Unit D: Multivariate Normal Distribution, Kalmann-Filter (Muilti-Dimention),  Extended-Kalmann-Filter(Non-Linear)
- Unit E: Particle Filter
- Unit F: Mapping, Extended-Kalmann-Filter SLAM
- Unit G: Particle Filter SLAM 

## Unit A 

Unit A 的内容是对一个乐高小车的运动进行建模，首先利用乐高小车的电机编码器返回的数据计算出小车的运动轨迹，其次利用乐高小车搭载的LiDAR传感器获得周围环境的深度信息，检测出Landmark所在的位置。

<div align=center><img src="https://i.imgur.com/0uQWKGA.png" width="400px" /> </div>
<div align=center> Fig A-1 Lego Car </div>
<div align=center><img src="https://i.imgur.com/DBH8449.png" width="400px" /> </div>
<div align=center> Fig A-2 Arena and Landmark </div>
下图展示了乐高小车的运动模型，由两个电机带动左右履带，履带的位移差距可以让小车进行转向。

<div align=center><img src="https://i.imgur.com/YErhk8h.png" width="400px" /> </div>
<div align=center> Fig A-3 Motion </div>
小车的编码器可以表示电机的角度，经过换算，1 tick 约为 0,349 mm 履带前进的距离。下图表示了一个电机的log-file中的内容。

<div align=center><img src="https://i.imgur.com/lMD8jBt.png" width="400px" /> </div>
<div align=center> Fig A-4 Motor log file </div>
其中，第0列 'M' 表示Motor, 第1列表示时间戳 (time stamp), 第2列表示左轮Motor的编码， 第6列表示右轮Motor的编码，第10列表示第三个motor的编码，这里我们只需要左轮和右轮的motor即可。slam\_01\_a， slam\_01\_b，slam\_01\_c 这三个程序展示了如何通过lego\_robot 这个class来读取robot4\_motors.txt 。

下一步是建立小车转弯时的运动模型。

<div align=center><img src="https://i.imgur.com/p6utG7D.png" width="400px" /> </div>
<div align=center> Fig A-5 Motion Model</div>
由下列公式可以得到小车绕中心点旋转时的旋转半径 \\(R\\) 和旋转的角度 \\(\alpha\\)：

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

p点和p'点之间的坐标关系可以通过中心点c联系起来：

<div align=center><img src="https://i.imgur.com/t2V9Qr2.png" width="400px" /> </div>
<div align=center> Fig A-5 Motion Model</div>
c点和p‘的坐标为：

$$
\begin{align}
\vec{c} &= \vec{p} - (R+\frac{w}{2}) \cdot \begin{bmatrix} \sin{\theta} \\\\ -\cos{\theta}\end{bmatrix} \\\\
\vec{p'} &= \vec{c} + (R + \frac{w}{2}) \cdot \begin{bmatrix} \sin{\theta + \alpha} \\\\ -\cos{\theta + \alpha}\end{bmatrix}
\end{align}
\tag{1.2}
$$

slam\_02\_a 展示了如何通过读取乐高小车的电机logfile，将其转换成前进距离，并利用上述公式计算出下一时刻的位置。由于传感器LiDAR 的位置并不是小车轮子中心的位置，所以对读取的位置减去displacement (30mm)，计算好新的位置之后，再将displacement加上。slam\_02\_b 展示了相关操作，并将过滤过的位置输出到 poses\_from\_ticks.txt。

<div align=center><img src="https://i.imgur.com/9ZCm8qe.png" width="300px" /> </div>
<div align=center> Fig A-6 Displacement</div>
logfile\_viewer.py 可以将 poses\_from\_ticks.txt 可视化。 

<div align=center><img src="https://i.imgur.com/fxJ9nVi.png" width="400px" /> </div>
<div align=center> Fig A-7 logfile_view</div>
在可视化窗口中加入robot4\_reference.txt，可以观察到小车和参考轨迹之间的差距，这是由于小车的宽度参数并不是实际参数，通过修改robot\_width这一变量，可以调整计算得来的轨迹精度。

<div align=center><img src="https://i.imgur.com/6m6u3x9.png" width="600px" /> </div>
<div align=center> Fig A-7 logfile_view, left: robot_width=150, right: robot_width=171</div>
下一步是引入传感器LiDAR得到的深度信息，以此来确定Arena中的Landmark位置。在可视化窗口载入robot4\_scan.txt, 可以观察到小车的传感器信息，

<div align=center><img src="https://i.imgur.com/5NyxAfq.png" width="800px" /> </div>
<div align=center> Fig A-8 left: robot4\_scan.txt, middle: 可视化窗口, right: 单次时间点读取的数据</div>
为了得到Landmark的数量，我们要对传感器数据进行微分，得到单次时间不同角度之间的差值 (derivative)，通过计数每次传感器数值的突变来得到Landmark的数量：

$$
f'(i) \approx \frac{f(i+1)-f(i-1)}{2} \tag{1.3}
$$

从上图右侧窗口可知，在某些角度，比如400附近，产生了一个传感器误差，此时传感器的值小于20，在程序slam\_03\_b我们对这样的数值进行过滤。最终得到该图像的微分图像：

<div align=center><img src="https://i.imgur.com/G93N2pT.png" width="400px" /> </div>
<div align=center> Fig A-9 传感器LiDAR的数据及其微分</div>
在程序slam\_03\_c中，通过对Landmark左边的edge和右边的edge进行计数，可以得到Landmark的数量，通过对左edge和右edge之间的数值求平均值，可以得到该Landmark相对于传感器的角度和深度信息，以此，我们可以确定该Landmark的位置。

<div align=center><img src="https://i.imgur.com/36qq7A7.png" width="400px" /> </div>
<div align=center> Fig A-10 Landmark 相对于LiDAR的极坐标</div>
在程序slam\_03\_d中，我们将Landmark的位置从相对LiDAR的极坐标位置转换成笛卡尔坐标系位置，并考虑到传感器接触Landmark表面与Lanmark中心点之间的差值大约为90mm, 最终结果输出到可视化窗口后如图，小绿点为Landmark通过建模估计得到的位置，灰点为Landmark实际位置：

<div align=center><img src="https://i.imgur.com/usofPRd.png" width="400px" /> </div>
<div align=center> Fig A-11 Landmark 相对于LiDAR的笛卡尔坐标</div>