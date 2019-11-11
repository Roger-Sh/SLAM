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
<div align=center> Fig A-3 Lego Car Structur</div>
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
<div align=center> Fig A-6 Motion Model</div>
c点和p‘的坐标为：

$$
\begin{align}
\vec{c} &= \vec{p} - (R+\frac{w}{2}) \cdot \begin{bmatrix} \sin{\theta} \\\\ -\cos{\theta}\end{bmatrix} \nonumber \\\\
\vec{p'} &= \vec{c} + (R + \frac{w}{2}) \cdot \begin{bmatrix} \sin{\theta + \alpha} \\\\ -\cos{\theta + \alpha}\end{bmatrix}\nonumber 
\end{align}
\tag{1.2}
$$

slam\_02\_a 展示了如何通过读取乐高小车的电机logfile，将其转换成前进距离，并利用上述公式计算出下一时刻的位置。由于传感器LiDAR 的位置并不是小车轮子中心的位置，所以对读取的位置减去displacement (30mm)，计算好新的位置之后，再将displacement加上。slam\_02\_b 展示了相关操作，并将过滤过的位置输出到 poses\_from\_ticks.txt。

<div align=center><img src="https://i.imgur.com/9ZCm8qe.png" width="300px" /> </div>
<div align=center> Fig A-7 Displacement</div>
logfile\_viewer.py 可以将 poses\_from\_ticks.txt 可视化。 

<div align=center><img src="https://i.imgur.com/fxJ9nVi.png" width="400px" /> </div>
<div align=center> Fig A-8 logfile_view</div>
在可视化窗口中加入robot4\_reference.txt，可以观察到小车和参考轨迹之间的差距，这是由于小车的宽度参数并不是实际参数，通过修改robot\_width这一变量，可以调整计算得来的轨迹精度。

<div align=center><img src="https://i.imgur.com/6m6u3x9.png" width="600px" /> </div>
<div align=center> Fig A-9 logfile_view, left: robot_width=150, right: robot_width=171</div>
下一步是引入传感器LiDAR得到的深度信息，以此来确定Arena中的Landmark位置。在可视化窗口载入robot4\_scan.txt, 可以观察到小车的传感器信息，

<div align=center><img src="https://i.imgur.com/5NyxAfq.png" width="800px" /> </div>
<div align=center> Fig A-10 left: robot4\_scan.txt, middle: 可视化窗口, right: 单次时间点读取的数据</div>
为了得到Landmark的数量，我们要对传感器数据进行微分，得到单次时间不同角度之间的差值 (derivative)，通过计数每次传感器数值的突变来得到Landmark的数量：

$$
f'(i) \approx \frac{f(i+1)-f(i-1)}{2} \tag{1.3}
$$

从上图右侧窗口可知，在某些角度，比如400附近，产生了一个传感器误差，此时传感器的值小于20，在程序slam\_03\_b我们对这样的数值进行过滤。最终得到该图像的微分图像：

<div align=center><img src="https://i.imgur.com/G93N2pT.png" width="400px" /> </div>
<div align=center> Fig A-11 传感器LiDAR的数据及其微分</div>
在程序slam\_03\_c中，通过对Landmark左边的edge和右边的edge进行计数，可以得到Landmark的数量，通过对左edge和右edge之间的数值求平均值，可以得到该Landmark相对于传感器的角度和深度信息，以此，我们可以确定该Landmark的位置。

<div align=center><img src="https://i.imgur.com/36qq7A7.png" width="400px" /> </div>
<div align=center> Fig A-12 Landmark 相对于LiDAR的极坐标</div>
在程序slam\_03\_d中，我们将Landmark的位置从相对LiDAR的极坐标位置转换成笛卡尔坐标系位置，并考虑到传感器接触Landmark表面与Lanmark中心点之间的差值大约为90mm, 最终结果输出到可视化窗口后如图，小绿点为Landmark通过建模估计得到的位置，灰点为Landmark实际位置：

<div align=center><img src="https://i.imgur.com/usofPRd.png" width="400px" /> </div>
<div align=center> Fig A-13 Landmark 相对于LiDAR的笛卡尔坐标</div>

### Unit B

在转弯的时候，Landmark的估计结果有较大误差，通过比较估计的Landmark和实际的Landmark的差距，我们可以矫正对应的小车位置。

<div align=center><img src="https://i.imgur.com/5KQvCNR.png" width="300px" /> </div>
<div align=center> Fig B-1 Landmark 估计与实际位置的差距</div>

首先通过 slam\_04\_a 可以将Landmark相对于地图的笛卡尔坐标计算出来，结果与上图通过 logfile\_viewer 类似。 我们要把探测到的Landmark与实际存在的最近的Landmark进行匹配。程序 slam\_04\_b 实现了这样的效果。 

<div align=center><img src="https://i.imgur.com/1zeFH7m.png" width="400px" /> </div>
<div align=center> Fig B-2 Landmark 估计位置与实际位置进行配准</div>

下一步是将匹配好的点经过旋转位移等操作进行配准。对点云进行配准需要相似变换：

$$
\lambda \vec{R} \vec{l}\_i + \vec{t} = \vec{r}\_i \tag{2.1}
$$

其中需要确认以下四个变量：

$$
\begin{aligned}
\lambda&: scale \in \mathbb{R} \\\\
\vec{R}&: \begin{bmatrix} \cos{\alpha} & -\sin{\alpha} \\\\ \sin{\alpha} & \cos{\alpha} \end{bmatrix} \in \mathbb{R}^{2x2}, \alpha \in \begin{bmatrix} 0&2 \pi \end{bmatrix} \\\\
\vec{t}&: \begin{bmatrix} t\_x & t\_y \end{bmatrix} 
\end{aligned}
\tag{2.2}
$$

由于存在噪声误差，上述变换公式中，等式左边并不完全等于等式右边，我们需要用到最小二乘法来优化该问题。但由于旋转矩阵和scale变量的存在，该优化问题为非线性问题：

$$
\sum_i = || \lambda \vec{R} \vec{l}\_i + \vec{t} - \vec{r}\_i ||^2 \tag{2.3}
$$

所以接下来我们先计算两个点云的中心，通过每个点减去点云中心，得到基于点云中心的每个点的坐标：

$$
\begin{aligned}
\vec{l}\_m &= \frac{1}{m} \sum\_i \vec{l}\_i \\\\
\vec{r}\_m &= \frac{1}{m} \sum\_i \vec{r}\_i \\\\
\vec{l}\_i' &= \vec{l}\_i - \vec{l}\_m \\\\
\vec{r}\_i' &= \vec{r}\_i - \vec{r}\_m 
\end{aligned}
\tag{2.4}
$$

并且满足：

$$
\sum\_i \vec{l}\_i' = 0 \tag{2.5}
$$

<div align=center><img src="https://i.imgur.com/osnykQi.png" width="250px" /> </div>
<div align=center> Fig B-3 点云基于中心点的坐标向量</div>

由此可得：

$$
\begin{aligned}
\lambda \vec{R} \vec{l}\_i - \vec{r}\_i + \vec{t} &= \lambda \vec{R} (\vec{l}\_i' + \vec{l}\_m) - (\vec{r}\_i' + \vec{r}\_m) + \vec{t} \\\\
&= \lambda \vec{R} \vec{l}\_i' - \vec{r}\_i' + \lambda \vec{R} \vec{l}\_m - \vec{r}\_m + \vec{t} \\\\
&= \lambda \vec{R} \vec{l}\_i' - \vec{r}\_i' + \vec{t}' 
\end{aligned}
\tag{2.6}
$$

接下来我们要对上述模型进行优化，利用最小二乘法可得：

$$
\begin{aligned}
& \sum\_i || \lambda \vec{R} \vec{l}\_i' - \vec{r}\_i' + \vec{t}' ||^2 \\\\
= &\sum\_i || \lambda \vec{R} \vec{l}\_i' - \vec{r}\_i' ||^2 + 2{\vec{t}'}^{\rm{T}} \sum\_i(\lambda \vec{R} \vec{l}\_i' - \vec{r}\_i') + \sum\_i ||\vec{t}' ||^2 \\\\
= &\sum\_i || \lambda \vec{R} \vec{l}\_i' - \vec{r}\_i' ||^2 + m ||\vec{t}'||^2 \longrightarrow min
\end{aligned}
\tag{2.7}
$$

为了使上式达到最小值，首先第一项要达到最小，其次第二项可以达到零:

$$
\begin{aligned}
\vec{t}' &= 0 \\\\
\lambda \vec{R} \vec{l}\_m - \vec{r}\_m + \vec{t} &= 0 \\\\
 \vec{t} &= \vec{r}\_m - \lambda \vec{R} \vec{l}\_m \\\\
\end{aligned}
\tag{2.8}
$$

通过分析上式可知，只需确认\\(\lambda\\) 和\\(\vec{R}\\) 即可使上式为零。接着我们分析第一项，通过对第一项等效变换可得：

$$
\begin{aligned}
&\sum\_i || \lambda \vec{R} \vec{l}\_i' - \vec{r}\_i||^2 \longrightarrow min \\\\
\Longrightarrow&\sum\_i || \sqrt{\lambda} \vec{R} \vec{l}\_i' - \frac{1}{\sqrt{\lambda}}\vec{r}\_i||^2 \longrightarrow min \\\\
=& \lambda \sum\_i || \vec{R}\vec{l}\_i'||^2 - 2 \sum\_i {\vec{r}\_i'}^{\rm{T}} \vec{R} \vec{l}\_i' + \frac{1}{\lambda}\sum\_i ||\vec{r}\_i' ||^2 \\\\
\Longrightarrow& \lambda \sum\_i ||\vec{l}\_i'||^2 - 2 \sum\_i \vec{r}\_i^{\rm{T}} \vec{R} \vec{l}\_i' + \frac{1}{\lambda}\sum\_i ||\vec{r}\_i' ||^2 \\\\
=& \lambda a - b + \frac{1}{\lambda}c \stackrel{min}{\longrightarrow} \lambda^2 = \frac{c}{a} \\\\
\Longrightarrow& \lambda^2 = \frac{\sum\_i ||\vec{r}\_i'||^2}{\sum\_i||\vec{l}\_i'||^2} \\\\
\Longrightarrow& \lambda = \sqrt{\frac{\sum\_i ||\vec{r}\_i'||^2}{\sum\_i||\vec{l}\_i'||^2}}
\end{aligned} 
\tag{2.9}
$$

要使得上式达到最小，我们还需要使b项达到最大：

$$
\begin{aligned}
& \sum\_i {\vec{r}\_i'}^{\rm{T}} \vec{R} \vec{l}\_i' \longrightarrow max \\\\
 = & \begin{bmatrix} r'\_x & r'\_y \end{bmatrix} \begin{bmatrix} \cos{\alpha} & -\sin{\alpha} \\\\ \sin{\alpha} & \cos{\alpha} \end{bmatrix} \begin{bmatrix} l'\_x \\\\ l'\_y \end{bmatrix} \\\\
 = & \begin{bmatrix} r'\_x & r'\_y \end{bmatrix} \begin{bmatrix} l'\_x \cos{\alpha} & -l'\_y & \sin{\alpha} \\\\ l'\_x \sin{\alpha} & +l'\_y & \cos{\alpha} \end{bmatrix} \\\\
 = & r'\_x l'\_x \cos{\alpha} - r'\_x l'\_y \sin{\alpha} + r'\_y l'\_x \sin{\alpha} + r'\_y l'\_y \cos{\alpha} \\\\
 = & \cos{\alpha} \sum\_i(r'\_xl'\_x + r'\_yl'\_y) + \sin{\alpha}\sum\_i(-r'\_xl'\_y + r'\_yl'\_x) \longrightarrow max \\\\
 = & \begin{bmatrix} \cos{\alpha} & \sin{\alpha} \end{bmatrix} \begin{bmatrix} \sum\_i(r'\_xl'\_x + r'\_yl'\_y) \\\\ \sum\_i(-r'\_xl'\_y + r'\_yl'\_x) \end{bmatrix} \longrightarrow max
\end{aligned}
\tag{2.10}
$$

上列式子由两个向量相乘，可以判断出，当两个向量具有相同方向时，其乘积达到最大，如下图所示：

<div align=center><img src="https://i.imgur.com/a9yx72T.png" width="300px" /> </div>
<div align=center> Fig B-4 向量乘积</div>

所以当满足以下条件时达到最大：

$$
\begin{bmatrix} \cos{\alpha} \\\\ \sin{\alpha}  \end{bmatrix} = \frac{1}{|| \begin{bmatrix} \sum\_i(r'\_xl'\_x + r'\_yl'\_y) \\\\ \sum\_i(-r'\_xl'\_y + r'\_yl'\_x) \end{bmatrix} ||} \begin{bmatrix} \sum\_i(r'\_xl'\_x + r'\_yl'\_y) \\\\ \sum\_i(-r'\_xl'\_y + r'\_yl'\_x) \end{bmatrix}\tag{2.11}
$$

接下来整理以下编程的思路：

Given:
$$
 \vec{l}\_i, \vec{r}\_i, 1 \leq i \leq m \tag{2.12}
$$

Compute:
$$
\begin{aligned}
&\vec{l}\_m = \frac{1}{m}\sum\_i\vec{l}\_i,\quad \vec{r}\_m = \frac{1}{m}\sum\_i\vec{r}\_i \\\\
&\vec{l}\_i' = \vec{l}\_i - \vec{l}\_m,\quad \vec{r}\_i' = \vec{r}\_i - \vec{r}\_m \\\\
\\\\
&cs, ss, rr, ll = 0.0 \\\\
&for \quad i\quad in\quad 1\quad \cdots m: \\\\
&\qquad sum\ of\ cos: cs += r\_x'l\_x' + r\_y'l\_y' \\\\
&\qquad sum\ of\ sins: ss  += -r\_x'l\_y' + r\_y'l\_x' \\\\
&\qquad length\ of\ vectors: rr += r\_x'r\_x' + r\_y'r\_y' \\\\
&\qquad length\ of\ vectors: ll += l\_x'l\_x' + l\_y'l\_y' \\\\
\\\\
&\lambda = \sqrt{\frac{rr}{ll}} \\\\
&\begin{bmatrix}\cos{\alpha} \\\\ \sin{\alpha} \end{bmatrix} = \frac{\begin{bmatrix} cs\\\\ ss \end{bmatrix}}{\sqrt{cs^2 + ss^2}} \\\\
& \begin{bmatrix} t\_x \\\\ t\_y \end{bmatrix} = \vec{r}\_m - \lambda\vec{R}\vec{l}\_m = \begin{bmatrix} r\_{m,x} \\\\ r\_{m,y} \end{bmatrix} - \lambda \begin{bmatrix} \cos{\alpha} & -\sin{\alpha} \\\\ \sin{\alpha} & \cos{\alpha} \end{bmatrix} \begin{bmatrix} l\_{m,x} \\\\ l\_{m,y} \end{bmatrix} \\\\
\\\\
&return (\lambda, \cos{\alpha}, \sin{\alpha}, t\_x, t\_y)
\end{aligned}
\tag{2.13}
$$

通过把\\(\lambda\\) 设置为1，上述式子还能从相似变换（similar transformation）转换为 僵硬变换 (rigid transformation) 。













