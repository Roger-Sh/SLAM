test 2

test
# SLAM - Simultaneous localization and mapping

## Introduction 

这份笔记是针对汉诺威大学Claus Brenner教授的课程SLAM 整理而来，笔记内的某些专业词汇可能是德语，该笔记仅供个人学习使用。课程所需的编程环境为python2.7. 在github页面正确显示公式需要安装一个插件: 
[MathJax Plugin for Github](https://chrome.google.com/webstore/detail/mathjax-plugin-for-github/ioemnmodlmafdkllaclgeombjnmnbima), 教程视频： [SLAM Lectures](https://www.youtube.com/playlist?list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm)，Repo：[SLAM](https://github.com/ipa-fog-ws/SLAM)，该课程分为以下几个模块：

- Unit A: Roboter, Sensor, Arena,Landmark, Motion Model, 
- Unit B: LS-Square, correkt the trajektory, Feature Based Approach，Featureless Approach，Iterative-Closest-Point
- Unit C: Probability-Distribution, Bayes-Filter, Kalmann-Filter (One-Dimention)
- Unit D: Multivariate Normal Distribution, Kalmann-Filter (Muilti-Dimention),  Extended-Kalmann-Filter(Non-Linear)
- Unit E: Particle Filter
- Unit F: Mapping, Extended-Kalmann-Filter SLAM
- Unit G: Particle Filter SLAM 

## Unit A

### 小车及环境构造

Unit A 的内容是对一个乐高小车的运动进行建模，首先利用乐高小车的电机编码器返回的数据计算出小车的运动轨迹，其次利用乐高小车搭载的LiDAR传感器获得周围环境的深度信息，检测出Landmark所在的位置。

<div align=center><img src="https://i.imgur.com/0uQWKGA.png" width="500px" /> </div>
<div align=center> Fig A-1 Lego Car </div>

<div align=center><img src="https://i.imgur.com/DBH8449.png" width="500px" /> </div>
<div align=center> Fig A-2 Arena and Landmark </div>

下图展示了乐高小车的运动模型，由两个电机带动左右履带，履带的位移差距可以让小车进行转向。

<div align=center><img src="https://i.imgur.com/YErhk8h.png" width="500px" /> </div>
<div align=center> Fig A-3 Lego Car Structur</div>

小车的编码器可以表示电机的角度，经过换算，1 tick 约为 0,349 mm 履带前进的距离。下图表示了一个电机的log-file中的内容。

<div align=center><img src="https://i.imgur.com/lMD8jBt.png" width="500px" /> </div>
<div align=center> Fig A-4 Motor log file </div>

其中，第0列 'M' 表示Motor, 第1列表示时间戳 (time stamp), 第2列表示左轮Motor的编码， 第6列表示右轮Motor的编码，第10列表示第三个motor的编码，这里我们只需要左轮和右轮的motor即可。slam\_01\_a， slam\_01\_b，slam\_01\_c 这三个程序展示了如何通过lego\_robot 这个class来读取robot4\_motors.txt 。

### 小车运动模型

下一步是建立小车转弯时的运动模型。

<div align=center><img src="https://i.imgur.com/p6utG7D.png" width="500px" /> </div>
<div align=center> Fig A-5 Motion Model</div>

由下列公式可以得到小车绕中心点旋转时的旋转半径 $R$ 和旋转的角度 $\alpha$：

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

<div align=center><img src="https://i.imgur.com/t2V9Qr2.png" width="500px" /> </div>
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

<div align=center><img src="https://i.imgur.com/9ZCm8qe.png" width="500px" /> </div>
<div align=center> Fig A-7 Displacement</div>

logfile\_viewer.py 可以将 poses\_from\_ticks.txt 可视化。 

<div align=center><img src="https://i.imgur.com/fxJ9nVi.png" width="500px" /> </div>
<div align=center> Fig A-8 logfile_view</div>

在可视化窗口中加入robot4\_reference.txt，可以观察到小车和参考轨迹之间的差距，这是由于小车的宽度参数并不是实际参数，通过修改robot\_width这一变量，可以调整计算得来的轨迹精度。

<div align=center><img src="https://i.imgur.com/6m6u3x9.png" width="600px" /> </div>
<div align=center> Fig A-9 logfile_view, left: robot_width=150, right: robot_width=171</div>

### 基于LiDAR信息预测Landmark位置 

下一步是引入传感器LiDAR得到的深度信息，以此来确定Arena中的Landmark位置。在可视化窗口载入robot4\_scan.txt, 可以观察到小车的传感器信息，

<div align=center><img src="https://i.imgur.com/5NyxAfq.png" width="800px" /> </div>
<div align=center> Fig A-10 left: robot4\_scan.txt, middle: 可视化窗口, right: 单次时间点读取的数据</div>

为了得到Landmark的数量，我们要对传感器数据进行微分，得到单次时间不同角度之间的差值 (derivative)，通过计数每次传感器数值的突变来得到Landmark的数量：

$$
f'(i) \approx \frac{f(i+1)-f(i-1)}{2} \tag{1.3}
$$

从上图右侧窗口可知，在某些角度，比如400附近，产生了一个传感器误差，此时传感器的值小于20，在程序slam\_03\_b我们对这样的数值进行过滤。最终得到该图像的微分图像：

<div align=center><img src="https://i.imgur.com/G93N2pT.png" width="500px" /> </div>
<div align=center> Fig A-11 传感器LiDAR的数据及其微分</div>

在程序slam\_03\_c中，通过对Landmark左边的edge和右边的edge进行计数，可以得到Landmark的数量，通过对左edge和右edge之间的数值求平均值，可以得到该Landmark相对于传感器的角度和深度信息，以此，我们可以确定该Landmark的位置。

<div align=center><img src="https://i.imgur.com/36qq7A7.png" width="500px" /> </div>
<div align=center> Fig A-12 Landmark 相对于LiDAR的极坐标</div>

在程序slam\_03\_d中，我们将Landmark的位置从相对LiDAR的极坐标位置转换成笛卡尔坐标系位置，并考虑到传感器接触Landmark表面与Lanmark中心点之间的差值大约为90mm, 最终结果输出到可视化窗口后如图，小绿点为Landmark通过建模估计得到的位置，灰点为Landmark实际位置：

<div align=center><img src="https://i.imgur.com/usofPRd.png" width="500px" /> </div>
<div align=center> Fig A-13 Landmark 相对于LiDAR的笛卡尔坐标</div>

## Unit B

### 基于Landmark预测建立相似变换公式

在转弯的时候，Landmark的估计结果有较大误差，通过比较估计的Landmark和实际的Landmark的差距，我们可以矫正对应的小车位置。

<div align=center><img src="https://i.imgur.com/5KQvCNR.png" width="300px" /> </div>
<div align=center> Fig B-1 Landmark 估计与实际位置的差距</div>

首先通过 slam\_04\_a 可以将Landmark相对于地图的笛卡尔坐标计算出来，结果与上图通过 logfile\_viewer 类似。 我们要把探测到的Landmark与实际存在的最近的Landmark进行匹配。程序 slam\_04\_b 实现了这样的效果。 

<div align=center><img src="https://i.imgur.com/1zeFH7m.png" width="500px" /> </div>
<div align=center> Fig B-2 Landmark 估计位置与实际位置进行配准</div>

下一步是将匹配好的点经过旋转位移等操作进行配准。对点云进行配准需要相似变换：

$$
\lambda \vec{R} \vec{l}\_i + \vec{t} = \vec{r}\_i \tag{2.1}
$$

其中需要确认以下3个变量：

$$
\begin{aligned}
\lambda&: scale \in \mathbb{R} \\\\
\vec{R}&: \begin{bmatrix} \cos{\alpha} & -\sin{\alpha} \\\\ \sin{\alpha} & \cos{\alpha} \end{bmatrix} \in \mathbb{R}^{2x2}, \alpha \in \begin{bmatrix} 0&2 \pi \end{bmatrix} \\\\
\vec{t}&: \begin{bmatrix} t\_x & t\_y \end{bmatrix} 
\end{aligned}
\tag{2.2}
$$

### 利用最小二乘法确定相似变换公式

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

<div align=center><img src="https://i.imgur.com/osnykQi.png" width="300px" /> </div>
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

通过分析上式可知，只需确认$\lambda$ 和$\vec{R}$ 即可使上式为零。接着我们分析第一项，通过对第一项等效变换可得：

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

<div align=center><img src="https://i.imgur.com/a9yx72T.png" width="400px" /> </div>
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

通过把$\lambda$ 设置为1，上述式子还能从相似变换（similar transformation）转换为 僵硬变换 (rigid transformation) 。值得注意的是，为了计算出上式中四个变量，我们需要至少四个观测值（Observation），否则无法执行最小二乘法（least square）。程序slam\_04\_c 实现了最小二乘法（least square）。

<div align=center><img src="https://i.imgur.com/BocRVM1.png" width="400px" /> </div>
<div align=center> Fig B-5 利用最小二乘法匹配到的Landmark</div>

### 利用相似变换公式矫正小车轨迹

下一步，在程序 >slam\_04\_d中 我们利用通过Landmark 确定下来的相似变换公式，同样可以用来矫正机器人的位置 (Pose) 和朝向 (Heading)，如下图所示：

<div align=center><img src="https://i.imgur.com/LDRSsvc.png" width="400px" /> </div>
<div align=center> Fig B-6 通过Landmark确定的相似变换公式矫正机器人的位置和朝向</div>

$$
\begin{aligned}
\begin{bmatrix} x' \\\\ y' \end{bmatrix} &= \lambda \begin{bmatrix} \cos{\alpha} & -\sin{\alpha} \\\\ \sin{\alpha} & \cos{\alpha} \end{bmatrix} \begin{bmatrix} x \\\\ y \end{bmatrix} + \begin{bmatrix} t\_x \\\\ t\_y \end{bmatrix} \\\\
\alpha &= atan2(\sin{\alpha}, \cos{\alpha})
\end{aligned}
\tag{2.14}
$$

<div align=center><img src="https://i.imgur.com/PXcII5Z.png" width="400px" /> </div>
<div align=center> Fig B-7 通过Landmark确定的相似变换公式矫正机器人的位置和朝向，修正后的轨迹</div>

将图 B-7与图 B-6 进行比较发现，在B-6中由于模型本身的参数误差造成的轨迹误差，通过观测Landmark的位置进行修正，在图 B-7 中有了很大的改善，但此时由于Feature点较少，轨迹修正不够顺滑，图中机器人的轨迹有较大突变，这不是我们想要的结果。利用雷达信息提取出Landmark信息，我们称之为 Feature Based Approach，但有时候雷达只能检测到一个 Landmark，此时便无法使用最小二乘法，但此时雷达仍然能获得墙壁的信息，这些信息没有特殊的Feature，但我们仍然可以利用它们，这种方法称为 Featureless Approach。

### 利用墙面信息进行 Featureless Approach， 更顺滑地矫正小车轨迹

<div align=center><img src="https://i.imgur.com/EGbfA0p.png" width="400px" /> </div>
<div align=center> Fig B-8 Feature Based Approach</div>

<div align=center><img src="https://i.imgur.com/GHLzSqP.png" width="400px" /> </div>
<div align=center> Fig B-9 利用墙壁的信息进行Non Feature Based Approach</div>

在程序slam\_05\_a 中，LiDAR获取的墙面数据与场地墙面坐标进行比较，找出墙面上与LiDAR数据最接近的坐标，下图展示了LiDAR数据点与最近的墙面坐标：

<div align=center><img src="https://i.imgur.com/oNeS4ET.png" width="500px" /> </div>
<div align=center> Fig B-10 匹配的墙面与LiDAR数据点</div>

同样的，基于墙面信息同样能得到相似变化公式，从而来矫正机器人的轨迹，程序slam\_05\_b 中实现了利用墙面信息进行相似矩阵估计并矫正机器人轨迹的算法，下图展示了Featureless Approach的效果：

<div align=center><img src="https://i.imgur.com/wnrajnU.png" width="500px" /> </div>
<div align=center> Fig B-11 Featureless Approach</div>

### ICP Iterative Closest Points Approach

该基于墙面信息的Featureless Approach 的缺点在于，每个墙面的数据点都想要进行修正，然而却互相牵制，无法直接达到最优化效果，如下图所示：

<div align=center><img src="https://i.imgur.com/e9EksOL.png" width="400px" /> </div>
<div align=center> Fig B-12 Featureless Approach 的缺点</div>

为了达到最优效果，我们采取迭代最近点的方法，即 Iterative Closest Point (ICP)，每次优化之后，更新相似变换矩阵，通过迭代的方式最终达到最优化状态：

<div align=center><img src="https://i.imgur.com/z6uBIrB.png" width="400px" /> </div>
<div align=center> Fig B-13 Iterative Closest Point (ICP)</div>

程序slam\_05\_c 中实现了ICP的算法，伪代码如下：

	# Init overall_trafoo	
	overall_trafo = (1.0, 1.0, 0.0, 0.0, 0.0)
	for j in xrange(iterations):
	 	将world_points_init 变换成 world_points_new
		Call get_correspoinding_points_on_wall(...) 找到对应的墙上的点
		找出从world_points_new 变换成 墙上的点的trafo
		将旧的overall_trafo与新的trafo连接，
		overall_trafo = concatenate_transform(trafo, overall_trafo)
	if trafo返回为none的时候，
		结束循环
	返回最终的overrall_trafo


利用 ICP 算法矫正的机器人轨迹如下：

<div align=center><img src="https://i.imgur.com/SDsBqdC.png" width="500px" /> </div>
<div align=center> Fig B-14 利用 ICP 算法矫正的机器人轨迹</div>

将图 B-14 与 图B-12 以及 B-13 相比较，可以看出，Feature Based Approach (利用Landmark信息) 的轨迹有较大突变，Feautureless Approach （利用墙面信息，无 ICP） 的轨迹比较顺滑，Featureless Approach with ICP 的算法，轨迹最为顺滑。

## Unit C

### 误差的类型

从Unit A 中我们了解到，由于未对机器人的建模参数进行矫正 (calibration), 所以机器人在转弯的时候有较大的误差。通过对前面章节机器人的轨迹误差进行分析，可以将机器人的轨迹误差大致分为两类：

- Systematic Error 系统误差
    - use/add calibration parameters, 矫正过的参数可能和实际不匹配
    - can't capture all real world properties in the model，比如更换地面之后，摩擦力和位移会发生变化，模型不再适用
- Random Error 随机误差

<div align=center><img src="https://i.imgur.com/As2XjUr.png" width="400px" /> </div>
<div align=center> Fig C-1 系统误差和随机误差</div>

### 贝叶斯过滤器

我们来考虑以下机器人一维下的移动，其位置精度的概率分布可能如下：

<div align=center><img src="https://i.imgur.com/wPwrvbH.png" width="500px" /> </div>
<div align=center> Fig C-2 机器人位置的概率分布</div>

在程序 distribution.py 中我们设置了一个关于离散分布的类，里面包含了一个离散分布可能用到的各种变量和各种方法，程序 slam\_06\_a 展示了如何设置一个 distribution 的 offset 和 values 。

接下来的问题是不确定性的传递，如下图所示，在初始位置我们很确定机器人的位置，到了第二步，机器人的位置有了不确定性，到了第三步，机器人的不确定性会在第二步之上继续叠加：

<div align=center><img src="https://i.imgur.com/lZKH42i.png" width="500px" /> </div>
<div align=center> Fig C-3 机器人位置的概率分布</div>

如上图所示，当A事件发生时B事件发生的概率为：

$$
P(A,B) = P(B|A)P(A) \tag{3.1}
$$

第二步中机器人位于每个位置的可能性如下图所示：

<div align=center><img src="https://i.imgur.com/LYPVJiC.png" width="500px" /> </div>
<div align=center> Fig C-3 机器人位置的概率分布</div>

$$
\rm{Total\ posibility:\ } P(A,B) = \sum\_i P(B|A\_i)P(A\_i) \tag{3.2}
$$

对于上图的概率分布，这实际上相当于对事件A的概率用事件B的概率进行卷积，程序 slam\_06\_b 实现了对概率分布进行卷积操作。下图展示了一个机器人的位置如何通过三次位移后达到一个不确定状态以及其每一步位移之后位置的概率分布：

<div align=center><img src="https://i.imgur.com/1K5XmIY.png" width="500px" /> </div>
<div align=center> Fig C-4 机器人经过三次位移之后的概率分布</div>

机器人通过建模对自身位置进行估计时，我们要考虑到由于模型的局限性带来的随机误差。 机器人通过传感器对自身位置进行估计时，同样要考虑随机误差，模型的随机误差和传感器的随机误差通过相乘结合到一起。

<div align=center><img src="https://i.imgur.com/c1P9a7b.png" width="500px" /> </div>
<div align=center> Fig C-5 测量随机误差，模型随机误差，以及其总体随机误差</div>

如上图，模型随机误差我们称之为 先验 (Prior)， 测量随机误差我们称之为 测量(Measurment), 他们的总体误差将通过这两者随机误差相乘得到。

首先我们来了解一下条件概率：

$$
\begin{aligned}
Posterior Probability\ (后验概率) &= \frac{Prior Probability\ (先验概率)}{Evidence\ (证据)} \\\\
\\\\
P(X|Z) &= \frac{P(Z|X)\cdot P(X)}{P(Z)}\\\\
&= \frac{P(Z|X)\cdot P(X)}{\sum\_{x'}P(Z|x')\cdot P(x')}
\end{aligned}
\tag{3.3}
$$

程序 slam\_06\_c 实现了两个概率分布相乘的算法，效果图如下：

<div align=center><img src="https://i.imgur.com/5qM2JIH.png" width="500px" /> </div>
<div align=center> Fig C-6 概率分布相乘</div>

通过对Measurement的位移，Posterior的改变如下：

<div align=center><img src="https://i.imgur.com/AhXYyi2.png" width="500px" /> </div>
<div align=center> Fig C-7 Measurement位移造成的Posterior的改变</div>

对于机器人小车而言，基于模型的位置估计有随机误差，基于传感器的观测也有随机误差，我们不能只相信模型或只相信传感器，而是要对这两者的概率分布进行结合。对于每一步小车的移动，首先要把小车模型的概率分布与上一时间点小车总体的概率分布进行卷积，得到这一步小车模型的概率分布，其次通过与传感器的高铝分布相乘，得到此时小车最终的概率分布：

1. Motion - Convolution
	
$$
P(X) = \sum\_y P(X|y) \cdot P(y) \tag{3.4}
$$

2. Measurement - Multiplication

$$
P(X|Z) = \alpha \cdot P(Z|X) \cdot P(X) \tag{3.5}
$$

<div align=center><img src="https://i.imgur.com/TNCq9dC.jpg" width="500px" /> </div>
<div align=center> Fig C-8 某一时间点小车的模型概率部分与传感器的概率分布</div>

计算机器人位置整体概率分布的算法如下：

<div align=center><img src="https://i.imgur.com/Gr6GnSz.png" width="500px" /> </div>
<div align=center> Fig C-9 贝叶斯过滤器，考虑小车模型的概率分布以及传感器的概率分布</div>

下图展示了一个机器人通过贝叶斯过滤器，综合考虑模型概率分布以及传感器概率分布，并分别使用三角分布以及正态分布的结果：

<div align=center><img src="https://i.imgur.com/6t6FofV.png" width="800px" /> </div>
<div align=center> Fig C-10 贝叶斯过滤器，左：三角分布，右：正态分布</div>

###正态分布

下图展示了一些正态分布的基础知识：

<div align=center><img src="https://i.imgur.com/tJD4vpY.png" width="500px" /> </div>
<div align=center> Fig C-11 正态分布</div>

正态分布的积分不是1，而是由 $\sigma$ 决定的。 正态分布的密度如下:

$$
\mathcal{N}(x;\mu,\sigma^2) = \frac{1}{\sqrt{2\pi} \cdot \sigma} \cdot \mathrm{e}^{-\frac{1}{2}(\frac{x-\mu}{\sigma})^2} 
$$

mit 

$$
\begin{aligned}
\mu:&\ mean \\\\
\sigma^2:&\ variance \\\\
\sigma:&\ standard\ deviation = \sqrt{variance}
\end{aligned}
$$

基于模型预测以及传感器观测，我们可以得到一个belief，假设模型预测以及传感器观测的概率分布皆为正态分布，则得到的belief也为正态分布，我们只需计算其变量 $\mu$ 以及$\sigma^2$:

<div align=center><img src="https://i.imgur.com/jsmkwDG.jpg" width="500px" /> </div>
<div align=center> Fig C-12 两个正态分布相乘仍是正态分布</div>

由以下公式我们可以确定两个正态分布相乘以后仍是正态分布:

$$
\begin{aligned}
bel(x) =& \alpha \cdot p(z|x) \cdot bel'(x) \\\\
=& \alpha' \cdot \mathrm{e}^{-\frac{1}{2}(\frac{z-cx}{\sigma\_Q})^2} \cdot \mathrm{e}^{-\frac{1}{2}(\frac{x-\mu'}{\sigma'})^2} \\\\
=& \alpha' \cdot \mathrm{e}^{-(\frac{1}{2}(\frac{z-cx}{\sigma\_Q})^2 + \frac{1}{2}(\frac{x-\mu'}{\sigma'})^2)} \\\\
=& \alpha'' \cdot \mathrm{e} ^{-\frac{1}{2}(\frac{x-\mu}{\sigma})^2} \longrightarrow Normalverteilung!
\end{aligned}
$$

接下来我们来推导变量 $\mu$ 以及$\sigma^2$:

由下述公式可得二次函数的系数:

$$
\begin{aligned}
f(x) =& \frac{1}{2}A(x-B)^2+C \\\\
\frac{\partial}{\partial x}f(x) =& A(x-B) \overset{!}{=} 0 \\\\
\frac{\partial^2}{\partial x^2}f(x) =& A 
\end{aligned}
$$

从两个正态分布相乘的公式可得:

$$
\begin{aligned}
g(x) =& \frac{1}{2}(\frac{z-cx}{\sigma\_Q})^2 + \frac{1}{2}(\frac{x-\mu'}{\sigma'})^2 \\\\
\frac{\partial}{\partial x}g(x) =& \frac{1}{\sigma\_Q^2}(z-cx)\cdot(-c) + \frac{1}{\sigma'^2}(x - \mu') \overset{!}{=} 0
\end{aligned}
$$
