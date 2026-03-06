# 五连杆控制器学习总结

## MATLAB 推导出了什么？

MATLAB 脚本推导出的是一个雅可比矩阵 **$J$** 的代数表达式。

### 这个 $J$ 有什么作用？ —— 力矩映射

#### 它的物理意义是：

当两个关节电机以速度 $\dot{q} = \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_4 \end{bmatrix}$ 旋转时，足端会以什么样的速度 $\dot{X} = \begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix}$ 在空间中移动。

#### 公式表示为：

$$
\dot{X} = J \cdot \dot{q}
$$

#### 展开就是：

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = \begin{bmatrix} J_{11} & J_{12} \\ J_{21} & J_{22} \end{bmatrix} \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_4 \end{bmatrix}
$$

**MATLAB 的工作到此结束。** 它帮我们算出了这个随时间变化的矩阵里四个元素（$J_{11}, J_{12}, J_{21}, J_{22}$）的庞大代数表达式，这个表达式在控制器 `five_link_controller.cpp` 的 `CaculateJacobian` 函数中直接使用。

---

## 空间力 F 是怎么映射到关节扭矩上的？

### **虚功原理**

### **推导过程：**

1. 假设足端在虚拟力 $F = \begin{bmatrix} F_x \\ F_y \end{bmatrix}$ 的拉动下，移动了一个极其微小的距离 $\delta X$。
2. 同时，两个电机在扭矩 $\tau = \begin{bmatrix} \tau_1 \\ \tau_4 \end{bmatrix}$ 的驱动下，转动了极其微小的角度 $\delta q$。
3. 根据能量守恒定律，**电机做的功，等于足端做的功**：
   $$
   \tau^T \cdot \delta q = F^T \cdot \delta X
   $$
4. 我们已知位移和角度的关系是由雅可比矩阵决定的：$\delta X = J \cdot \delta q$。
5. 把这个代入上面的功的等式：
   $$
   \tau^T \cdot \delta q = F^T \cdot (J \cdot \delta q)
   $$
6. 消去两边的 $\delta q$，再对两边取转置，就得到了整个 VMC 最核心的公式：
   $$
   \tau = J^T \cdot F
   $$

**结论：** 雅可比矩阵不仅能正向翻译速度（$\dot{X} = J \cdot \dot{q}$），它的转置矩阵 **$J^T$** 还能反向翻译力矩

---

## 代码中的具体实现

在 `update` 函数中：

PID 算出了两个虚拟力：

`force_x` 就是 $F_x$

`force_y` 就是 $F_y$

原矩阵是 $J = \begin{bmatrix} J_{11} & J_{12} \\ J_{21} & J_{22} \end{bmatrix}$。

它的**转置矩阵**是把行列互换：$J^T = \begin{bmatrix} J_{11} & J_{21} \\ J_{12} & J_{22} \end{bmatrix}$

我们把力矩映射公式 $\tau = J^T \cdot F$ 展开：

$$
\begin{bmatrix} \tau_1 \\ \tau_4 \end{bmatrix} = \begin{bmatrix} J_{11} & J_{21} \\ J_{12} & J_{22} \end{bmatrix} \begin{bmatrix} F_x \\ F_y \end{bmatrix}
$$

按照矩阵乘法法则（行乘以列），这就等于：

$\tau_1 = J_{11} \cdot F_x + J_{21} \cdot F_y$

$\tau_4 = J_{12} \cdot F_x + J_{22} \cdot F_y$

**对应代码中：**

```cpp
double tau1 = J11 * force_x + J21 * force_y;
double tau4 = J12 * force_x + J22 * force_y;
```


## 学习过程中产生的重要问题

### **\$\\dot {X} = J \\cdot \\dot {q}\$** 具体是怎么来的？

假设有一个简单的机器人，能直接写出它末端坐标 **\$(x, y)\$** 关于关节角度 **\$(\\theta\_1, \\theta\_4)\$** 的方程：

\$\$x = f\_x(\\theta\_1, \\theta\_4)\$\$

\$\$y = f\_y(\\theta\_1, \\theta\_4)\$\$

现在，随着时间 **\$t\$** 变化，我们想知道 **\$x\$** 和 **\$y\$** 的速度（即对时间 **\$t\$** 求导）。根据**多元复合函数的链式求导法则**：

\$\$\\frac{dx}{dt} = \\frac{\\partial f\_x}{\\partial \\theta\_1} \\cdot \\frac{d\\theta\_1}{dt} + \\frac{\\partial f\_x}{\\partial \\theta\_4} \\cdot \\frac{d\\theta\_4}{dt}\$\$

\$\$\\frac{dy}{dt} = \\frac{\\partial f\_y}{\\partial \\theta\_1} \\cdot \\frac{d\\theta\_1}{dt} + \\frac{\\partial f\_y}{\\partial \\theta\_4} \\cdot \\frac{d\\theta\_4}{dt}\$\$

我们把它打包成**矩阵形式**：

\$\$\\begin{bmatrix} \\dot{x} \\\\ \\dot{y} \\end{bmatrix} = \\begin{bmatrix} \\frac{\\partial f\_x}{\\partial \\theta\_1} & \\frac{\\partial f\_x}{\\partial \\theta\_4} \\\\ \\frac{\\partial f\_y}{\\partial \\theta\_1} & \\frac{\\partial f\_y}{\\partial \\theta\_4} \\end{bmatrix} \\begin{bmatrix} \\dot{\\theta}\_1 \\\\ \\dot{\\theta}\_4 \\end{bmatrix}\$\$

**中间那个由偏导数组成的矩阵，就被定义为雅可比矩阵 **\$J\$****。

而在本次的 MATLAB 推导中：

\$\$f\_1(x, y, \\theta\_1, \\theta\_4) = 0\$\$

\$\$f\_2(x, y, \\theta\_1, \\theta\_4) = 0\$\$

对这种式子两边同时对时间 **\$t\$** 求导：

\$\$\\frac{\\partial f\_1}{\\partial x}\\dot{x} + \\frac{\\partial f\_1}{\\partial y}\\dot{y} + \\frac{\\partial f\_1}{\\partial \\theta\_1}\\dot{\\theta}\_1 + \\frac{\\partial f\_1}{\\partial \\theta\_4}\\dot{\\theta}\_4 = 0\$\$

*这就是全微分方程*。把它写成矩阵：

\$\$\\underbrace{\\begin{bmatrix} \\frac{\\partial f\_1}{\\partial x} & \\frac{\\partial f\_1}{\\partial y} \\\\ \\frac{\\partial f\_2}{\\partial x} & \\frac{\\partial f\_2}{\\partial y} \\end{bmatrix}}\_{J\_x} \\begin{bmatrix} \\dot{x} \\\\ \\dot{y} \\end{bmatrix} + \\underbrace{\\begin{bmatrix} \\frac{\\partial f\_1}{\\partial \\theta\_1} & \\frac{\\partial f\_1}{\\partial \\theta\_4} \\\\ \\frac{\\partial f\_2}{\\partial \\theta\_1} & \\frac{\\partial f\_2}{\\partial \\theta\_4} \\end{bmatrix}}\_{J\_{q\\\_raw}} \\begin{bmatrix} \\dot{\\theta}\_1 \\\\ \\dot{\\theta}\_4 \\end{bmatrix} = 0\$\$

整理一下：

\$\$J\_x \\cdot \\dot{X} = -J\_{q\\\_raw} \\cdot \\dot{q}\$\$

令 **\$J\_q = -J\_{q\\\_raw}\$**，并在等式两边同乘 **\$J\_x^{-1}\$**，就得到了最终的：

\$\$\\dot{X} = J\_x^{-1} \\cdot J\_q \\cdot \\dot{q}\$\$

此时，我们的总雅可比矩阵就是

\$J = J\_x^{-1} \\cdot J\_q\$

---

### **\$\\tau = J^T \\cdot F\$** 这个公式对电机数量有没有限制？

#### **雅可比矩阵 **\$J\$** 的维度是：任务维度 × 关节维度。控制 **\$n\$** 维运动至少需要 **\$n\$** 个电机。**

#### 假设我们的手臂在桌面上推杯子：

**任务维度（2维）**：杯子只能在桌面上沿着 **\$X\$** 轴和 **\$Y\$** 轴移动。所以我们需要指尖产生一个二维平面的推力

\$F = \\begin{bmatrix} F\_x \\\\ F\_y \\end{bmatrix}\$

**关节维度（3维）**：手臂平放在桌面上时，能转动的关节有 3 个：**肩膀（电机1）、手肘（电机2）、手腕（电机3）**。所以需要输出 3 个扭矩

\$\\tau = \\begin{bmatrix} \\tau\_1 \\\\ \\tau\_2 \\\\ \\tau\_3 \\end{bmatrix}\$

在这个系统中，雅可比矩阵的维度是 **\$2 \\times 3\$**，那么转置后的矩阵 **\$J^T\$** 就会变成一个 **\$3 \\times 2\$** 的矩阵。

我们把 **\$\\tau = J^T \\cdot F\$** 展开看看：

\$\$\\begin{bmatrix} \\tau\_1 \\\\ \\tau\_2 \\\\ \\tau\_3 \\end{bmatrix} = \\begin{bmatrix} J\_{11} & J\_{21} \\\\ J\_{12} & J\_{22} \\\\ J\_{13} & J\_{23} \\end{bmatrix} \\begin{bmatrix} F\_x \\\\ F\_y \\end{bmatrix}\$\$

按照矩阵乘法展开，大脑实际上在做这样的计算：

**肩膀发力**：**\$\\tau\_1 = J\_{11} \\cdot F\_x + J\_{21} \\cdot F\_y\$**

**手肘发力**：**\$\\tau\_2 = J\_{12} \\cdot F\_x + J\_{22} \\cdot F\_y\$**

**手腕发力**：**\$\\tau\_3 = J\_{13} \\cdot F\_x + J\_{23} \\cdot F\_y\$**
