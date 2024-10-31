 
class PID():     # 定义一个名为PID的类
	def __init__(self, dt, max, min, Kp, Kd, Ki):     # 类的初始化方法，接收6个参数：时长dt、最大值max、最小值min、比例增益Kp、微分增益Kd和积分增益Ki
		self.dt = dt    # 循环时长
		self.max = max  # 操作变量最大值
		self.min = min  # 操作变量最小值
		self.Kp = Kp         # 比例增益
		self.Kd = Kd         # 微分增益
		self.Ki = Ki         # 积分增益
		self.integral = 0    # 直到上一次的误差值      # 初始化积分项累计误差为0
		self.pre_error = 0   # 上一次的误差值         # 初始化上一次的误差值为0
 
	def calculate(self, setPoint, pv):       # 定义一个名为calculate的方法，用于计算PID控制器的输出。接收2个参数：设定点setPoint和过程值pv
		# 其中 pv:process value 即过程值，
		error = setPoint - pv           # 误差    # 计算误差值
		Pout = self.Kp * error          # 比例项   # 计算比例项输出
		self.integral += error * self.dt      # 更新积分项累计误差
		Iout = self.Ki * self.integral  # 积分项    # 计算积分项输出
		derivative = (error - self.pre_error)/self.dt    # 计算误差的变化率（导数）
		Dout = self.Kd * derivative     # 微分项   # 计算微分项输出
 
		output = Pout + Iout + Dout     # 新的目标值    # 新的目标值为比例、积分、微分项之和
 
		if(output > self.max):     # 限制输出值不超过最大值
			output = self.max
		elif(output < self.min):   # 限制输出值不低于最小值
			output = self.min
 
		self.pre_error = error         # 保存本次误差，以供下次计算
		return output               # 返回输出值


# 下面是程序的主入口，用于测试PID控制器的效果
if __name__ == '__main__':
    import matplotlib.pyplot as plt     # 导入matplotlib库绘图功能
    t = range(150)                            # 定义时间序列t
    pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5)      # 创建一个PID对象
    val = 20                         # 初始化过程值为20
    z = []                          # 初始化一个空列表z，用于存储设定点与过程值之间的误差
    for i in t:                 # 对每个时刻进行循环
        inc = pid.calculate(0, val)      # 调用calculate方法计算PID输出
        print("val:{} inc:{}".format(val,inc))   # 打印过程值和输出递增值
        z.append(20-val)             # 将误差值添加到列表z
        val += inc                  # 更新过程值
    plt.figure(figsize=(8,6), dpi = 80)           # 创建一个绘图窗口
    plt.plot(t,z,color="blue",linewidth=1.0,linestyle="-")          # 绘制误差随时间的变化曲线
    plt.show()                   # 显示绘图结果
    

	#这个程序定义了一个PID类，用于实现简单的比例-积分-微分（PID）控制器。
	# 另外，也提供了一个主入口部分，通过matplotlib库展示了采用该PID控制器进行调节时过程值与设定点之间误差的变化趋势。

	#上面程序中的PID控制器计算过程包含了以下数学公式：

#误差计算：
#error = setPoint - pv

#比例项（P）计算：
#Pout = Kp * error

#积分项（I）计算：
#self.integral += error * self.dt
#Iout = Ki * self.integral

#微分项（D）计算：
#derivative = (error - self.pre_error) / self.dt
#Dout = Kd * derivative

#PID控制器输出计算：
#output = Pout + Iout + Dout

#这些公式描述了比例-积分-微分（PID）控制器的基本原理，其中Kp、Ki和Kd分别表示比例增益、积分增益和微分增益。
# 通过调整这些增益参数，可以改变PID控制器的性能，使其适应不同的控制需求。