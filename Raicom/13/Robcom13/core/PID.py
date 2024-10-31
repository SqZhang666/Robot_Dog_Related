 
class PID():     # ����һ����ΪPID����
	def __init__(self, dt, max, min, Kp, Kd, Ki):     # ��ĳ�ʼ������������6��������ʱ��dt�����ֵmax����Сֵmin����������Kp��΢������Kd�ͻ�������Ki
		self.dt = dt    # ѭ��ʱ��
		self.max = max  # �����������ֵ
		self.min = min  # ����������Сֵ
		self.Kp = Kp         # ��������
		self.Kd = Kd         # ΢������
		self.Ki = Ki         # ��������
		self.integral = 0    # ֱ����һ�ε����ֵ      # ��ʼ���������ۼ����Ϊ0
		self.pre_error = 0   # ��һ�ε����ֵ         # ��ʼ����һ�ε����ֵΪ0
 
	def calculate(self, setPoint, pv):       # ����һ����Ϊcalculate�ķ��������ڼ���PID�����������������2���������趨��setPoint�͹���ֵpv
		# ���� pv:process value ������ֵ��
		error = setPoint - pv           # ���    # �������ֵ
		Pout = self.Kp * error          # ������   # ������������
		self.integral += error * self.dt      # ���»������ۼ����
		Iout = self.Ki * self.integral  # ������    # ������������
		derivative = (error - self.pre_error)/self.dt    # �������ı仯�ʣ�������
		Dout = self.Kd * derivative     # ΢����   # ����΢�������
 
		output = Pout + Iout + Dout     # �µ�Ŀ��ֵ    # �µ�Ŀ��ֵΪ���������֡�΢����֮��
 
		if(output > self.max):     # �������ֵ���������ֵ
			output = self.max
		elif(output < self.min):   # �������ֵ��������Сֵ
			output = self.min
 
		self.pre_error = error         # ���汾�����Թ��´μ���
		return output               # �������ֵ


# �����ǳ��������ڣ����ڲ���PID��������Ч��
if __name__ == '__main__':
    import matplotlib.pyplot as plt     # ����matplotlib���ͼ����
    t = range(150)                            # ����ʱ������t
    pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5)      # ����һ��PID����
    val = 20                         # ��ʼ������ֵΪ20
    z = []                          # ��ʼ��һ�����б�z�����ڴ洢�趨�������ֵ֮������
    for i in t:                 # ��ÿ��ʱ�̽���ѭ��
        inc = pid.calculate(0, val)      # ����calculate��������PID���
        print("val:{} inc:{}".format(val,inc))   # ��ӡ����ֵ���������ֵ
        z.append(20-val)             # �����ֵ��ӵ��б�z
        val += inc                  # ���¹���ֵ
    plt.figure(figsize=(8,6), dpi = 80)           # ����һ����ͼ����
    plt.plot(t,z,color="blue",linewidth=1.0,linestyle="-")          # ���������ʱ��ı仯����
    plt.show()                   # ��ʾ��ͼ���
    

	#�����������һ��PID�࣬����ʵ�ּ򵥵ı���-����-΢�֣�PID����������
	# ���⣬Ҳ�ṩ��һ������ڲ��֣�ͨ��matplotlib��չʾ�˲��ø�PID���������е���ʱ����ֵ���趨��֮�����ı仯���ơ�

	#��������е�PID������������̰�����������ѧ��ʽ��

#�����㣺
#error = setPoint - pv

#�����P�����㣺
#Pout = Kp * error

#�����I�����㣺
#self.integral += error * self.dt
#Iout = Ki * self.integral

#΢���D�����㣺
#derivative = (error - self.pre_error) / self.dt
#Dout = Kd * derivative

#PID������������㣺
#output = Pout + Iout + Dout

#��Щ��ʽ�����˱���-����-΢�֣�PID���������Ļ���ԭ������Kp��Ki��Kd�ֱ��ʾ�������桢���������΢�����档
# ͨ��������Щ������������Ըı�PID�����������ܣ�ʹ����Ӧ��ͬ�Ŀ�������