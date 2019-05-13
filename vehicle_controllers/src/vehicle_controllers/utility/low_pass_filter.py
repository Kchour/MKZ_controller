#!/usr/bin/env python
import pdb
import numpy as np
import matplotlib.pyplot as plt

#### Define a Base Class
class LowPass(object):
	def __init__(self,order,cons=None):
		self.values = []
		self.filtValues=[0.0]*int(order)
		if cons == None:	
			# Set box filter by default (a very bad one)
			self.cons = [1.0/float(order)]*int(order)
		else:
			self.cons = cons
		self.order = order
		self.len_ = len(self.filtValues)

	def set_constants(self,cons):
		#### cons should be a python list []
		self.cons = cons

	def return_constants(self):
		return self.cons
	
	def update(self,valNow):
		len_ = self.len_	
		self.filtValues[1:len_]  = self.filtValues[0:len_-1]
		self.filtValues[0] = valNow

		#### Keep track of the past 100 raw values to estimate std
		if len(self.values)<100:
			#### PREPEND
			self.values.insert(0,valNow)
		else:
			self.values[1:100]  = self.values[0:99]
			self.values[0] = valNow

#### Define Child Classes which will inherit the Base Class and its methods from above
class LinearFilter(LowPass):
	
	def __init__(self,order,cons=None):
		#### We use the super keyword to avoid directly referring to the base class...
		#### In python3.0 it's a lot easier to use than in python 2.7
		super(LinearFilter, self).__init__(order,cons)

	def set_constants(self,cons):
		super(LinearFilter, self).set_constants(cons)

	def return_constants(self):
		super(LinearFilter, self).return_constants()

	def filter(self,valNow):
		# Priori Update: Latest Measurement
		super(LinearFilter,self).update(valNow)
		
		# Compute Weighted Filtered Value
		temp=0.0
		for i in range(len(self.cons)):
			temp += self.cons[i]*self.filtValues[i]
		
		# Posteriori Update: Latest filtered value 
		self.filtValues[0] = temp		
		return temp

#### Nonlinear filter test
#### Same as previous
class NonLinearFilter(LowPass):
		
	def __init__(self,order,cons=None):
		#### We use the super keyword to avoid directly referring to the base class...
		#### In python3.0 it's a lot easier to use than in python 2.7
		super(NonLinearFilter, self).__init__(order,cons)

	def set_constants(self,cons):
		super(NonLinearFilter, self).set_constants(cons)

	def return_constants(self):
		super(NonLinearFilter, self).return_constants()

	def __nonlinear_gain(self,valNow,R):
		# To prevent underestimating of the variance, use ddof=1
		std_ = np.std(self.values,ddof=1)
		if abs(valNow) <= R*std_:
			P = 1.0 - abs(valNow/(R*std_))
		else:
			P = 0.0
		return P 

	def filter(self,valNow,R=3.5):
		super(NonLinearFilter,self).update(valNow)
		P = self.__nonlinear_gain(valNow,R) #Tune R value
		self.cons[0] = 1-P


		#### Assign remaining 'filter gains' equally
		len_ = len(self.cons)
		if P <= 0.0000000001:
			vals_ = 0.0
		else:
			vals_= P/(len_-1.0)
		self.cons[1:len_]  = [vals_]*(len_-1)
		print self.cons, [vals_]*(len_-1)

		# Compute Weighted Filtered Value
		temp=0.0
		for i in range(len(self.cons)):
			temp += self.cons[i]*self.filtValues[i]
		
		# Posteriori Update: Latest filtered value 
		self.filtValues[0] = temp		
		return temp

		
#### EXAMPLE USAGE ####
if __name__=="__main__":
	lowpassFilter = LinearFilter(5,[0.292885,0.,1,2.6132,1])	#Create a 2nd order filter and define gains
							#...index 0 is weighting on the most recent value
	#lowpassFilter.set_constants([0.25,0.75])		# You can also set the gains like this	
	#lowpassFilter.set_constants([0.01,0.01,0.01,0.97])	#Setting gains for a 4th order filter
								#...MAKE SURE YOU ACTUALLY CREATE A 4th order one before this

	NLlowpassFilter = NonLinearFilter(4.0)

	#### CREATE SINE WAVE WITH NORMALLY DISTRIBUTED NOISE	
	t = np.linspace(0,2*np.pi,1000)
	y = np.sin(t)
	noise = np.random.normal(0,0.15,1000)
	yn = y+noise
	yn_filt = [0.0]*1000
	yn_filt_nonL = [0.0]*1000
	for i in range(1000):
		yn_filt[i] = lowpassFilter.filter(yn[i])
		yn_filt_nonL[i] = NLlowpassFilter.filter(yn[i],3.5)


	g = lowpassFilter.return_constants()	
	print g
	plt.figure
	plt.plot(t,y)
	plt.plot(t,yn)	
	plt.plot(t,yn_filt)
	plt.plot(t,yn_filt_nonL)
	plt.legend(['y','yn','yn_filt','yn_filt_nonL'])
	plt.show()

	


	
		
		
