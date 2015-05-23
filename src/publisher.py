#!/usr/bin/env python

import rospy
import serial
from serial import SerialException
import time
import binascii

from sensor_msgs.msg import JointState # Import JointState msg to our code
from sensor_msgs.msg import Joy

from math import pi
from numpy import interp

poses = JointState()
poses.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
poses.position = [0, 0, 0, 0, 0, 0]
joint_values = [0, 0, 0, 0, 0, 0]

min_axes = [-0.5, -0.5, -0.17, -0.25, -0.25, -0.25]
max_axes = [0.5, 0.5, 0.5, 0.25, 0.25, 0.25]

nombrePuerto = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB_2.0_To_COM_Device-if00-port0" 
velocidadSerial = 57600
ad_string_r="0,0,0,0,0,0"       #string de lecturas AD que se reciben del arduino
pwm_vector=[0,0,0,0,0,0,0,0]    #string de posiciones que se envia al arduino
ad_vector=[0,0,0,0,0,0]
posc_e=[200,0,201,0,202,0,203,0,204,0,205,0,206,0,207,0] #posc_e = vector de posc a enviar, se intercalan codigos de comprobacion

arduino = serial.Serial()
arduino.port = nombrePuerto
arduino.baudrate = velocidadSerial

# Traduce los valores de 0 a 1023 leidos por serie a valores de ddp [Volts]
def a_ddp(strings):
        v = [0,0,0,0,0,0];                   
        for i in range(len(strings)-1):
                v[i] = interp(int(strings[i],10),[0,1023],[0,180])
                v[i] = v[i] * pi/180	
        return v

def enviar_serie(v):			#Funcion que envia el vector de posiciones por puerto serie al microcontrolador
    global arduino

    for i in v:
        if i<0:
            i = 0
        if i>90:
            i = 90
	
    for i in range(len(v)):				#Recorro todo el vector
        if arduino.writable(): 		#escribo solo si buffer esta disponible
	    arduino.write(chr(v[i]))

def indexar_vector(posc, posc_array):	#funcion que agrega indices al vector que se envia por serie
    for i in posc:
        i = int(i)
        if i<0: 
	    i = 0
        if i>90: 
            i = 90
    posc_array=[200,0,201,int(posc[1]),202,int(posc[2]),203,int(posc[3]),204,int(posc[4]),205,int(posc[5]),206,int(posc[6]),207,0] # Pinza y base siempre 0
	
    return posc_array

# Parte el string que se recibe desde puerto serie.
def parsear_string(ad_string):   
    #ad_string = binascii.unhexlify(ad_string)
    ad_vector = ad_string.split(',', len(ad_string))
        
    return ad_vector

# Joystick callback
def joy_callback(data):
    global pwm_vector 
    for i in range(len(ad_vector)):  
        #joint_values[i] += 0.1 * data.axes[i]
        if(((data.axes[i]>0) and (pwm_vector[i]<90)) or ((data.axes[i]<0) and (pwm_vector[i]>0))):
            pwm_vector[i] += 5 * data.axes[i]

def arm_interface():
    global posc_e, arduino, pwm_vector, ad_vector;
    
    # Configuramos nodo de ROS
    rospy.init_node('arm_interface', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("joy", Joy, joy_callback)

    global pub
    pub = rospy.Publisher('joint_states', JointState, queue_size=6)
    r = rospy.Rate(10) # 10hz 

    # Configurar el puerto serie
    try:
    	arduino.open()
    except serial.SerialException:
    	print 'El puerto ya esta abierto o bien Termi desconectado.'
        quit()

    while not rospy.is_shutdown():	#mientras que el usuario no cerro la ventana del programaes
	poses.position = joint_values
        poses.header.stamp = rospy.Time.now()

	posc_e=indexar_vector(pwm_vector,posc_e)
	enviar_serie(posc_e)
        rospy.loginfo("Enviando: %s", posc_e)

        ad_string_r = arduino.readline(); #le puse como argumento 30, o sea leer 30 bytes y tambien funciona ok
        ad_string_r = (ad_string_r.split('\r',2))[0]
        ad_vector = parsear_string(ad_string_r)
        if len(ad_vector) == 6:
            ad_vector = a_ddp(ad_vector)

            for i in range(len(ad_vector)-1):
                poses.position[i] = ad_vector[i]
            ad_vector[5] = 0 # El ultimo deberia andar, pero no anda 

            rospy.loginfo(poses)
            pub.publish(poses)
 	
        r.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        arm_interface()
    except rospy.ROSInterruptException: pass
