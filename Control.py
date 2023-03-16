#Importación de Librerias
import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt

#Parametros del Robot
l = 1 #Tamaño del brazo
m = 1 #Masa del brazo
Ib = (1/3)*m*l**2 #Momento de inercia del braz<o
Bb = 0.01 #Coeficiente de gricción viscosa del brazo
lc = 0.5 #Distancia al centro de masa
Jm = 0.008 #Momento de inercia del eje del motor
Bm = 0.04 #Coeficiente de fricción del motor
k = 50 #Constante torsional del eje flexible
g = 9.81 #Aceleracion de la gravedad


#Ganancias del controlador
k1 = 5
k2 = 0.0001
k3 = 1
k4 = 0.5

#Vector de Tiempo de Simulación
start = 0
stop = 30
step =1e-3
t = np.arange(start,stop,step)

def f(x,t):
    
    d1 = math.exp(-3*t) #Primera perturbación del sistema
    d2 = 5*math.exp(-3*t) #Segunda perturbación del sistema
    
    r = math.sin(t) #Referencia del sistema
    
    #Errores
    e1 = x[0] - r
    e2 = x[1]
    e3 = x[2]
    e4 = x[3]
    e = [e1, e2, e3, e4] #Vector de los errores
    
    w = -1*(k1*e[0]+k2*e[1]+k3*e[2]+k4*e[3])
    u = Bm*x[3]/Jm+x[2]*k/Jm-k*x[0]/Jm+w+d2/Jm
    
    #Funciones del sistema
    dx_dt = [0, 0, 0, 0]
    dx_dt[0] = x[1]
    dx_dt[1] = (1/Ib)*(-m*g*lc*math.sin(x[0])-k*x[0]) - (Bb/Ib*x[1]) + (k/Ib*x[2]) + d1/Ib
    dx_dt[2] = x[3]
    dx_dt[3] = w
    
    return dx_dt

#Ponemos de nuevo la referencia para poder graficarla
ref = np.sin(t)

#Solución de las ecuaciones diferenciales
Solucion = odeint(f, y0 = [0, 0, 0, 0], t = t)
print('Solución', Solucion)

#Grafica

plt.plot(t,Solucion[:,0], 'b', label='Control')
plt.plot(t, ref, 'g', label='sin(t)')
plt.xlabel('Tiempo (seg)')
plt.ylabel('x1(t)')
plt.title('Angulo del Robot (rad)')
plt.grid()
plt.ylim(-1.5,1.5)
plt.show()