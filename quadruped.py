import argparse
import math
import pybullet as p
import matplotlib.pyplot as plt
from time import sleep
import time

''' Rappels 
x = axe rouge
y = axe vert
z = axe bleu
1 case = 100
'''

#temps
dt = 0.01

#dimensions des pattes
L1 = 4
L2 = 4.5
L3 = 6.5
L4 = 8.7

#angles trigo
leg_1_angle = math.pi/4
leg_2_angle = math.pi/4 + math.pi/2
leg_3_angle = -1 * math.pi/4 - 1 * math.pi/2
leg_4_angle = -1 * math.pi/4


#rayons accessibles par une patte dans son repere
rayon_max_leg = L2 + L3 + L4
rayon_min_leg = 2 * L2

#valeurs min et max des sliders dans le repere de la patte
x_min_leg = L1
x_max_leg = rayon_max_leg
x_default_leg = (x_min_leg + x_max_leg) / 2
y_min_leg = -rayon_max_leg
y_max_leg = rayon_max_leg
y_default_leg = (y_min_leg + y_max_leg) / 2
z_min_leg = -rayon_max_leg
z_max_leg = rayon_max_leg
z_default_leg = (z_min_leg + z_max_leg) / 2

#rayons accessibles par le robot dans son repere
rayon_max_robot = 5
rayon_min_robot = -5

#valeurs min et max des sliders dans le repère du robot
coef_pos_base = 3.5
x_min_robot = rayon_min_robot
x_max_robot = rayon_max_robot
x_default_robot = (x_min_robot + x_max_robot) / 2
y_min_robot = rayon_min_robot
y_max_robot = rayon_max_robot
y_default_robot = (y_min_robot + y_max_robot) / 2
z_min_robot = 0
z_max_robot = 2 * rayon_max_robot
z_default_robot = (z_min_robot + z_max_robot) / 2

max_angle = 2
rayon_max_repere = L1+L2+L3+L4

angle_repere = 3*math.pi/4
default_angle = 1
previous_theta = default_angle

default_periode = 1
def init():
    """Initialise le simulateur
    
    Returns:
        int -- l'id du robot
    """
    # Instanciation de Bullet
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # Chargement du sol
    planeId = p.loadURDF('plane.urdf')

    # Chargement du robot
    startPos = [0, 0, 0.1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = p.loadURDF("./quadruped/robot.urdf",
                        startPos, startOrientation)

    p.setPhysicsEngineParameter(fixedTimeStep=dt)
    return robot

def setJoints(robot, joints):
    """Définis les angles cibles pour les moteurs du robot
    
    Arguments:
        int -- identifiant du robot
        joints {list} -- liste des positions cibles (rad)
    """
    jointsMap = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for k in range(len(joints)):
        jointInfo = p.getJointInfo(robot, jointsMap[k])
        p.setJointMotorControl2(robot, jointInfo[0], p.POSITION_CONTROL, joints[k])

def demo(t, amplitude):
    """Démonstration de mouvement (fait osciller une patte)
    
    Arguments:
        t {float} -- Temps écoulé depuis le début de la simulation
    
    Returns:
        list -- les 12 positions cibles (radian) pour les moteurs
        float -- amplitude de l'oscillation
    """
    joints = [0]*12
    joints[0] = math.sin(t) * amplitude
    joints[3] = math.sin(t) * amplitude
    joints[7] = math.sin(t) * amplitude
    return joints

#Coord sont dans le repère de la patte
def pos_to_angle_leg(x, y, z):

    # Theta zéro
    t0 = math.atan2(y, x)

    # Theta un
    x1 = L2 * math.cos(t0)
    y1 = L2 * math.sin(t0)
    z1 = 0
    temp = (x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2
    cos_value = (L3 ** 2 + L4 ** 2 - temp) / (2 * L3 * L4)
    if cos_value < -1:
        #print("Inférieur à -1 pour t1")
        cos_value = -1
    elif cos_value > 1:
        #print("Supérieur à 1 pour t1")	
        cos_value = 1
    t2 = math.pi - math.acos(cos_value)

    # Theta deux
    beta = math.asin(z / math.sqrt(temp))
    cos_value_t2 = (L3 ** 2 + temp - L4 ** 2) / (2 * L3 * math.sqrt(temp))
    if cos_value_t2 < -1:
        #print("Inférieur à -1 pour t2")
        cos_value_t2 = -1
    elif cos_value_t2 > 1:
        #print("Supérieur à 1 pour t2")	
        cos_value_t2 = 1
    alpha = math.acos(cos_value_t2)
    t1 = alpha + beta

    return [t0, t1, t2]

#valeurs possibles dans le demi-disque de rayon=L2+L3+L4
def is_min_legal_position(x,y,z):
	#(x-xc)² + (y-yc)² + (z-zc)² < R²
    distance = x*x + y*y + z*z
    xc, yc, zc = 0, 0, 0
    if distance > rayon_min_leg*rayon_min_leg:
         return True
    return False
    
def is_max_legal_position(x,y,z):
	#(x-xc)² + (y-yc)² + (z-zc)² < R²
    distance = x*x + y*y + z*z
    xc, yc, zc = 0, 0, 0
    if distance < rayon_max_leg*rayon_max_leg:
         return True
    return False

def leg_ik(x,y,z):
    joints = starting_position()
    #si pos < sphere min alors on fixe x
    if not is_min_legal_position(x,y,z):
        x = rayon_min_leg
    elif not is_max_legal_position(x,y,z):
        x = x - 0.2
    return pos_to_angle_leg(x,y,z) 

def rotateZ(x, y, z, theta):
    return (
        x * math.cos(theta) - y * math.sin(theta),
        x * math.sin(theta) + y * math.cos(theta),
        z,
    )

#Coord dans le repère absolu
def pos_to_angle_leg_abs(x, y, z, angle):
    # Translation
    x = x - L1 * math.cos(angle)
    y = y - L1 * math.sin(angle)
    z = z
    # Rotation
    x, y, z = rotateZ(x, y, z, -angle)
    return pos_to_angle_leg(x, y, z)

def move_leg(x,y,z,leg_id):
	if leg_id == 1:
		return pos_to_angle_leg_abs(x,y,z,leg_1_angle)
	elif leg_id == 2:
		return pos_to_angle_leg_abs(x,y,z,leg_2_angle)
	elif leg_id == 3:
		return pos_to_angle_leg_abs(x,y,z,leg_3_angle)
	elif leg_id == 4:
		return pos_to_angle_leg_abs(x,y,z,leg_4_angle)
	else:
		print("Error in leg id")

def abs_to_angle(coord_abs):
    return (move_leg(*coord_abs[:3],1)
          + move_leg(*coord_abs[3:6],2)
          + move_leg(*coord_abs[6:9],3)
          + move_leg(*coord_abs[9:],4)
    )

def starting_position():
    z = -z_default_robot
    starting_pos = [
		#patte en haut a droite
        L1 * math.cos(leg_1_angle) * coef_pos_base,
        L1 * math.sin(leg_1_angle) * coef_pos_base,
        z,
        #patte en haut à gauche
        L1 * math.cos(leg_2_angle) * coef_pos_base,
        L1 * math.sin(leg_2_angle) * coef_pos_base,
        z,
        #patte en bas a gauche
        L1 * math.cos(leg_3_angle) * coef_pos_base,
        L1 * math.sin(leg_3_angle) * coef_pos_base,
        z,
        #patte en bas a droite
        L1 * math.cos(leg_4_angle) * coef_pos_base,
        L1 * math.sin(leg_4_angle) * coef_pos_base,
        z,
    ]
    
    return starting_pos

def body_forward_y(x,y,z):
    y_before = y
    y = coef_pos_base - y
    pos = [
            L1 * math.cos(leg_1_angle) * y,
            L1 * math.sin(leg_1_angle) * coef_pos_base,
            z,
            L1 * math.cos(leg_2_angle) * coef_pos_base - y_before,
            L1 * math.sin(leg_2_angle) * coef_pos_base + y_before,
            z,
            L1 * math.cos(leg_3_angle) * coef_pos_base - y_before,
            L1 * math.sin(leg_3_angle) * coef_pos_base - y_before,
            z,
            L1 * math.cos(leg_4_angle) * y,
            L1 * math.sin(leg_4_angle) * coef_pos_base,
            z,
            ]
    return pos;

def body_forward_x(x,y,z):
    x_before = x
    x = coef_pos_base - x
    pos = [
            L1 * math.cos(leg_1_angle) * coef_pos_base + x_before,
            L1 * math.sin(leg_1_angle) * coef_pos_base + x_before,
            z,
            L1 * math.cos(leg_2_angle) * coef_pos_base - x_before,
            L1 * math.sin(leg_2_angle) * coef_pos_base + x_before,
            z,
            L1 * math.cos(leg_3_angle) * coef_pos_base,
            L1 * math.sin(leg_3_angle) * x,
            z,
            L1 * math.cos(leg_4_angle) * coef_pos_base,
            L1 * math.sin(leg_4_angle) * x,
            z,
            ]
    return pos
    
    
# Toutes les fonctions body imprécises sans interpolation    
def body_backward_y(x,y,z):
    y_before = y
    y_temp = -y
    y = coef_pos_base - y_temp
    pos = [
            L1 * math.cos(leg_1_angle) * coef_pos_base - y_before,
            L1 * math.sin(leg_1_angle) * coef_pos_base - y_before,
            z,
            L1 * math.cos(leg_2_angle) * y,
            L1 * math.sin(leg_2_angle) * coef_pos_base,
            z,
            L1 * math.cos(leg_3_angle) * y,
            L1 * math.sin(leg_3_angle) * coef_pos_base,
            z,
            L1 * math.cos(leg_4_angle) * coef_pos_base - y_before,
            L1 * math.sin(leg_4_angle) * coef_pos_base + y_before,
            z,
            ]
    return pos
  
def body_backward_x(x,y,z):
    x_before = x
    x_temp = -x
    x = coef_pos_base - x_temp
    pos = [
            L1 * math.cos(leg_1_angle) * coef_pos_base ,
            L1 * math.sin(leg_1_angle) * x,
            z,
            L1 * math.cos(leg_2_angle) * coef_pos_base ,
            L1 * math.sin(leg_2_angle) * x,
            z,
            L1 * math.cos(leg_3_angle) * coef_pos_base + x_before,
            L1 * math.sin(leg_3_angle) * coef_pos_base + x_before,
            z,
            L1 * math.cos(leg_4_angle) * coef_pos_base - x_before,
            L1 * math.sin(leg_4_angle) * coef_pos_base + x_before,
            z,
            ]
    return pos
	
#demo corps robot avec fonctions imprécises
def demo_robot_ik(t,x,y,z):
    z = -z
    coef = 3.5
    t = math.fmod(t, 8)
    if t >= 0 and t < 1:
        pos = body_forward_x(3,y,z)
    if t >= 1 and t < 2:
        pos = starting_position()
    if t >= 2 and t < 3:
        pos = body_backward_x(-3,y,z)
    if t >= 3 and t < 4:
        pos = starting_position()
    if t >= 4 and t < 5:
        pos = body_forward_y(x,3,z)
    if t >= 5 and t < 6:
        pos = starting_position()
    if t >= 6 and t < 7:
        pos = body_backward_y(x,-3,z)
    if t >= 7 and t < 8:
        pos = starting_position()
    
    return abs_to_angle(pos)

def robot_ik_imprecis(x,y,z):
    z = -z
    if x == 0 and y==0:
        print("x=0 et y=0")
        return abs_to_angle(starting_position())
    if x > 0:
        print("x>0")
        return abs_to_angle(body_forward_x(x,y,z))
    if y > 0:
        print("y>0")
        return abs_to_angle(body_forward_y(x,y,z))
    if x < 0:
        print("x<0")
        return abs_to_angle(body_backward_x(x,y,z))
    if y < 0:
        print("y<0")
        return abs_to_angle(body_backward_y(x,y,z))

def additioner_tab(tab1,tab2):
    if (len(tab1) != len(tab2)):
        print("Pas la meme taille")
    target = [0]*len(tab1)
    for i in range(len(tab1)):
        target[i] = tab1[i] + tab2[i]
    return target

def robot_ik_precis(x,y,z):
    #inverser les x et y pour etre dans le plan
    z = -z
    y = -y
    pos = [y,x,z,
           y,x,z,
           y,x,z,
           y,x,z]
    return abs_to_angle(additioner_tab(starting_position(),pos))

def tourner_imprecis(t,angle):
    z = -z_default_robot
    angle = -angle
    minus = coef_pos_base - angle
    maxi = coef_pos_base + angle
    t = math.fmod(t, 1)
    if t >= 0 and t < 0.25:
        pos = [
            L1 * math.cos(leg_1_angle) * minus,
            L1 * math.sin(leg_1_angle) * maxi,
            z,
            L1 * math.cos(leg_2_angle) * minus,
            L1 * math.sin(leg_2_angle) * maxi,
            0,
            L1 * math.cos(leg_3_angle) * minus,
            L1 * math.sin(leg_3_angle) * maxi,
            0,
            L1 * math.cos(leg_4_angle) * minus,
            L1 * math.sin(leg_4_angle) * maxi,
            z,
            ]
    if t >= 0.25 and t < 0.5:
        pos = [
            L1 * math.cos(leg_1_angle) * minus,
            L1 * math.sin(leg_1_angle) * maxi,
            0,
            L1 * math.cos(leg_2_angle) * minus,
            L1 * math.sin(leg_2_angle) * maxi,
            z,
            L1 * math.cos(leg_3_angle) * minus,
            L1 * math.sin(leg_3_angle) * maxi,
            0,
            L1 * math.cos(leg_4_angle) * minus,
            L1 * math.sin(leg_4_angle) * maxi,
            z,
            ]
    if t >= 0.5 and t < 0.75:
        pos = [
            L1 * math.cos(leg_1_angle) * maxi,
            L1 * math.sin(leg_1_angle) * minus,
            0,
            L1 * math.cos(leg_2_angle) * maxi,
            L1 * math.sin(leg_2_angle) * minus,
            z,
            L1 * math.cos(leg_3_angle) * maxi,
            L1 * math.sin(leg_3_angle) * minus,
            0,
            L1 * math.cos(leg_4_angle) * maxi,
            L1 * math.sin(leg_4_angle) * minus,
            z,
            ]
    if t >= 0.75 and t < 1:
        pos = [
            L1 * math.cos(leg_1_angle) * maxi,
            L1 * math.sin(leg_1_angle) * minus,
            z,
            L1 * math.cos(leg_2_angle) * maxi,
            L1 * math.sin(leg_2_angle) * minus,
            0,
            L1 * math.cos(leg_3_angle) * maxi,
            L1 * math.sin(leg_3_angle) * minus,
            z,
            L1 * math.cos(leg_4_angle) * maxi,
            L1 * math.sin(leg_4_angle) * minus,
            0,
            ]
   
    return abs_to_angle(pos)

def reverse_tab(tab):
	for i in range(len(tab)):
		tab[i] = -tab[i]
	return tab

def interpolate(ts, ys, t):
    for i in range(len(ts)):
        if ts[i] >= t:
            t1, t2 = ts[i - 1], ts[i]
            y1, y2 = ys[i - 1], ys[i]
            dt = t2 - t1
            p = (t - t1) / dt 
            return y1 * (1 - p) + y2 * p
    return 0

def interpolate3D(t,xs,ys,zs,ts):
    return [interpolate(ts, xs, t), interpolate(ts, ys, t), interpolate(ts, zs, t)]

def demo_body(t):
    z = -z_default_robot
    periode = 4
    coef = 5
    ys = [coef,   0,-coef,    0,coef,   0,-coef,   0]
    xs = [0   ,coef,    0,-coef,   0,coef,   0,-coef]    
    zs = [0,0,z,z,z,0,0,0]
    ts = [0,0.5,1,1.5,2,2.5,3,3.5]

    #Période et décalage entre paire de pattes
    t = math.fmod(t, 3.5)
    
    patte1 = interpolate3D(t,xs,ys,zs,ts)
    patte2 = interpolate3D(t,xs,ys,zs,ts)
    patte3 = interpolate3D(t,xs,ys,zs,ts)
    patte4 = interpolate3D(t,xs,ys,zs,ts)

     
    interpolate_pos =  patte1 + patte2 + patte3 + patte4
    return abs_to_angle(additioner_tab(starting_position(),interpolate_pos))

def tourner_precis(t,t_speed,periode):
    
    if t_speed == 1:
        z = 0
    else:
        z = -5
	# point mort pi
    angle = math.pi/t_speed
        
    target = L1* math.sin(angle)*coef_pos_base
    opposed_target = L1* math.sin(angle+math.pi)*coef_pos_base
    keep = [0,0,0,0]
    xs = keep
    ys = [0,0,target,target]   
    ys_opposed = [0,0,opposed_target,opposed_target]  
    zs = [0,z,z,0]
    ts = [0,periode/3,periode/2,periode]

    #Période et décalage entre paire de pattes
    t = math.fmod(t, periode)
    offset = periode/3

    patte1 = interpolate3D(t,xs,ys_opposed,zs,ts)
    patte2 = interpolate3D(t + offset,keep,keep,zs,ts)
    patte3 = interpolate3D(t,xs,ys,zs,ts)
    patte4 = interpolate3D(t + offset,keep,keep,zs,ts)

     
    interpolate_pos =  patte1 + patte2 + patte3 + patte4

    return abs_to_angle(additioner_tab(starting_position(),interpolate_pos))

def deg_to_radian(angle):
    return (angle*math.pi)/180

	
def walk_imprecis(t,x,y,angle):
    z = -angle
    coef = 3.5
    t = math.fmod(t, 4)
    if t >= 0 and t < 1:
        pos = [
            L1 * math.cos(leg_1_angle) * 3.5,
            L1 * math.sin(leg_1_angle) * 3.5,
            z,
            L1 * math.cos(leg_2_angle) * 3.5,
            L1 * math.sin(leg_2_angle) * 3.5,
            z,
            L1 * math.cos(leg_3_angle) * 3.5,
            L1 * math.sin(leg_3_angle) * 3.5,
            z,
            L1 * math.cos(leg_4_angle) * 1,
            L1 * math.sin(leg_4_angle) * 5,
            0,
            ]
    if t >= 1 and t < 2:
        pos = [
            L1 * math.cos(leg_1_angle) * 3.5,
            L1 * math.sin(leg_1_angle) * 3.5,
            z,
            L1 * math.cos(leg_2_angle) * 3.5,
            L1 * math.sin(leg_2_angle) * 3.5,
            z,
            L1 * math.cos(leg_3_angle) * 1,
            L1 * math.sin(leg_3_angle) * 5,
            0,
            L1 * math.cos(leg_4_angle) * 1,
            L1 * math.sin(leg_4_angle) * 5,
            z,
            ]
    if t >= 2 and t < 3:
        pos = [
            L1 * math.cos(leg_1_angle) * 4,
            L1 * math.sin(leg_1_angle) * 1,
            z,
            L1 * math.cos(leg_2_angle) * 3.5,
            L1 * math.sin(leg_2_angle) * 3.5,
            z,
            L1 * math.cos(leg_3_angle) * 3.5,
            L1 * math.sin(leg_3_angle) * 3.5,
            z,
            L1 * math.cos(leg_4_angle) * 5,
            L1 * math.sin(leg_4_angle) * 3,
            0,
            ]
    if t >= 3:
        pos = [
            L1 * math.cos(leg_1_angle) * 1,
            L1 * math.sin(leg_1_angle) * 3.3,
            z,
            L1 * math.cos(leg_2_angle) * 3.5,
            L1 * math.sin(leg_2_angle) * 3.5,
            0,
            L1 * math.cos(leg_3_angle) * 1,
            L1 * math.sin(leg_3_angle) * 5,
            z,
            L1 * math.cos(leg_4_angle) * 5,
            L1 * math.sin(leg_4_angle) * 3,
            z,
            ]            
    '''        
    if t >= 3 and t <= 4:
        pos = [
            L1 * math.cos(leg_1_angle) * 1,
            L1 * math.sin(leg_1_angle) * 5,
            z,
            L1 * math.cos(leg_2_angle) * 4,
            L1 * math.sin(leg_2_angle) * 1,
            0,
            L1 * math.cos(leg_3_angle) * 1,
            L1 * math.sin(leg_3_angle) * 5,
            z,
            L1 * math.cos(leg_4_angle) * 5,
            L1 * math.sin(leg_4_angle) * 3,
            z,
            ]
            '''
    return abs_to_angle(pos)

def walk(t,x_speed,y_speed,t_speed,periode,angle):
    global previous_theta
    if previous_theta == t_speed:
        previous_angle = t_speed
        z = -2
        #periode entre 1 et 10
        default_periode = 1.5
        periode = default_periode - periode/10
    
        #3pi/4 pour être dans le repère global
        #angle = 3*math.pi/4
    
        #On inverse tout car les axes sont inversés de base
        target_x = L1* math.cos(angle)*x_speed
        target_y = L1* math.sin(angle)*y_speed
        
        #Description du mouvement en absolu
        keep = [0,0,0,0]
        xs = [0,0,-target_y,-target_y]   
        ys = [0,0,-target_x,-target_x]    
        zs = [0,z,z,0]
        ts = [0,periode/3,periode/2,periode]

        #Période et décalage entre paire de pattes
        t = math.fmod(t, periode)
        offset = periode/3

        patte1 = interpolate3D(t,xs,ys,zs,ts)
        patte2 = interpolate3D(t + offset,xs,ys,zs,ts)
        patte3 = interpolate3D(t,xs,ys,zs,ts)
        patte4 = interpolate3D(t + offset,xs,ys,zs,ts)

        interpolate_pos =  patte1 + patte2 + patte3 + patte4

        return abs_to_angle(additioner_tab(starting_position(),interpolate_pos))
    else:
        previous_angle = t_speed
        return tourner_precis(t,t_speed,periode)

def pythagore(x,y):
    return math.sqrt(x*x + y*y)

def goto(t,x_target,y_target,t_target,periode,chrono):
    periode = 0.75
    speed_max = 2;
    
    if abs(x_target) == 0:
        x_speed = 0
        y_speed = speed_max
    elif abs(y_target) == 0:
        x_speed = speed_max;
        y_speed = 0;
    else:
        value_min = min(x_target,y_target)
        value_max = max(x_target,y_target)
        percent = (100 * value_min)/value_max
        if x_target == value_max:
            x_speed = speed_max
            y_speed = speed_max * (percent/100)
        else:
            x_speed = speed_max * (percent/100)
            y_speed = speed_max;
    
    #quart en haut a droite (base du repere)
    if x_target >= 0 and y_target >= 0:
        angle = 3*math.pi/4
    #quart en haut a gauche (base du repere)
    elif x_target <= 0 and y_target >= 0:
        angle = math.pi/4
        if not(abs(x_target) == 0 or abs(y_target) == 0):
            x_speed = -x_speed
    #quart en bas a gauche (base du repere)
    elif x_target <= 0 and y_target <= 0:
        angle = -math.pi/4
    #quart en bas a droite (base du repere)
    elif x_target >= 0 and y_target <= 0:
        angle = -3 * math.pi/4
        if not(abs(x_target) == 0 or abs(y_target) == 0):
            y_speed = -y_speed

    
    #t = 13 pour 1m en 24s donc 1.85 de coef 
    #0.077m/t ou 7.7 cm/t sur ligne droite
    t_speed = 1
    
    distance = pythagore(x_target,y_target)
    delta_t = distance * 0.125
    if t < delta_t:
        #print(t,chrono)
        return walk(t,x_speed,y_speed,t_speed,periode,angle)
    elif t >= delta_t and t < delta_t + periode*t_target:
        return tourner_imprecis(t,math.pi/4)
    else:
        return [0]*12

#mode fun on fait comme on peut
def twerk(t,amplitude):
    x = 8
    z = -z_default_robot
    y = -math.cos(t) *3
    t = math.fmod(t, 0.5)
    if (t < 0.25):
        z1 = -z
        z2 =  z + amplitude
    else:
        z1 = z-amplitude
        z2 = 0
    pos = [y,x,z1,
           y,x,z1,
           y,x,z2,
           y,x,z2]
    
    return abs_to_angle(additioner_tab(starting_position(),pos))

if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser(prog="TD Robotique S8")
    parser.add_argument('-m', type=str, help='Mode', required=True)
    parser.add_argument('-x', type=float, help='X target for goto (m)', default=1.0)
    parser.add_argument('-y', type=float, help='Y target for goto (m)', default=0.0)
    parser.add_argument('-t', type=float, help='Theta target for goto (rad)', default=0.0)
    args = parser.parse_args()

    mode, x_target, y_target, t_target = args.m, args.x, args.y, args.t
    
    if mode not in ['demo', 'leg_ik', 'robot_ik', 'demo_body', 'walk', 'goto', 'fun']:
        print('Le mode %s est inconnu' % mode)
        exit(1)

    robot = init()
    if mode == 'demo':
        amplitude = p.addUserDebugParameter("amplitude", 0.1, 1, 0.3)
        print('Mode de démonstration...')
    elif mode == 'leg_ik':
        x_slider = p.addUserDebugParameter("x", x_min_leg, x_max_leg, x_default_leg)
        y_slider = p.addUserDebugParameter("y", y_min_leg, y_max_leg, y_default_leg)
        z_slider = p.addUserDebugParameter("z", z_min_leg, z_max_leg, z_default_leg)
        print('Mode leg_ik')
    elif mode == 'robot_ik':
        x_slider = p.addUserDebugParameter("x", x_min_robot, x_max_robot, x_default_robot)
        y_slider = p.addUserDebugParameter("y", y_min_robot, y_max_robot, y_default_robot)
        z_slider = p.addUserDebugParameter("z", z_min_robot, z_max_robot, z_default_robot)
        print('Mode robot_ik...')
    elif mode == 'demo_body':
        print('Demo body')
    elif mode == 'walk':
        x_speed_slider = p.addUserDebugParameter("x_speed (en m/s)", 0, 2, 0)
        y_speed_slider = p.addUserDebugParameter("y_speed (en m/s)", 0, 2, 0)
        t_speed_slider = p.addUserDebugParameter("t_speed (en rad/s)", 1, 2, default_angle)
        periode = p.addUserDebugParameter("Global speed", 1, 10, 1)
        print('Mode walk...')	
    elif mode == 'goto':
        print('Mode goto...')	
    elif mode == 'fun':
        amplitude = p.addUserDebugParameter("Amplitude", 0, 5, 0)
        
        print("Let's twerk...")
    else:
        raise Exception('Mode non implémenté: %s' % mode)
    
    t = 0
    
    temps = time.time()
    # Boucle principale
    while True:
        t += dt
        chrono = time.time() - temps

        if mode == 'demo':
            # Récupération des positions cibles
            joints = demo(t, p.readUserDebugParameter(amplitude))
        elif mode == 'leg_ik':
            joints = leg_ik(p.readUserDebugParameter(x_slider),
                               p.readUserDebugParameter(y_slider),
                               p.readUserDebugParameter(z_slider))
        elif mode == 'robot_ik':
            joints = robot_ik_precis(p.readUserDebugParameter(x_slider),
                               p.readUserDebugParameter(y_slider),
                               p.readUserDebugParameter(z_slider))
        elif mode == 'demo_body':
            joints = demo_body(t)
        elif mode == 'walk':
            joints = walk(t, p.readUserDebugParameter(x_speed_slider),
                               p.readUserDebugParameter(y_speed_slider),
                               p.readUserDebugParameter(t_speed_slider),
                               p.readUserDebugParameter(periode),
                               angle_repere
                               )
        elif mode == 'goto':
            joints = goto(t,x_target,y_target,t_target,default_periode,chrono)
        elif mode == 'fun':
            joints = twerk(t,p.readUserDebugParameter(amplitude))
        try: 
            joints
        except NameError:
            print('No joints! Exiting ...')
            exit();
        else:
            # Envoi des positions cibles au simulateur
            setJoints(robot, joints)

            # Mise à jour de la simulation
            p.stepSimulation()
            sleep(dt)
