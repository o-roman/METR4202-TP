import numpy as np



def invers(x,y,z_init):
    l0 = 10
    z = z_init-l0
    l1 = 11.75
    l2 = 9.5
    l3 = 11
    i =0
    j0_pi = np.arctan2(y,x)
    a = x/np.cos(j0_pi)
    if (x ==0):
        a = y
    b = z

    for j1 in range(-90,90):
        j1_pi = j1 *(np.pi/180)
        j3_pi = np.arccos((a*a + b*b +l1*l1 -l2*l2 -l3*l3 -2*a*l1*np.sin(j1_pi)-2*b*l1*np.cos(j1_pi))/(2*l2*l3))
        m = l2 * np.sin(j1_pi) + l3*np.sin(j1_pi)*np.cos(j3_pi) + l3*np.cos(j1_pi)*np.sin(j3_pi)
        n = l2 * np.cos(j1_pi) + l3 * np.cos(j1_pi) * np.cos(j3_pi) + l3 * np.sin(j1_pi) * np.sin(j3_pi)
        t = a -l1*np.sin(j1_pi)
        p =pow ((n*n + m*m),0.5)
        q = np.arcsin(m/p)
        j2_pi = np.arcsin(t/p) -q

        x1 = (l1*np.sin(j1_pi) + l2*np.sin(j1_pi + j2_pi) + l3*np.sin(j1_pi+j2_pi+j3_pi))*np.cos(j0_pi)
        y1 = (l1 * np.sin(j1_pi) + l2 * np.sin(j1_pi + j2_pi) + l3 * np.sin(j1_pi + j2_pi + j3_pi)) * np.sin(j0_pi)
        z1 = l1 * np.cos(j1_pi) + l2 * np.cos(j1_pi + j2_pi) + l3 * np.cos(j1_pi + j2_pi + j3_pi)
        theta1 = j1_pi
        theta2 = j2_pi
        theta0 = j0_pi
        theta3 = j3_pi
        '''''
        if abs(theta2) < 0.1:
           continue
           '''''
        if x1 < (x + 1) and x1 > (x - 1) and y1 < (y + 1) and y1 > (y - 1) and z1 < (z + 1) and z1 > (z - 1):
            print("joint0:", theta0)
            print("joint1:", theta1)
            print("joint2:", theta2)
            print("joint3:", theta3)
            print(x1, y1, z1+l0)
            print()
            i = 1


    for j1 in range(-90, 90):
        j1_pi = j1 * (np.pi / 180)
        j3_pi = np.arccos((a * a + b * b + l1 * l1 - l2 * l2 - l3 * l3 - 2 * a * l1 * np.sin( j1_pi) - 2 * b * l1 * np.cos(j1_pi)) / (2 * l2 * l3))
        m = l2 * np.sin(j1_pi) + l3 * np.sin(j1_pi) * np.cos(j3_pi) + l3 * np.cos(j1_pi) * np.sin(j3_pi)
        n = l2 * np.cos(j1_pi) + l3 * np.cos(j1_pi) * np.cos(j3_pi) + l3 * np.sin(j1_pi) * np.sin(j3_pi)
        t = a - l1 * np.sin(j1_pi)
        p = pow((n * n + m * m), 0.5)
        q = np.arcsin(m / p)
        j2_pi = -np.arcsin(t / p) - q

        x1 = (l1 * np.sin(j1_pi) + l2 * np.sin(j1_pi + j2_pi) + l3 * np.sin(j1_pi + j2_pi + j3_pi)) * np.cos(j0_pi)
        y1 = (l1 * np.sin(j1_pi) + l2 * np.sin(j1_pi + j2_pi) + l3 * np.sin(j1_pi + j2_pi + j3_pi)) * np.sin(j0_pi)
        z1 = l1 * np.cos(j1_pi) + l2 * np.cos(j1_pi + j2_pi) + l3 * np.cos(j1_pi + j2_pi + j3_pi)
        theta1 = j1_pi
        theta2 = j2_pi
        theta0 = j0_pi
        theta3 = j3_pi
        '''''
        if abs(theta2) < 0.1:
            continue
            '''
        if x1 < (x + 1) and x1 > (x - 1) and y1 < (y + 1) and y1 > (y - 1) and z1 < (z + 1) and z1 > (z - 1):
            print("joint0:", theta0)
            print("joint1:", theta1)
            print("joint2:", theta2)
            print("joint3:", theta3)
            print(x1, y1, z1+l0)
            print()
            i = 1

        '''''
        theta1 = j1_pi*(180/np.pi)
        theta2 = j2_pi*(180/np.pi)
        theta0 = j0_pi*(180/np.pi)
        theta3 = j3_pi*(180/np.pi)
        '''

    if i==0:
        print("no answer")


invers(10,1,10)


# joint4 in range from -pi/2 <= theta4 <= pi/2
# joint3 (-2.4,2.5)
#joint2 (-2,2.25)