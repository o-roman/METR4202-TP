import math

import numpy as np

'''''
  z = z_init -10
  h = 10
  L = 11.75
  M = 9.5
  N = 11
  R = x*x + y*y
  s = R - N
  Q = pow((s*s + z*z),0.5)
  f = np.arctan2(z,s)
  g = np.arccos((L*L + Q*Q - M*M)/(2*L*Q))
  a = f + g
  b = np.arccos((M*M + L*L - Q*Q)/(2*L*M))
  c = -b - a + 2*np.pi
  d = math.atan2(x,y)
  print("joint 0:",d)
  print("joint 1:",a)
  print("joint 2:",b)
  print("joint 3:",c)
  '''''
import numpy as np
import matplotlib.pyplot as plt

def sample():
    r = 12
    # 2.圆心坐标
    a=0
    b = 19
    m =[0 for i in range(60)]
    m[0] = -15
    for i in range(1,60):
        m[i] = m[i-1] + 0.5
    print(m)
    n = [0 for i in range(60)]
    n[0] = 5
    for i in range(1, 60):
        n[i] = n[i - 1] + 0.5
    print(n)
    mat = np.zeros(shape=(1793,3))
    for i in range(1793):
        mat[i][2] = 1.5
    flag=0
    # ==========================================
    # 方法一：参数方程
    theta = np.arange(0, 2 * np.pi, 0.01)
    x = a + r * np.cos(theta)
    y = b + r * np.sin(theta)
    fig = plt.figure()
    axes = fig.add_subplot(111)
    plt.ylim(0,45)
    for i in range(60):
        for j in range(60):
           #result = any(np.sqrt((m[i]-a)*(m[i]-a)+(n[j]-b)*(n[j]-b))<=12)
           if np.sqrt((m[i]-a)*(m[i]-a)+(n[j]-b)*(n[j]-b))<=12:
              plt.scatter(m[i],n[j])
              mat[flag][0] = m[i]
              mat[flag][1] = n[j]
              flag=flag+1
    print("flag:",flag)
    print(mat)
    plt.draw()
    axes.plot(x, y)
    axes.axis('equal')
    plt.title('www.ai8py.com')
    plt.show()
    return mat
def invers_two( ):
    mat = sample()
    joint_mat = np.zeros(shape=(1793,7))
    num = 0
    for i in range(1793):
        flag = 0

        l0 = 10
        x = mat[i][0]
        y = mat[i][1]
        z = mat[i][2]-l0

        l1 = 11.75
        l2 = 9.5
        l3 = 11
        j0_pi = np.arctan2(y, x)
        a = x / np.cos(j0_pi)
        if (x == 0):
            a = y
        b = z
        for j1 in range(0, 90):
            j1_pi = j1 * (np.pi / 180)
            if flag == 1:
                break
            for j2 in range(0,150):
                j2_pi = j2 * (np.pi / 180)
                if flag == 1:
                    break
                for j3 in range(0,90):
                    j3_pi = j3 *(np.pi/180)

                    x1 = (l1 * np.sin(j1_pi) + l2 * np.sin(j1_pi + j2_pi) + l3 * np.sin(j1_pi + j2_pi + j3_pi)) * np.cos(
                        j0_pi)
                    y1 = (l1 * np.sin(j1_pi) + l2 * np.sin(j1_pi + j2_pi) + l3 * np.sin(j1_pi + j2_pi + j3_pi)) * np.sin(
                        j0_pi)
                    z1 = l1 * np.cos(j1_pi) + l2 * np.cos(j1_pi + j2_pi) + l3 * np.cos(j1_pi + j2_pi + j3_pi)
                    theta1 = j1_pi
                    theta2 = j2_pi
                    theta0 = j0_pi
                    theta3 = j3_pi
                    '''''
                    if abs(theta2) < 0.1:
                       continue
                       '''''
                    if x1 < (x + 1) and x1 > (x -1 ) and y1 < (y + 1) and y1 > (y - 1) and z1 < (z + 0.5) and z1 > (z - 0.5):
                        joint_mat[i][0] = theta0
                        joint_mat[i][1] = theta1
                        joint_mat[i][2] = theta2
                        joint_mat[i][3] = theta3
                        joint_mat[i][4] = mat[i][0]
                        joint_mat[i][5] = mat[i][1]
                        joint_mat[i][6] = mat[i][2]
                        print(joint_mat[i])
                        num = num+1
                        print(num)
                        '''''
                        print("joint0:", theta0)
                        print("joint1:", theta1)
                        print("joint2:", theta2)
                        print("joint3:", theta3)
                        print("joint0:", theta0 * (180 / np.pi))
                        print("joint1:", theta1 * (180 / np.pi))
                        print("joint2:", theta2 * (180 / np.pi))
                        print("joint3:", theta3 * (180 / np.pi))
                        print(x1, y1, z1 + l0)
                        print()
                        '''
                        flag =1
                        if flag ==1:
                            break
    np.savetxt(r'test.txt', joint_mat, fmt='%s', delimiter=',')


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
            print("joint0:", theta0* (180 / np.pi))
            print("joint1:", theta1* (180 / np.pi))
            print("joint2:", theta2* (180 / np.pi))
            print("joint3:", theta3* (180 / np.pi))
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
            print("joint0:", theta0 * (180 / np.pi))
            print("joint1:", theta1 *  (180 / np.pi))
            print("joint2:", theta2 *  (180 / np.pi))
            print("joint3:", theta3 *  (180 / np.pi))
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

if __name__ == '__main__':
    invers_two()
#invers(10,7,9)
#invers_two(0,2.8,1)

# joint4 in range from -pi/2 <= theta4 <= pi/2
# joint3 (-2.4,2.5)
#joint2 (-2,2.25)