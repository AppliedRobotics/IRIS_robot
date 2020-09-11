from math import cos,sin,pi,sqrt
import numpy as np
import linecache as lc
import bisect as bis 

class LeftRoboticArm:
    def __init__(self):
        self.__l1 = 76
        self.__l2 = 205
        self.__l31 = 35
        self.__l3 = 90
        self.__l4 = 110
        self.motorId = [0, 1, 2, 3]
        self.motorOffset = [pi/2,pi/2,pi/2,2.62]
        self.motorSign = [-1,1,1,1]
        self.__q1_min = -1.46
        self.__q1_max = 0
        self.__q2_min = 0.15
        self.__q2_max = 1.534
        self.__q3_min = 0.27
        self.__q3_max = 1.57
        self.__stepAngleBD = 0.02
        self.__nameBDFile = 'ipbdlra.txt'
        self.__maxPosErr = 2
        self.__countSolve = 346764

    def DirectProblem (self,q1,q2,q3):

        l1 = self.__l1
        l2 = self.__l2
        l31 = self.__l31
        l3 = self.__l3
        l4 = self.__l4

        x = -cos(q1)*cos(q2)*l2 + cos(q1)*sin(q2)*l31 + (l3 + l4)*(-cos(q1)*cos(q2)*sin(q3) - cos(q3)*sin(q1))
        y = -cos(q2)*sin(q1)*l2 + sin(q1)*sin(q2)*l31 + (l3 + l4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))
        z = cos(q2)*l31 + l1 + l2*sin(q2) + sin(q2)*sin(q3)*(l3 + l4)
        
        return np.array([x,y,z])

    def DataIP(self):
        q1_min = self.__q1_min
        q1_max = self.__q1_max
        q2_min = self.__q2_min
        q2_max = self.__q2_max
        q3_min = self.__q3_min
        q3_max = self.__q3_max
        step = self.__stepAngleBD
        data = self.__FSData(q1_min,q1_max,q2_min,q2_max,q3_min,q3_max,step)

        f = open(self.__nameBDFile,'w')
        np.savetxt(f,data,fmt='% .2f')
        f.close()

        print(len(data[:,1]))

    def __FSData(self,q1_min, q1_max, q2_min, q2_max, q3_min, q3_max, step):
        data = []
        q1 = q1_min
        q2 = q2_min
        q3 = q3_min
        i = j = k = 0
        while q1 <= q1_max and i < 2:
            while q2 <= q2_max and j < 2:
                while q3 <=q3_max and k < 2:
                    data.append(np.concatenate([self.DirectProblem(q1,q2,q3),np.array([q1,q2,q3])]))
                    q3 = q3 + step
                    if(q3>=q3_max):
                        q3 = q3_max
                        k = k + 1
                k = 0
                q3 = q3_min
                q2 = q2+step
                if(q2>=q2_max):
                    q2 = q2_max
                    j = j + 1
            j = 0
            q2 = q2_min
            q1 = q1 + step
            if(q1>=q1_max):
                q1 = q1_max
                i = i + 1

        data = np.array(data, dtype = 'float16')
        data = data[data[:,0].argsort(kind='mergesort')]
        return data

    def SolveIP(self,x,y,z):
        err = self.__maxPosErr*4
        data = self.__ReadNearlyData(x,err)
        max_err = self.__maxPosErr
        nearlySolve = self.__FindSolve(x,y,z,err,data)
        lastNearlySolve = nearlySolve
        last_step = self.__stepAngleBD
        i = 1 
        if (type(nearlySolve) is int):
            return 0,0
        while (sqrt((x-nearlySolve[0])**2+(y-nearlySolve[1])**2+(z-nearlySolve[2])**2) >= max_err ):
            step = self.__stepAngleBD/i
            err = self.__maxPosErr + 0.1
            data = self.__FSData(nearlySolve[3]-last_step,nearlySolve[3]+last_step,
                        nearlySolve[4]-last_step,nearlySolve[4]+last_step,
                        nearlySolve[5]-last_step,nearlySolve[5]+last_step,step)
            nearlySolve = self.__FindSolve(x,y,z,err,data)
            if(type(nearlySolve) is int):
                nearlySolve = lastNearlySolve
                break
            i = i + 0.3
            last_step = step
        cur_err = sqrt((x-nearlySolve[0])**2+(y-nearlySolve[1])**2+(z-nearlySolve[2])**2)
        return nearlySolve,cur_err 

    def __ReadNearlyData(self,x,err):
        left_lim = 1
        right_lim = self.__countSolve
        ind = -1 
        while True:
            current_ind = int((right_lim+left_lim)/2)
            if (current_ind == 0):
                ind = 1
                break
            if (current_ind == self.__countSolve):
                ind = self.__countSolve
                break
            first_num = float(lc.getline(self.__nameBDFile,current_ind+1).split()[0])
            second_num = float(lc.getline(self.__nameBDFile,current_ind).split()[0])
            if(x-err<=first_num and x-err >=second_num):
                ind = current_ind + 1 
                break
            elif(x - err < second_num):
                right_lim = current_ind-1
            elif(x - err > first_num):
                left_lim = current_ind + 1

        data = []
        first_arr = [float(elem) for elem in lc.getline(self.__nameBDFile,ind).split()]
        data.append(first_arr)
        i = 1
        while True:
            if (ind == current_ind):
                break
            else:
                next_arr = [float(elem) for elem in lc.getline(self.__nameBDFile,ind+i).split()]
                if(next_arr[0] <= x + err):
                    data.append(next_arr)
                    i = i + 1
                else:
                    break

        return np.array(data, dtype = 'float16')

    def __FindSolve(self,x,y,z, err,data):
        nearly_data,flag = self.__SearchNearlySolve(data,1,y,err)
        if (flag == 0):
            return 0
        solve,flag = self.__SearchNearlySolve(nearly_data,2,z,err)
        if (flag ==0):
            return 0
        min = sqrt((solve[0,0]-x)**2+(solve[0,1]-y)**2+(solve[0,2]-z)**2)
        ind_min = 0
        for i in range(1,len(solve[:,0])):
            new_err = sqrt((solve[i,0]-x)**2+(solve[i,1]-y)**2+(solve[i,2]-z)**2)
            if(new_err < min):
                min = new_err
                ind_min = i
        return solve[ind_min,:]

    def __SearchNearlySolve(self,data, num_col, value, err):
        flag = 1
        data = data[data[:,num_col].argsort(kind='mergesort')]
        r_ind = bis.bisect_right(data[:,num_col],value+err)
        l_ind = bis.bisect_left(data[:,num_col],value-err)
        if (r_ind-l_ind == 0):
            flag = 0
        if(r_ind == len(data[:num_col])):
            r_ind = r_ind-1
        return data[l_ind:r_ind+1,:],flag