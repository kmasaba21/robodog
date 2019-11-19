import numpy as np
import math
import matplotlib.pyplot as plt

pi = 3.1415926535898


def t(a):
    return a*pi/180 

threshold_a = 6



# typedef struct _CSData
# {
#     std::vector<unsigned int> index;//索引值
#     std::vector<double> bearings;//角度
#     std::vector<double> cos_value;//余弦
#     std::vector<double> sin_value;//正弦
# }CSdata;

# typedef struct _RangeData
# {
#     std::vector<double> ranges;//role数值
#     std::vector<double> xs;//x坐标
#     std::vector<double> ys;//y坐标
# }Rangedata;

class AttrDict(dict):
  __getattr__ = dict.__getitem__
  __setattr__ = dict.__setitem__



def gen_noisy_line(n, eps):
    line = []

    noise = np.random.rand(n)
    noise = noise/2

    def f(x):
        return 0.5*x
    for i in range(n):

        line.append(f(i) + noise[i])


    return line





def initLineFeature(scan_msg, line_feature,scan_ranges_doubles):
    angle_increment = scan_msg.angle_increment
    angle_start = scan_msg.angle_start
    angle_max = scan_msg.angle_max

    angle_min = scan_msg.angle_min

    line_feature.angle_increment = angle_increment
    line_feature.angle_start = angle_start
    bearings = []
    cos_bearings = [] 
    sin_bearings = []
    index = []
    i = 0
    b = 0

    while b <= angle_max + angle_increment:

    # for b in range(angle_min, angle_max + angle_increment,angle_increment):
        bearings.append(b)
        cos_bearings.append(math.cos(b))
        sin_bearings.append(math.sin(b))
        index.append(i)
        i += 1

        b += angle_increment


    line_feature.setCosSinData(index, cos_bearings, sin_bearings, bearings)


    line_feature.setRangeData(scan_ranges_doubles)


    #     def __init__(self, angle_increment, angle_start, 
    #                   least_thresh, min_line_length,
    #                    predict_distance, min_line_points,
    #                    seed_line_points):


    line_feature.params = Params(pi/180, 0, 2, 0.5, 1, 12, 6)
    # line_feature.params.least_threshold = 0.04;
    # line_feature.params.min_line_length = 0.5;
    # line_feature.params.predict_distance = 0.1;
    # line_feature.params.seed_line_points = 6;
    # line_feature.params.min_line_points = 12;


class ScanRange:
    def __init__(self, x, y, z, zz):
        self.angle_increment = x
        self.angle_start = y
        self.angle_min = z
        self.angle_max = zz


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y


class CSData:
    def __init__(self, index, bearings, cos_value, sin_value):
        self.index = index
        self.bearings = bearings
        self.cos_value = cos_value
        self.sin_value = sin_value


class RangeData:
    def __init__(self, ranges, xs, ys):
        self.ranges = ranges
        self.xs = xs
        self.ys = ys

class Params:
    def __init__(self, angle_increment, angle_start, 
                    least_thresh, min_line_length,
                       predict_distance, min_line_points,
                       seed_line_points):


        self.angle_increment = angle_increment
        self.angle_start = angle_start
        self.least_thresh = least_thresh
        self.min_line_length = min_line_length
        self.predict_distance = predict_distance
        self.min_line_points = min_line_points
        self.seed_line_points = seed_line_points



class word_params:
    def __init__(self, _role, _theta_one, _theta_two):
        self._role = _role
        self._theta_one = _theta_one
        self._index_theta_two = _theta_two





class signal_params:
    def __init__(self, distance_signal):
        self.distance_signal = distance_signal




# typedef struct _line
# {
#     double a;//直线参数
#     double b;
#     double c;
#     int left;//直线范围
#     int right;
#     POINT p1;
#     POINT p2;
#     bool inte[2];
# }line;



class line:
    def __init__(self, a,b,c, left, right, p1, p2, inte):
        self.a = a
        self.b = b
        self.c = c
        self.left = left
        self.right = right
        self.p1 = p1
        self.p2 = p2
        self.inte = inte




class leastt:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c




# typedef struct _point
# {
#     double role;
#     double theta;
#     double m_x;
#     double m_y;
#     double distance;
#     double m_gradient;
#     bool flag;
# }PoinT;

class PointT:
    def __init__(self, role, theta, m_x, m_y, distance, m_gradient, flag):
        self.role = role
        self.theta = theta
        self.m_x = m_x
        self.m_y  = m_y
        self.distance  = distance
        self.m_gradient = m_gradient
        self.flag = flag




class gline:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2




# typedef struct _signal
# {
#     double _angle1_radian;
#     double _angle2_radian;
#     double _angle1_degree;
#     double _angle2_degree;
#     double _role;
# }Signal;


class Signal:
    def __init__(self, _angle1_radian, _angle2_radian, _angle1_degree, _angle2_degree):
        self._angle1_radian = _angle1_radian
        self._angle2_radian = _angle2_radian
        self._angle1_degree = _angle1_degree
        self._angle2_degree = _angle2_degree
        self.role = role



# typedef struct _feature_point
# {
#     POINT _point;
#     double _angle;
# }featurepoint;


class featurepoint:
    def __init__(self, _point, _angle):
        self._point = _point
        self._angle = _angle





# typedef struct _keyword
# {
#     int _index_role;
#     int _index_theta_one;
#     int _index_theta_two;
#     std::vector<int> _frame_index;
# }keyword;

class _keyword:
    def __init__(self, _index_role, _index_theta_one,
                       _index_theta_two, _frame_index):
        self._index_role = _index_role
        self._index_theta_one = _index_theta_one
        self._index_theta_two
        self._frame_index = _frame_index


class LineFeature:
    def __init__(self):
        self.cs_data = None
        self.range_data = AttrDict()
        self.params = None

    def setCosSinData(self, index, cos_value, sin_value, bearings):
        self.cs_data  = CSData(index, bearings, cos_value, sin_value)



    def setRangeData(self, ranges):
        self.range_data.xs = []
        self.range_data.ys = []
        self.range_data.ranges = ranges


        for i in range(len(ranges)):
            self.range_data.xs.append(self.cs_data.cos_value[i]*ranges[i])
            self.range_data.ys.append(self.cs_data.sin_value[i]*ranges[i])


    def leastsquares(self, start, end, firstfit):
        w = np.zeros(3)

        n = end - start  + 1


        rangedata = self.range_data
        xs = rangedata.xs
        ys = rangedata.ys
        mid = np.zeros(5)

        if firstfit == 1:
            loop_start = start
            loop_end = end

        else:
            if firstfit == 2:
                loop_start = end
                loop_end = end
            else:
                loop_start = start
                loop_end = start
     
        k = 0

        while k < loop_end:
            mid[0] += xs[k]
            mid[1] += ys[k]
            mid[2] += xs[k]**2
            mid[3] += ys[k]**2
            mid[4] += xs[k]*ys[k]
            k += 1


        w[0] = n*mid[4] - mid[0]*mid[1]
        w[1] = mid[1]**2  - n*mid[3] - mid[0]**2 + n*mid[2]
        w[2] = mid[0]*mid[1] - n*mid[4]



        if w[0] == 0:
            return leastt(-1, 0, mid[0]/n)
        else:

            a = (-w[1] + math.sqrt(w[1]**2 - 4*w[0]*w[2]))/2.0/w[0]
            return leastt(a,
                          -1,
                          (mid[1]-a*mid[0])/n 
                          )


    
    def detectline(self, start, num, m_least):
        flag = False

        #Define the vertical error from point to line
        error1 = 0
        #Define the error from the next point to the predicted position
        error2 = 0
        k = 0
        #Predict the next position
        m_pn = Point(0, 0)
        #The next point, y = kp*x;
        kp = 0
        theta = 0
        
        rangedata = self.range_data
        xs = rangedata.xs
        ys = rangedata.ys


        a = m_least.a
        b = m_least.b
        c = m_least.c

        params = self.params


        for k in range(start, start + num):
            error1 = abs(a*xs[k] + b*ys[k] + c)/math.sqrt(1 + a**2)

            print(error1)
            if error1 > params.least_thresh:
                print('least here')
                flag = True
                break

            theta = params.angle_increment*k + params.angle_start


            if abs(abs(theta) - pi/2) < 1e-05:
                m_pn.x = 0
                m_pn.y = c

            else:
                kp = math.tan(theta)
                m_pn.x = c/(kp - a)
                m_pn.y = kp*m_pn.x



            error2 = np.linalg.norm(np.array([xs[k], ys[k]]) - np.array([m_pn.x, m_pn.y]))

            if error2 > params.predict_distance:
                flag = True
                break
        

        if flag:

            print('broke')

            return False
        else:
            return True


    def delete_short_line(self, n1, n2):
        v1 = np.array([self.range_data.xs[n1], self.range_data.ys[n1]])
        v2 = np.array([self.range_data.xs[n2], self.range_data.ys[n2]])
        
        if np.linalg.norm(v1, v2) < self.params.min_line_length:
            return False
        else:
            return True


    def detectfullline(self, start, point_num, m_line, m_least):

        flag1 = True
        flag2 = True

        n1 = 0
        n2 = 0
        a = 0
        b = 0
        c = 0

        params = self.params
        a = m_least.a

        b = m_least.b
        c = m_least.c

        n2 = start + params.seed_line_points
        m_result = leastt(0,0,0)

        rangedata = self.range_data
        xs = rangedata.xs
        ys = rangedata.ys


        while flag2:
            if (abs(a*xs[n2] + b*ys[n2] + c)/(math.sqrt(1 + a**2))) < params.least_thresh:
                self.m_least = self.leastsquares(start, n2, 2)

                if (n2 < len(point_num) - 1):
                    print('kickback ', n2, len(point_num))
                    n2 = n2 + 1
                    a = m_least.a
                    b = m_least.b
                    c = m_least.c
                else:
                    flag2 = False
            else:
                flag2 = False


        n2 = n2 - 1

        n1 = start - 1

        if (n1 < 0):
            flag1 =  False


        while flag1:
            if (abs(a*xs[n1] + b*ys[n1] + c)/(math.sqrt(1 + a**2)))  < params.least_thresh:
                self.m_least = self.leastsquares(n1, n2, 3)

                if (n1 > 0):

                    print('kickback 2', n1)
                    n2 = n2 + 1
                    a = m_least.a
                    b = m_least.b
                    c = m_least.c
                else:
                    flag1 = False
            else:
                flag1 = False




        n1 += 1

        m_result = leastsquares(n1, n2, 1)

        m_temp = line(m_result.a, m_result.b, m_result.c, n1, n2, None, None, None)

        if (n2 - n1) > params.min_line_points:
            if delete_short_line(n1, n2):
                m_line.append(m_temp)

            return n2
        else:
            return start




    def cleanline(self, m_line):
        if len(m_line) < 2:
            return


        m = 0
        n = 0
        m_iter = 0
        error1 = 0
        error2 = 0
        line_temp = 0

        temp_least = leastt(0,0,0)


        theta_one = 0
        theta_two = 0 
        theta_d_ = 0

        i = 0 
        j = 0

        xs = self.range_data.xs
        ys = self.range_data.ys


        for i in range(0, len(m_line) - 1):
            m = m_line[j].right

            for j in range(i+1, len(m_line) - 1):
                n = m_line[j].left

                if m >= n:
                    theta_one = math.atan(m_line[p].a)
                    theta_two = math.atan(m_line[q].a)

                    theta_d_ = math.abs(theta_one - theta_two)


                    if theta_d_ < 0.1 or theta_d_ < pi - 0.1:
                        left = min(m_line[i].left, m_line[j].left)
                        m_temp = leastsquares(left, m_line[j].right, 1)

                        m_line[i].a = m_temp.a
                        m_line[i].b = m_temp.b
                        m_line[i].c = m_temp.c

                        m_line[i].left = left

                        m_line[i].right = m_line[j].right

                        del m_line[j]

                        m = m_line[i].right
                        q -= 1


        for i in range(0, len(m_line) - 1):
            j = i+1

            m = m_line[i].right
            n = m_line[j].left

            if m >= n:
                for k in range(n, m):
                    line_temp = k
                    a = m_line[i].a
                    b = m_line[i].b
                    c = m_line[i].c
                    error1 = abs(a*xs[k] + b*ys[k] + c)/math.sqrt(1 + a**2)
                    a = m_line[j].a
                    b = m_line[j].b
                    c = m_line[j].c
                    error2 = abs(a*xs[k] + b*ys[k] + c)/math.sqrt(1 + a**2)


                    if error1 > error2:
                        break

                m_line[i].right = k-1
                temp_least = leastsquares(m_line[i].left, m_line[i].right, 1)
                m_line[i].a = temp_least.a 
                m_line[i].b = temp_least.b
                m_line[i].c = temp_least.c


                m_line[j].left = k
                temp_least = leastsquares(m_line[j].left, m_line[j].right, 1)
                m_line[j].a = temp_least.a 
                m_line[j].b = temp_least.b
                m_line[j].c = temp_least.c



    def generate(self, m_line):
        m = 0
        n = 0
        k1 = 0
        k2 = 0

        rangedata = self.range_data
        endpoint1 = Point(None,None)
        endpoint1 = Point(None,None)

        line_temp = gline(None, None, None, None)

        xs = rangedata.xs
        ys = rangedata.ys

        output = []

        for i in range(0, len(m_line) - 1):
            m = m_line[i].left
            n = m_line[i].right
            a = m_line[i].a
            b = m_line[i].b
            c = m_line[i].c

            if m_line[i].b != 0:
                endpoint1.x = (xs[m]/a + ys[m] - c)/(a + 1.0/a)
                endpoint1.y = (a*endpoint1.x + c)
            else:
                endpoint1.x = ys[m]
                endpoint1.y = c/a

            line_temp.x = endpoint1.x
            line_temp.y = endpoint1.y

            m_line[i].p1 = endpoint1

            if b != 0:
                endpoint2.x = (xs[n]/a + ys[n] - c)/(a + 1.0/a)
                endpoint2.y = a*x + c
            else:
                #### SUS
                endpoint2.x = ys[n]
                endpoint2.y = c/a

            line_temp.x2 = endpoint2.x
            line_temp.y2 = endpoint2.y


            m_line[i].p2 = endpoint2
            output.append(line_temp)


        return output


    def extractlines(self, temp_line1, temp_line2):
        line_include = 0
        m_line = []
        point_num = self.cs_data.index
        params = self.params

        if len(point_num) < params.min_line_points:
            return [], []

        for i in range(0, len(point_num) - params.min_line_points):

            m_least = self.leastsquares(i, i + params.seed_line_points - 1,  1)



            if self.detectline(i, params.seed_line_points, m_least):
                print('WE ARE HEREEE')
                i = self.detectfullline(i, point_num, m_line, m_least)


        print('m_line',m_line)

        self.cleanline(m_line)

        for p in range(0, len(m_line)):
            if not delete_short_line(m_line[p].left, m_line[p].right):
                del m_line[p]


        temp_line2 = self.generate(m_line)

        temp_line1 = m_line


if __name__ == '__main__':
    x = np.array(list(range(10)))/10
    y = gen_noisy_line(60, 0)

    # plt.scatter(x,y
    #     )

    # plt.show()





    lf = LineFeature()
    data = []
    scan_msg = ScanRange(0.0174533, 0, 0, 1.0471975511965976)

    initLineFeature(scan_msg,lf, y)



    tl1 = []
    tl2 = []

    lf.extractlines(tl1, tl2)


    print(tl1, tl2)





