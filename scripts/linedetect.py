# Python program to illustrate HoughLine 
# method for line detection 
import cv2 
import numpy as np 
import sys
import matplotlib.pyplot as plt
from PIL import Image

# Reading the required image in  
# which operations are to be done.  
# Make sure that the image is in the same  
# directory in which this python program is 
def gen_noisy_line(n, eps):
    line = []

    noise = np.random.rand(n)
    noise = noise/2

    def f(x):
        return 0.5*x
    for i in range(n):

        line.append(f(i) + noise[i])
    return line
np.set_printoptions(threshold=sys.maxsize)

x = np.array(list(range(60)))/10


y = gen_noisy_line(60, 0)

plt.scatter(x,y, marker='.')
plt.axis('off')

# edges = y
plt.savefig('asd.png')

img = cv2.imread('/Users/rednecked_crake/Desktop/asd.png')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)

print(edges)
img = Image.fromarray(edges, 'L')
img.save('my.png')
img.show()


# This returns an array of r and theta values 
lines = cv2.HoughLines(edges,1,np.pi/180, 200) 
  
# The below for loop runs till r and theta values  
# are in the range of the 2d array 
for r,theta in lines[0]: 
      
    # Stores the value of cos(theta) in a 
    a = np.cos(theta) 
  
    # Stores the value of sin(theta) in b 
    b = np.sin(theta) 
      
    # x0 stores the value rcos(theta) 
    x0 = a*r 
      
    # y0 stores the value rsin(theta) 
    y0 = b*r 
      
    # x1 stores the rounded off value of (rcos(theta)-1000sin(theta)) 
    x1 = int(x0 + 1000*(-b)) 
      
    # y1 stores the rounded off value of (rsin(theta)+1000cos(theta)) 
    y1 = int(y0 + 1000*(a)) 
  
    # x2 stores the rounded off value of (rcos(theta)+1000sin(theta)) 
    x2 = int(x0 - 1000*(-b)) 
      
    # y2 stores the rounded off value of (rsin(theta)-1000cos(theta)) 
    y2 = int(y0 - 1000*(a)) 
      
    # cv2.line draws a line in img from the point(x1,y1) to (x2,y2). 
    # (0,0,255) denotes the colour of the line to be  
    #drawn. In this case, it is red.  
    cv2.line(img,(x1,y1), (x2,y2), (0,0,255),2) 
      
# All the changes made in the input image are finally 
# written on a new image houghlines.jpg 
cv2.imwrite('linesDetected.jpg', img) 
