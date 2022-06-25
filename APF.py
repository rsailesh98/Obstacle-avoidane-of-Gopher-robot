import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

dt=1

#This class defines the point we need it to plot the start and the goal
class Point():

    def __init__ (self,x,y):
        self.x=x
        self.y=y

    def draw_point(self,ax,color,marker,size):
        ax.scatter(self.x,self.y,c=color, marker=marker,s=size)

#This class defines the robot and various points
class Gopher():
    def __init__(self, x,y,r,vel):
        self.x=x
        self.y=y
        self.radius=r
        self.maximum_vel=vel
        self.robot_path=[]
        self.robot_path.append([self.x,self.y])
        self.path_length=0

    def robot_move(self, next_pt):
        self.x=next_pt.x 
        self.y=next_pt.y 

        self.robot_path.append([self.x,self.y])

    def path_length_calculation(self, next_pt):
        self.path_length+=((self.x-next_pt.x)**2+(self.y-next_pt.y)**2)**0.5

    def draw_robot(self,ax,color):
        c=plt.Circle((self.x,self.y),self.radius, color=color, fill=True)
        ax.add_patch(c)
        



#This class defines the obstacles and various parameters
class Obstacle():

    def __init__(self, x,y,r,vel,theta):

        self.x=x
        self.y=y
        self.radius=r
        self.vel=vel
        self.x_vel=vel*cos(theta)
        self.y_vel=vel*sin(theta)


    def Obs_move(self):
        self.x+= self.x_vel*dt
        self.y+=self.y_vel*dt


    def draw_obstacle(self,ax,color):
        c=plt.Circle((self.x,self.y),self.radius, color=color, fill=True)
        ax.add_patch(c)


class Environment():

    def __init__(self,minX, minY, maxX, maxY, begin, end):
        self.mX=minX
        self.mY=minY
        self.MX=maxX 
        self.MY=maxY

        self.start=begin
        self.end=end

        self.fig, self.ax = plt.subplots(1, 1, figsize=(15,15))


    def draw_Environment(self, Gopher, obs,step):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(15,15))
        self.ax.set(xlim=(self.mX, self.MX), ylim=(self.mY, self.MY))


        #Defines the titles and start and end point
        self.ax.set_title("MPC",loc='center')
        self.start.draw_point(self.ax,'g','*',100)
        self.end.draw_point(self.ax,'r','*',100)

        for o in obs:
            o.draw_obstacle(env.ax,'r')

        #Used to draw the robot
        Gopher.draw_robot(env.ax,'b')

        
        #Enter the path of the images in this location
        env.fig.savefig('./{}.png'.format(str(step)), dpi=100)

        


    


def distance(x1,y1,x2,y2):
    return ((x1-x2)**2+(y1-y2)**2)**0.5




#This class implements the Artificial Potential Field path planning
class APF():


    def __init__(self,env, gopher,obstacles, begin, end):
        self.start=begin
        self.env=env
        self.goal=end
        self.obstacles=obstacles
        self.gopher=gopher
        self.k_att=0.03
        self.k_rep=1000
        self.max_dist=150
        self.max_iteration=1000
        self.i=0
        self.thresh=5
        self.path=[]
        self.path_found=False

    def attraction(self):
        attractive_force=np.array([(self.goal.x-self.gopher.x)*self.k_att, (self.goal.y-self.gopher.y)*self.k_att])
        return attractive_force

    def repulsive(self):
        repulsive_force=np.array([0,0])
        for o in self.obstacles:
            d=np.array([self.gopher.x,self.gopher.y])-np.array([o.x,o.y])
            u_v=d/np.linalg.norm(d)
            if np.linalg.norm(d)>self.max_dist or o.vel==0:
                continue
            else:
                repulsive_force=repulsive_force+ np.array([u_v[0],u_v[1]])*self.k_rep*((1/np.linalg.norm(d))-(1/self.max_dist))/(np.linalg.norm(d)**2)
        return repulsive_force


    #This function calculates the net force that should be exerted on the robot.
    def force_calculation(self):

        step=0
        total_time=0
        min_clear=np.inf
        while (self.i < self.max_iteration) and (distance(self.gopher.x,self.gopher.y,self.goal.x,self.goal.y)> self.thresh):
            f=self.attraction()+self.repulsive()
            f_vector=f/np.linalg.norm(f)
            next_x=self.gopher.x+(f_vector[0]*dt)
            next_y=self.gopher.y+(f_vector[1]*dt)
            next_step=Point(next_x,next_y)  
            self.path.append([next_x,next_y])
            step+=1
            for o in self.obstacles:
                dist=distance(self.gopher.x,self.gopher.y,o.x,o.y)
                min_clear=min(dist,min_clear)
                o.Obs_move()
            self.gopher.path_length_calculation(next_step)
            self.gopher.robot_move(next_step)
            total_time+=dt
            if distance(self.gopher.x,self.gopher.y,self.goal.x,self.goal.y)<self.thresh:
                self.path_found=True
                
            self.env.draw_Environment(self.gopher,self.obstacles,step)

        #Display the various evaluation metrics
        print(total_time)
        print(self.gopher.path_length)
        print(min_clear)


minX=0
miny=0
maxX=100
maxY=100
vel_max=5
radius=2


start=Point(0,50)
goal=Point(70,60)

# start=Point(0,0)
# goal=Point(100,100)


robot=Gopher(start.x,start.y, radius, vel_max)


#Scenario 1
#obstacles=[Obstacle(35, 75, 2, 0, np.pi/6), Obstacle(30, 40, 2, 0, np.pi/3), Obstacle(10, 30, 2 , 0, 0), Obstacle(50, 50, 2 , 0, np.pi/2), Obstacle(80, 50, 2, 0, np.pi/6), Obstacle(60, 80, 2, 0, np.pi/3), Obstacle(10, 20, 2 , 0, 0), Obstacle(10,90, 2 , 0, np.pi/2)]


#Scenario 2
#obstacles=[Obstacle(35, 75, 2, 2, 0), Obstacle(30, 40, 2, 2, 0), Obstacle(10, 30, 2 , 2, 0), Obstacle(50, 50, 2 , 2, 0), Obstacle(80, 50, 2, 2, 0), Obstacle(60, 80, 2, 1, 0), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 1, 0)]


#Scenario 3
#obstacles=[Obstacle(35, 75, 5, 0.03, 0), Obstacle(30, 40, 5, 0.5, 0), Obstacle(10, 30, 5 , 0.3, 0), Obstacle(50, 50, 6 , 0.7, 0), Obstacle(80, 80, 6 , 0.0, 0), Obstacle(40, 60, 6 , 0.2, 0) ]

#Scenario 4
#obstacles=[Obstacle(50,50,3,2,1.25*np.pi)]


#Scenario 5
#Hospital Environment
obstacles=[Obstacle(50,60,1,1,-np.pi), Obstacle(50,80,2,0.5,1.5*np.pi)]


for i in range(0,30):
    obstacles.append(Obstacle(i,30,1,0,1.25*np.pi))
    obstacles.append(Obstacle(i,70,1,0,1.25*np.pi))
    obstacles.append(Obstacle(100-i,30,1,0,1.25*np.pi))
    obstacles.append(Obstacle(100-i,70,1,0,1.25*np.pi))


for j in range(70,100):
    obstacles.append(Obstacle(70,100-j,1,0,1.25*np.pi))
    obstacles.append(Obstacle(30,100-j,1,0,1.25*np.pi))
    obstacles.append(Obstacle(70,j,1,0,1.25*np.pi))
    obstacles.append(Obstacle(30,j,1,0,1.25*np.pi))




env=Environment(minX,miny,maxX,maxY,start,goal)

Artificial=APF(env, robot ,obstacles,start,goal)
Artificial.force_calculation()
