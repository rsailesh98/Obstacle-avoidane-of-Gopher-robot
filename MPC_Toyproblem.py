import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
import cvxpy as cp
import time
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




#This class implements the Model Predictive Controller

class MPC:
    def __init__(self,env, gopher,obstacles, start,goal):
        self.env=env
        self.gopher=gopher
        self.obstacles=obstacles
        self.start=start
        self.goal=goal
        self.threshold=5
        self.horizon=1
        self.max=self.gopher.maximum_vel
        self.solthresh=5
        self.min_clear=np.inf
        
        #Define the constraints
    def constraints(self, X, Y, obs):
        fx=((self.gopher.x-obs.x+X.sum()*dt)**2+(self.gopher.y-obs.y+Y.sum()*dt))**2-(self.gopher.radius+obs.radius)**2
        delf= (np.array([2*dt*(self.gopher.x + X.sum() * dt - obs.x)]*len(X)), np.array([2*dt*(self.gopher.y + Y.sum() * dt - obs.y)]*len(Y)))
        return fx,delf

    #The Path is determined in this function w.r.t constraints and objective function
    def MPC_path_planner(self):
        step=0
        #Finding the path for MPC path planner
        total_time=0
        while distance(self.gopher.x,self.gopher.y,self.goal.x,self.goal.y)>self.threshold:
            xd,yd=np.zeros(shape=self.horizon), np.zeros(shape=self.horizon)
            Xd,Yd=cp.Variable(shape=(self.horizon,1)),cp.Variable(shape=(self.horizon,1))
            
            prev_sol=np.inf
            while True:
                cons=[]

                for s in range(self.horizon):
                    cons.extend([-self.max<=Xd[s],Xd[s]<=self.max, -self.max<=Yd[s], Yd[s]<=self.max])


                for o in self.obstacles:
                    dist=distance(self.gopher.x,self.gopher.y,o.x,o.y)
                    self.min_clear=min(dist,self.min_clear)
                    fx,delf=self.constraints(xd[:s+1],yd[:s+1],o)
                    cons.append(fx+delf[0].reshape(1,-1)@(Xd[:s+1]-xd[:s+1].reshape(-1,1))+delf[1].reshape(1,-1)@(Yd[:s+1]-yd[:s+1].reshape(-1,1))>=0)

                optimize=cp.Minimize((self.gopher.x-self.goal.x+cp.sum(Xd))**2+(self.gopher.y-self.goal.y+cp.sum(Yd))**2)

                Definition=cp.Problem(optimize,cons)

                solution=Definition.solve(solver='ECOS_BB')
               
                if abs(solution-prev_sol)<=self.solthresh:
                    break
                prev_sol=solution

                xd=Xd.value.flatten()
                yd=Yd.value.flatten()
            

            X_next=self.gopher.x+Xd.value[0]
            Y_next=self.gopher.y+Yd.value[0]

            next=Point(X_next,Y_next)
            
            self.gopher.path_length_calculation(next)
            self.gopher.robot_move(next)
            print(self.gopher.path_length)
            
            step+=1

            for o in self.obstacles:
                o.Obs_move()
            self.env.draw_Environment(self.gopher,self.obstacles,step)
            total_time+=dt
            
        print(self.min_clear)

        print(total_time)

            








minX=0
miny=0
maxX=100
maxY=100
vel_max=5
radius=2




#Defining the goal and scenarios
start=Point(0,50)
goal=Point(70,60)


# start=Point(0,0)
# goal=Point(100,100)


robot=Gopher(start.x,start.y, radius, vel_max)

#Scenario 1
# obstacles=[Obstacle(35, 75, 2, 0, np.pi/6), Obstacle(30, 40, 2, 0, np.pi/3), Obstacle(10, 30, 2 , 0, 0), Obstacle(50, 50, 2 , 0, np.pi/2), Obstacle(80, 50, 2, 0, np.pi/6), Obstacle(60, 80, 2, 0, np.pi/3), Obstacle(10, 20, 2 , 0, 0), Obstacle(10,90, 2 , 0, np.pi/2)]




#Scenario 2
#obstacles=[Obstacle(35, 75, 2, 2, 0), Obstacle(30, 40, 2, 2, 0), Obstacle(10, 30, 2 , 2, 0), Obstacle(50, 50, 2 , 2, 0), Obstacle(80, 50, 2, 2, 0), Obstacle(60, 80, 2, 1, 0), Obstacle(10, 20, 2 , 0.001, 0), Obstacle(10,90, 2 , 1, 0)]

#Scenario 3
# obstacles=[Obstacle(35, 75, 5, 0.03, 0), Obstacle(30, 40, 5, 0.5, 0), Obstacle(10, 30, 5 , 0.3, 0), Obstacle(50, 50, 6 , 0.7, 0), Obstacle(80, 80, 6 , 0.0, 0), Obstacle(40, 60, 6 , 0.2, 0) ]



#Scenario 4

#obstacles=[Obstacle(50,50,3,2,1.25*np.pi)]




#Scenario 5
#Hospital scenario
obstacles=[Obstacle(60,50,1,1.5,-np.pi), Obstacle(50,80,2,1,1.5*np.pi), Obstacle(80,50,1,1.5,-np.pi)]


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

mpc=MPC(env, robot ,obstacles,start,goal)
mpc.MPC_path_planner()
