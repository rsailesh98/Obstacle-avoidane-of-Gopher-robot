import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

dt=1

#Defining a class point which we will need to plot the start and goal points 
class Point():

    def __init__ (self,x,y):
        self.x=x
        self.y=y

    def draw_point(self,ax,color,marker,size):
        ax.scatter(self.x,self.y,c=color, marker=marker,s=size)

#Defining the Robot class

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
        

#Here we shall define a class for obstacle with its various parameters    
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



#This is the class in which the robot and obstacles interact with one another
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

        self.ax.set_title("VO",loc='center')
        self.start.draw_point(self.ax,'g','*',100)
        self.end.draw_point(self.ax,'r','*',100)

        for o in obs:
            o.draw_obstacle(env.ax,'r')

        
        Gopher.draw_robot(env.ax,'b')

        env.fig.savefig('./{}.png'.format(str(step)), dpi=100)

  
def distance(x1,y1,x2,y2):
    return ((x1-x2)**2+(y1-y2)**2)**0.5



def velocity_cone(o,gopher, gvel, theta):

    #Calculate the relative velocity and determine if collision will take place
    vrelx=gvel*cos(theta)-o.x_vel
    vrely=gvel*sin(theta)-o.y_vel

    relx=o.x-gopher.x
    rely=o.y-gopher.y

    dotproduct= (relx*vrelx+rely*vrely)**2
    r=relx**2+rely**2

    vr=vrelx**2+vrely**2

    rad=(gopher.radius+o.radius)**2


    if(r-(dotproduct/vr)>=rad):
        return True
    else:
        return False



#This class gives the the path that the robot needs to follow in order to successfully implement path planning
class Velocity_Obstacles():

    def __init__(self, env, begin, end, gopher, obstacles):
        self.env=env
        self.begin=begin
        self.end=end
        self.gopher=gopher
        self.obstacles=obstacles
        self.threshold=2
        self.found_path=False
        self.gen_rand=200
        self.traj= [start]

    def gen_random_vel(self, current):
        theta=np.random.uniform(0,2*np.pi)
        vel=np.random.uniform(0,self.gopher.maximum_vel)

        xnew=current.x+(vel*cos(theta)*dt)
        ynew=current.y+(vel*sin(theta)*dt)

        return vel,theta,Point(xnew,ynew)
    
    def cost_function(self, next):

        cost=np.sqrt((next.x-self.end.x)**2+(next.y-self.end.y)**2)
        
        return cost

    def velocity_optimizer(self):
        best=self.cost_function(self.traj[-1])
        step=0
        total_time=0
        min_clear=np.inf
        while not self.found_path:
            best_step=None

            for _ in range(self.gen_rand):
                collision_free_step=True

                next_vel,next_theta,next_step=self.gen_random_vel(self.traj[-1])
                for o in self.obstacles:
                    dist=distance(self.gopher.x,self.gopher.y,o.x,o.y)
                    min_clear=min(dist,min_clear)
                    if not velocity_cone(o,self.gopher,next_vel, next_theta):
                        collision_free_step=False
                        break

                if collision_free_step:
                    cost=self.cost_function(next_step)
                    if cost<best:
                        best=cost
                        best_step=next_step

            if best_step is not None:
                step+=1
                self.traj.append(best_step)      
                for o in self.obstacles:
                    o.Obs_move()

                self.gopher.path_length_calculation(best_step)
                self.gopher.robot_move(best_step)

            
            if best< self.threshold:
                self.found_path=True

            self.env.draw_Environment(self.gopher,self.obstacles,step)
            total_time+=dt

        print(self.gopher.path_length)
        print(total_time)
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

VO_traj=Velocity_Obstacles(env, start,goal,robot,obstacles)
VO_traj.velocity_optimizer()