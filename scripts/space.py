import perlin
import numpy as np


sonar_range = 10

class Space:
    def __init__(self,x,y):
        self.x_diff = int(2*x)
        self.y_diff = int(2*y)
        self.x_0 = -x
        self.y_0 = -y
        self.z = -1
        self.plot_count = 1
        self.discritize_space()
    def discritize_space(self):
        x_blocks = self.x_diff + 1
        y_blocks = self.y_diff + 1

        
        p = perlin.PerlinNoiseFactory(2,unbias=True)
        perlin_delta = 4.0/x_blocks

        self.bel = np.ones((x_blocks,y_blocks))*0.5
        for i in range(x_blocks):
            for j in range(y_blocks):
                self.bel[i,j] += (p(i*perlin_delta,j*perlin_delta)+1)/2.0

        self.x_cords = [self.x_0 + i for i in range(x_blocks)]
        self.y_cords = [self.y_0 + i for i in range(y_blocks)]


        self.bel/=self.bel.sum()
        self.scan = self.sonarScans(20)
        self.p_obs, self.p_dynm = self.defineModels(self.bel,self.scan)
    def sonarScans(self,num_ang):
        #this determines which relative angles are within a scan range so I don't have to commute it evertime
        delta = 2*np.pi/ num_ang
        scans = [[]for i in range(num_ang)]
        for i in range(num_ang):
            for x in range(-10,11):
                for y in range(-10,11):
                    rng = np.linalg.norm([x,y])
                    if rng < 10:
                        ang = np.arctan2(y,x)
                        if ang%(2*np.pi) >= (i*delta) and ang%(2*np.pi) <= ((i+1)*delta):
                            scans[i].append([x,y,rng])
        return scans
    def defineModels(self,bel,scans):
        #p_obs(x_t,y_t|bx_t,by_t,sonar_ang_t)
        p_obs = np.ones(shape=(2*sonar_range+1,2*sonar_range+1,len(scans)))

        #this is my accuracy model, I assume accuracy linearly decreases with distance
        #first is intercept and 2nd is slope
        acc = [.1,.8/10]

        #Loop through all the possible scans the bluerov could be doing
        for k in range(len(scans)):
            #change the probability for all scans in the range that are within the grid
            for i in range(len(scans[k])):
                p_obs[scans[k][i][0]+10,scans[k][i][1]+10,k] = acc[0] + acc[1]*scans[k][i][2]
        


        #p_dynm(x_t+1,y_t+1|x_t,y_t)
        p_outside = 0.001     #probability it is out of space, deals with last row of p_dynm
        p_dynm = np.array([[0.01,0.01,0.01],
                        [0.01,0.92,0.01],
                        [0.01,0.01,0.01],
                        [p_outside/(2*bel.shape[0]+2*bel.shape[1]),0,0]])
        p_dynm[:3,:]/=p_dynm[:3,:].sum()

        
        return p_obs,p_dynm
    def bayes(self,bel,z,p_dyn,p_obs):
        belBar = np.zeros(shape=bel.shape) 
        newBel = np.zeros(shape=bel.shape)
        #loop through space
        for i in range(bel.shape[0]):
            for j in range(bel.shape[1]):
                #the way I have it set up the red asset is assumed to not be able to move
                #more than one space so we only have to check neighbors
                for x in range(-1,2):
                    for y in range(-1,2):
                        if i+x >= 0 and i+x < bel.shape[0] and j+y>=0 and j+y < bel.shape[1]:
                            belBar[i,j] += p_dyn[x+1,y+1]*bel[i+x,j+y]
                        else:
                            belBar[i,j] += p_dyn[3,0]
                if np.linalg.norm([i-z[0],j-z[1]]) <=10:
                    newBel[i,j] = belBar[i,j] * p_obs[i-z[0]+10,j-z[1]+10,z[2]]
                else:
                    newBel[i,j] = belBar[i,j]
        newBel /= newBel.sum()
        return newBel