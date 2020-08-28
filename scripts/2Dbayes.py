import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(precision=3)
sonar_range = 10
angle_diff = 2*np.pi/20


def sonarScans(num_ang):
    #this determines which relative angles are within a scan range so I don't have to commute it evertime
    delta = 2*np.pi/ num_ang
    scans = [[]for i in range(num_ang)]
    for i in range(num_ang):
        for x in range(-10,11):
            for y in range(-10,11):
                rng = np.linalg.norm([x,y])
                if rng < 10:
                    ang = np.arctan2(y,x)
                    if ang >= i*delta and ang <= (i+1)*delta:
                        scans[i].append([x,y,rng])
    return scans




                



def defineModels(bel,scans):
    #p_obs(x_t,y_t|bx_t,by_t,sonar_ang_t)
    p_obs = np.ones(shape=(bel.shape[0],bel.shape[1],bel.shape[0],bel.shape[1],20))

    #this is my accuracy model, I assume accuracy linearly decreases with distance
    #first is intercept and 2nd is slope
    acc = [.1,.8/10]

    #Loop through all the possible places the bluerov could be with and its scan angle
    for i in range(p_obs.shape[0]):
        for j in range(p_obs.shape[1]):
            for k in range(p_obs.shape[4]):
                #change the probability for all scans in the range that are within the grid
                for scan in scans[k]:
                    if i+scan[0] >=0 and i+scan[0] < bel.shape[0]:
                        if j+scan[1]>=0 and j+scan[1]< bel.shape[1]:
                            p_obs[i+scan[0],j+scan[1],i,j,k] = acc[0] + acc[1]*scan[2]
                #normalize
                p_obs[:,:,i,j,k] /= p_obs[:,:,i,j,k].sum()





    #p_dynm(x_t+1,y_t+1|x_t,y_t)
    p_dynm = np.zeros(shape=(bel.shape[0],bel.shape[1],bel.shape[0],bel.shape[1]))
    
    #loop through all the states at time t and and assign probabilites of where it could be at t+1
    for i in range(p_dynm.shape[0]):
        for j in range(p_dynm.shape[1]):
            p_dynm[max(0,i-1),j,i,j] = 0.05    #left
            p_dynm[max(0,i-1),max(0,j-1),i,j] = 0.05     #left and up
            p_dynm[max(0,i-1),min(p_dynm.shape[1]-1,j+1),i,j] = 0.05  #left and down
            p_dynm[i,max(0,j-1),i,j] = 0.05     #up
            p_dynm[i,min(p_dynm.shape[1]-1,j+1),i,j] = 0.05  #down
            p_dynm[min(p_dynm.shape[0]-1,i+1),j,i,j] = 0.05   #right
            p_dynm[min(p_dynm.shape[0]-1,i+1),max(0,j-1),i,j] = 0.05     #right and up
            p_dynm[min(p_dynm.shape[0]-1,i+1),min(p_dynm.shape[1]-1,j+1),i,j] = 0.05  # right and down

            p_dynm[i,j,i,j] = 0.6   #doesn't move
            p_dynm[:,:,i,j] /= p_dynm[:,:,i,j].sum()    #renormalize
    return p_obs,p_dynm



def bayes(bel,z,p_dyn,p_obs):
    belBar = np.zeros(shape=bel.shape) 
    newBel = np.zeros(shape=bel.shape)
    #loop through space
    for i in range(bel.shape[0]):
        for j in range(bel.shape[1]):
            #the way I have it set up the red asset is assumed to not be able to move
            #more than one space so we only have to check neighbors
            for x in range(-1,2):
                for y in range(-1,2):
                    if i+x >= 0 and i+x < bel.shape[0]:
                        if j+y>=0 and j+y < bel.shape[1]:
                            belBar[i,j] += p_dyn[i,j,i+x,j+y]*bel[i+x,j+y]
            newBel[i,j] = belBar[i,j] * p_obs[i,j,z[0],z[1],z[2]]
    newBel /= newBel.sum()
    return newBel



def simulate(bel,state,plotting=False):
    scans = sonarScans(20)
    p_obs,p_dyn = defineModels(bel,scans)
    
    if(plotting):
        fig,ax = plt.subplots()
    
    z = [5,5,0]
    
    for i in range(200):
        print(i)

        bel = bayes(bel,z,p_dyn,p_obs)
        z[2] = (z[2]+1)%20
        if(plotting):
            x = np.array(range(40))
            X,Y = np.meshgrid(x,x)
            cp = plt.contourf(X, Y, bel.transpose())

            ax.set_title('Contour Plot')
            plt.pause(0.05)




if __name__=="__main__":
    bel = np.random.rand(40,40)
    bel/=bel.sum()
    state = np.array([np.random.randint(0,40),np.random.randint(0,40)])
    simulate(bel,state,True)
    # print(state.shape)