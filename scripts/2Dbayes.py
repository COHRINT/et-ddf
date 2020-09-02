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
                    if ang%(2*np.pi) >= (i*delta) and ang%(2*np.pi) <= ((i+1)*delta):
                        scans[i].append([x,y,rng])
    return scans




                



def defineModels(bel,scans):
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
    p_dynm = np.array([[0.0,0.05,0.0],
                       [0.05,0.8,0.05],
                       [0.0,0.05,0.0],
                       [p_outside/(2*bel.shape[0]+2*bel.shape[1]),0,0]])
    p_dynm[:3,:]/=p_dynm[:3,:].sum()
    
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



def simulate(bel,state,plotting=False):
    scans = sonarScans(20)
    p_obs,p_dyn = defineModels(bel,scans)

    
    if(plotting):
        fig,ax = plt.subplots()
    


    pos1 = [[5,5],[20,5],[35,5],[35,15],[20,15],[5,15]]
    pos2 = [[5,35],[20,35],[35,35],[35,25],[20,25],[5,25]]

    posnum = 0
    ang = 0
    for i in range(200):
        print(i)
        z1 = [pos1[posnum][0],pos1[posnum][1],ang]
        z2 = [pos2[posnum][0],pos2[posnum][1],ang]

        bel = bayes(bel,z1,p_dyn,p_obs)
        bel = bayes(bel,z2,p_dyn,p_obs)
        if ang == 19:
            posnum+=1
        ang = (ang+1)%20
        if(plotting):
            x = np.array(range(40))
            X,Y = np.meshgrid(x,x)
            cp = plt.contourf(X, Y, bel.transpose())
            if i ==0:
                fig.colorbar(cp)

            ax.set_title('Contour Plot')
            plt.pause(0.05)




if __name__=="__main__":
    bel = np.random.rand(40,40)
    bel/=bel.sum()
    state = np.array([np.random.randint(0,40),np.random.randint(0,40)])
    simulate(bel,state,True)
    # print(state.shape)