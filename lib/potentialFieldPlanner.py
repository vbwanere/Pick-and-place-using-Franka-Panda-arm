import numpy as np
from math import pi, acos
from scipy.linalg import null_space
from copy import deepcopy
from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap


class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])#respect this!!!
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()
    min_step_size = 2e-4
    def __init__(self, tol=0.7, max_steps=500):#2):#1000):#, min_step_size=2e-4):#0.3):#0.5):#0.4):#0.35):#2e-2):#1e-2):#5e-3):#1e-4):#1e-5):#
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        #self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

    @staticmethod
    def attractive_force(target, current):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the 
        target joint position and the current joint position 

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE

        att_f = np.zeros((3, 1)) 
        currentpos=current
        targetpos=target
        diffpos=currentpos-targetpos
        zeta=8#5#10#1#0.1#
        att_f=-zeta*diffpos#for each joint!
        ## END STUDENT CODE
        att_f=att_f.reshape((3,1))
        return att_f

    @staticmethod
    def repulsive_force(obstacle, current, unitvec=np.zeros((3,1))):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the 
        obstacle and the current joint position 

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE

        rep_f = np.zeros((3, 1))
        #do we really need the unitvec provided from outside?

        gradrhooiq=unitvec
        currentr=current.reshape((-1,3))#here it is equivalent to (1,3), right?

        rhooiq,gradrhooiq2=PotentialFieldPlanner.dist_point2box(currentr, obstacle)#pay attention to the dimension of current
        #('rhooiq',rhooiq)
        if rhooiq<=1e-6:#gradrhooiq2 is a unit vector!
            rhooiq=1e-6
        rho0=0.085#0.1#0.25#0.5#0.2#0.4#
        eta=4#3#2#1#0.1#0.05#tune by yourself!#
        if rhooiq>rho0:
            req_f=np.zeros((3,1))
        else:
            #rep_f=strength*(1/rhooiq-1/rho0)*1/(rhooiq**2)*gradrhooiq
            rep_f = eta * (1 / rhooiq - 1 / rho0) * 1 / (rhooiq ** 2) * gradrhooiq2
            rep_f=rep_f.reshape((3,1))
        ## END STUDENT CODE

        return rep_f

    @staticmethod
    def dist_point2box(p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point 
    
        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector 
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    @staticmethod
    def compute_forces(target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE

        joint_forces = np.zeros((3, 7))

        #just to test if the start and goal configuration is collision free!



        #we need to write a loop for it!
        for i in range(7):
            attractforcei=PotentialFieldPlanner.attractive_force(target[:,i], current[:,i])#target i will be the 3d target position of joint i
            repulsiveforcei=np.zeros_like(attractforcei)
            if len(obstacle)==0:#(not obstacle.all()):
                repulsiveforcei = np.zeros_like(attractforcei)
            else:
                for j in range(obstacle.shape[0]):
                    repulsiveforceij=PotentialFieldPlanner.repulsive_force(obstacle[j], current[:,i])#, unitvec=np.zeros((3,1)))
                    #repulsiveforceij = PotentialFieldPlanner.repulsive_force(obstacle[j], target[:, i])  #
                    repulsiveforcei+=repulsiveforceij
            #print('repulsiveforcei',repulsiveforcei)
            totalforcei=attractforcei+repulsiveforcei
            totalforceir=totalforcei.T
            joint_forces[:,i]=totalforceir#pay attention to the dimension!
        ## END STUDENT CODE

        return joint_forces
    
    @staticmethod
    def compute_torques(joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        joint_torques = np.zeros((1, 7)) 
        #LOOP OF 7!
        #print('q.shape', q.shape)#(1,7)
        Jacobian=calcJacobian(q)
        Jv=Jacobian[0:3]
        torque=np.zeros(7,)
        for i in range(7):
            Jveffectivei=np.zeros_like(Jv)
            Jveffectivei[:,0:i+1]=Jv[:,0:i+1]
            Jveffectiveit=np.transpose(Jveffectivei)
            torquei=Jveffectiveit@joint_forces[:,i]
            torque+=torquei

        joint_torques=torque.reshape((1,7))
        ## END STUDENT CODE

        return joint_torques

    @staticmethod
    def q_distance(target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE

        distance = 0#should it be calculated in configuration/7d space or in task/3d space
        distance=np.linalg.norm((target-current))
        ## END STUDENT CODE

        return distance
    
    @staticmethod
    def compute_gradient(q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        ## STUDENT CODE STARTS HERE

        dq = np.zeros((1, 7))
        #print('map_struct',map_struct)
        obstacle=map_struct[0]#how to deal with it?
        #print('obstacle',obstacle)#
        #print('obstacle.shape', obstacle.shape)#(1,6)!#Now I know what to do!!!
        #print('q.shape',q.shape,'target.shape',target.shape)#q is (1,7), target is 7
        qflat=q.reshape((7))
        #print('qflat.shape', qflat.shape)
        #jointPositionsq, T0eq=PotentialFieldPlanner.fk.forward2(qflat)#we don't need to flatten this q anymore
        #jointPositionst, T0et=PotentialFieldPlanner.fk.forward2(target)
        jointPositionsq, T0eq = PotentialFieldPlanner.fk.forward(qflat)  # we don't need to flatten this q anymore
        jointPositionst, T0et = PotentialFieldPlanner.fk.forward(target)
        jpqmatter=jointPositionsq[1:]
        jptmatter=jointPositionst[1:]
        jpq=np.transpose(jpqmatter)
        jpt=np.transpose(jptmatter)

        joint_forces=PotentialFieldPlanner.compute_forces(jpt, obstacle, jpq)
        tauqi=PotentialFieldPlanner.compute_torques(joint_forces, q)
        #print('tauqi',tauqi)
        #print('tauqinorm',np.linalg.norm(tauqi))
        tauqinorm=np.linalg.norm(tauqi)#have doubt
        tauqinormal=tauqi/(tauqinorm+1e-8)#tauqi/(tauqinorm+1e-3)#
        jptlast=jpt[:,-1]
        jpqlast=jpq[:,-1]
        #alpha=0.05#0.01#0.1#0.025#0.1#a hyperparameter I pick!
        distlast=PotentialFieldPlanner.q_distance(jptlast, jpqlast)
        alpha =0.02#0.002# 0.01#1* np.maximum(np.minimum(1,distlast),1e-4) # 0.01#0.1#0.025#0.1#a hyperparameter I pick!
        #print('distlast',distlast)
        if distlast<=alpha:
            stepsize=PotentialFieldPlanner.min_step_size
            alpha=np.maximum(distlast,stepsize)#2e-4)#PotentialFieldPlanner.min_step_size)#self.min_step_size)#min_step_size/5)#
        #print('alpha',alpha)
        dqs=alpha*tauqinormal#dqs means dq scalar
        dq=dqs.reshape((1,7))
        ## END STUDENT CODE

        return dq,tauqinorm

    ###############################
    ### Potential Field Solver  ###
    ###############################

    def withinlimit(self,qi):
        qi = qi.reshape(7)
        qi = np.clip(qi, PotentialFieldPlanner.lower + 0.01, PotentialFieldPlanner.upper - 0.01)
        qi = qi.reshape((1, 7))
        return qi
    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the starting configuration to
        the goal configuration.

        INPUTS:

        map_struct - a map struct containing min and max positions of obstacle boxes 
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """

        q_path = np.array([]).reshape(0,7)
        obstacle = map_struct[0]  # how to deal with it?
        print('obstacle', obstacle)  # [] when no obstacle
        jpstart, T0eqstart = self.fk.forward(start)
        jps07 = jpstart[0:7]
        jps18 = jpstart[1:8]
        collisions = np.zeros((1))
        if len(obstacle) > 0:
            for k in range(obstacle.shape[0]):
                obsk = obstacle[k]
                collision7s = detectCollision(jps07, jps18, obsk)
                collisions = np.concatenate((collisions, collision7s))
        collisions = collisions[1:]
        if np.sum(collisions) > 0:
            print('start point is already in collision! Return empty list!')
            return q_path
        jpgoal, T0eqgoal = self.fk.forward(goal)
        jpg07 = jpgoal[0:7]
        jpg18 = jpgoal[1:8]
        collisiong = np.zeros((1))
        if len(obstacle) > 0:
            for k in range(obstacle.shape[0]):
                obsk = obstacle[k]
                collision7g = detectCollision(jpg07, jpg18, obsk)
                collisiong = np.concatenate((collisiong, collision7g))
        collisiong = collisiong[1:]
        if np.sum(collisiong) > 0:
            print('goal point is already in collision! Return empty list!')
            return q_path
        nsteps=0#nsteps means number of steps#I change this!#
        qi=start.reshape((1,7))
        q_path = np.vstack((q_path, qi))#add the starting configuration. Don't forget that!
        #print('qi.shape',qi.shape)
        target=goal
        #print('target.shape',target.shape)#(7,)

        while nsteps<=self.max_steps:#True:#I change this!#

            ## STUDENT CODE STARTS HERE
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            
            # Compute gradient 
            # TODO: this is how to change your joint angles
            #q=start
            dq,tauqinorm=self.compute_gradient(qi, target, map_struct)#put the normalization here!!!!!!!!
            #print('dq.shape', dq.shape)
            #print('dqnorm',np.linalg.norm(dq))
            qi+=dq
            #qi=qi.reshape(7)
            #qi=np.clip(qi,PotentialFieldPlanner.upper-0.01,PotentialFieldPlanner.lower+0.01)
            #qi=qi.reshape((1,7))
            qi=self.withinlimit(qi)
            #print('qi',qi)
            #q_path=np.vstack((q_path,qi))#I shouldn't do it yet!
            # Termination Conditions
            #japq,T0eq=self.fk.forward2(qi)
            #japt,T0et=self.fk.forward2(target)
            japq, T0eq = self.fk.forward(qi)
            japt, T0et = self.fk.forward(target)
            oqi=japq[-1]#T0eq[0:3,3]
            #print('oqi',oqi)
            oqt=japt[-1]#T0et[0:3,3]
            #print('oqt',oqt)
            distoq=np.linalg.norm((oqi-oqt))
            #print('distoq',distoq)
            #zps = np.abs(target[-1] - qi[0,-1])  # zps means zero point seven
            #print('zps',zps)
            #if self.q_distance(qi,target)<=np.sqrt(self.min_step_size**2+zps**2):#self.min_step_size+zps:#True: # TODO: check termination conditions
            if np.linalg.norm(qi[0,0:6] - target[0:6]) < self.tol:#self.min_step_size:#epsilon:
            #if distoq <= self.min_step_size:  # True: # TODO: check termination conditions
                q_path=np.vstack((q_path,target))#do I add goal configuration by myself?
                return q_path#break # exit the while loop if conditions are met!




            # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
            # TODO: when detect a local minima, implement a random walk
            tth = 1e-2#2e-2  # 2.5e-2#
            lmflag=0#lm means local minimum
            if q_path.shape[0] > 7:  # 7 is enough I think
                qm6 = q_path[-6]
                qm4 = q_path[-4]
                qm3 = q_path[-3]
                qm2 = q_path[-2]
                #print('qm2', qm2)
                print('qi', qi)
                print('distance42', self.q_distance(qm4, qm2))
                print('distance20', self.q_distance(qm2, qi))
                print('distance63', self.q_distance(qm6, qm3))
                print('distance30', self.q_distance(qm3, qi))
                #tm=np.sqrt(0.30 ** 2 + zps ** 2)#np.sqrt(0.35 ** 2 + zps ** 2)#
                tm=0.3
                qdist=np.linalg.norm(qi[0, 0:6] - target[0:6])
                if (self.q_distance(qm2,qi)<tth and self.q_distance(qm4,qm2)<tth and qdist>tm) or (self.q_distance(qm3,qi)<tth and self.q_distance(qm6,qm3)<tth and qdist>tm):
                    print('avoid local minimum!')
                    qdisp = (target - qi)
                    dqnoisecoef = 0.5*np.random.rand(*qdisp.shape)#
                    dqnoise = np.multiply(qdisp, dqnoisecoef)
                    #dqnoise=np.random.uniform(low=-0.5,high=0.5,size=dq.shape)#
                    # dqnoise = 0.4*(np.random.rand(*dq.shape)-0.5)
                    # dqnoise = np.random.randn(*dq.shape)  # 0.4*np.random.randn(*dq.shape)#
                    # dqnoise = dqnoise / np.linalg.norm(dqnoise)  # normalize to avoid it from going too big!
                    qi += dqnoise
                    lmflag=1



            # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
            # TODO: Figure out how to use the provided function 
            #don't we check the collision of the last step?

            jp,T0eqi=self.fk.forward(qi)
            jp07=jp[0:7]
            jp18=jp[1:8]
            dqescape = np.zeros_like(dq)
            qiescape = qi + dqescape
            collision=np.zeros((1))
            width=0.12#0.1#0.06#0.04#0.02#0.05#0#
            expansion=np.array([-width,-width,-width,width,width,width])
            if len(obstacle)>0:
                for k in range(obstacle.shape[0]):
                    obsk=obstacle[k]
                    #obsk=obsk+expansion
                    print('obsk',obsk)
                    collision7=detectCollision(jp07, jp18, obsk)
                    collision=np.concatenate((collision,collision7))
            collision = collision[1:]
            ncoll=0
            while np.sum(collision)>0 and ncoll<=50:#20:#this means that it runs into collision
                #print('might collide!')
                if ncoll<2 and lmflag==0:
                    laststep=qi-q_path[-1]
                    dqescape=-3*laststep
                else:
                    dqescape = np.random.uniform(low=-0.32,high=0.32,size=dq.shape)#np.random.randn(*dq.shape)  # 0.4*np.random.randn(*dq.shape)#
                    #dqescape = dqescape / np.linalg.norm(dqescape)  # normalize to avoid it from going too big!
                qiescape=qi+dqescape#keep qi fixed!
                qiescape = self.withinlimit(qiescape)
                print('qiescape',qiescape)
                jpe, T0eqie = self.fk.forward(qiescape)
                jpe07 = jpe[0:7]#e means escape
                jpe18 = jpe[1:8]
                collision = np.zeros((1))
                if len(obstacle) > 0:
                    for k in range(obstacle.shape[0]):
                        obsk = obstacle[k]
                        obsk = obsk + expansion
                        collision7 = detectCollision(jpe07, jpe18, obsk)
                        collision = np.concatenate((collision, collision7))
                collision=collision[1:]
                #collision7 = detectCollision(jpe07, jpe18, box)
                ncoll+=1
                #print('ncoll:',ncoll,'collision joints:',collision)
                if np.sum(collision)==0:
                    #print('find a collision free dq!')
                    break
                #continue
            #qinew=qiescape
            qi = qiescape#free of collision now! escaping from the collision now!
            #for i in range(7-1):
                #detectCollision(linePt1, linePt2, box)
            q_path = np.vstack((q_path, qi))  #
            print('iteration: ',len(q_path))
            nsteps += 1
            ## END STUDENT CODE
        #q_path = np.vstack((q_path, target))  #
        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    map_struct = loadmap("../maps/map2.txt")#map3.txt")#map4.txt")#map5.txt")#map1.txt")#("../maps/emptyMap.txt")#
    start = np.array([0, 0.4, 0, -2.5, 0, 2.7, 0.707])#([0,-1,0,-2,0,1.57,0.0])#np.array([-1.2, 1.57 , 1.57, -2.07, 1.57, 3.32, 0.7])#np.array([-1.2, 1.57 , 1.57, -2.07, 1.69, 3.51, 0.7])##np.array([-1.1, 1.4 , 1.47, -2.17, -1.47, 1.47, 0.7])#we should do FK to both the start and the goal configuration!
    goal = np.array([1.9, 1.57, -1.57, -1.57, 1.57, 1.57, 0.707])#([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])# need FK to turn this 7*1 to 3*1
    #map 5 goal 1 is bad? 6, 7 are 0!map 4 goal 1 box 1 6 is only 0.07265!map 3 goal 1 is bad? 5, 6, 7 are 0!#map 2 goal 1 is too close, box 1, 6, 0.06128? box2, 1, 0.01894??#map 1 goal 1 is fine! all more than 0.16!
    #map 5 goal 2 is good, at least 0.154 meters margin!#map 4 goal 2 is good, at least 0.15 meters margin!#map 3 goal 2 is good, at least 0.2 meters margin!#map 2 goal 2 is good, at least 0.237 meters margin!#map 1 goal 2 is good, at least 0.233 meters margin!#map
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    # show results
    for i in range(q_path.shape[0]):
        error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error}'.format(error=error))

    print("q path: ", q_path)