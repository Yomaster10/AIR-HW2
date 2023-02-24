#!/usr/bin/env python
import rospy
import sys
import multi_move_base 

def vacuum_cleaning(agent_id, agent_max_vel):
       
    x = 0
    y = 1   
    print('cleaning (%d,%d)'%(x,y))
    result = multi_move_base.move(agent_id, x,y)
    
    print('moving agent %d'%agent_id)
    x = 1
    y = 0   
    print('cleaning (%d,%d)'%(x,y))
    result = multi_move_base.move(agent_id, x,y)

def inspection(agent_id, agent_max_vel):
    print('start inspection')
    raise NotImplementedError



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('assignment3')

    exec_mode = sys.argv[1] 
    print('exec_mode:' + exec_mode)        

    print('agent id:' + agent_id)        
    if exec_mode == 'cleaning':
	agent_id = sys.argv[2]
	agent_max_vel = sys.argv[3]
        vacuum_cleaning(agent_id, agent_max_vel)

    elif exec_mode == 'inspection':
	agent_id = sys.argv[2]
	agent_max_vel = sys.argv[3]
        inspection(agent_id, agent_max_vel)
	
    else:
        print("Code not found")
        raise NotImplementedError

