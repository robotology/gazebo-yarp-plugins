from xml.dom import minidom
import os 
import random


def generate_random_pedestrians(num_ped, bounds=[-10,-10, 10, 10], max_num_wayp=2, cyclic=True):
    actors = []
    group_actor_name = []
    poses = []
    cyclic = []
    waypoints = []
    for n in range(num_ped):
        actors.append('actor'+str(n+1))
        group_actor_name.append('actor'+str(random.randint(1,num_ped)))
        poses.append(str(random.uniform(bounds[0],bounds[2])) + ' ' +
                    str(random.uniform(bounds[1],bounds[3])) + ' ' +
                    '0 0 0 0'
                    )
        cyclic.append('true' if random.random() > 0.5 else 'false')
        temp_waypoints = []
        for m in range(random.randint(0,max_num_wayp)):
            wayp = (str(random.uniform(bounds[0],bounds[2])) + ' ' +
                    str(random.uniform(bounds[1],bounds[3])) + ' ' +
                    '0'
                    )
            temp_waypoints.append(wayp)
        waypoints.append(temp_waypoints)

    return actors, group_actor_name, poses, cyclic, waypoints


actors = ['actor1','actor2','actor3']
group_actor_names = ['actor1','actor1','actor1']
poses =['1 2 3 0 0 0', '4 5 6 0 0 0', '7 8 9 0 0 0']
cyclics = ['true','true','true']
waypoints = []

actors, group_actor_names, poses, cyclics, waypoints = generate_random_pedestrians(10)

root = minidom.Document()
xml = root.createElement('world') 
xml.setAttribute('name', 'default')

for i, name in enumerate(actors):  


    actor = root.createElement('actor') 
    actor.setAttribute('name', name)
    root.appendChild(xml)
      
    pose = root.createElement('pose')
    text = root.createTextNode(poses[i])
    pose.appendChild(text)
    actor.appendChild(pose)

    skin = root.createElement('skin')
    filename = root.createElement('filename')
    text = root.createTextNode('walk.dae')
    filename.appendChild(text)
    scale = root.createElement('scale')
    text = root.createTextNode('1.0')
    scale.appendChild(text)
    skin.appendChild(filename)
    skin.appendChild(scale)
    actor.appendChild(skin)

    animation = root.createElement('animation')
    animation.setAttribute('name', 'walking')
    filename = root.createElement('filename')
    text = root.createTextNode('walk.dae')
    filename.appendChild(text)
    scale = root.createElement('scale')
    text = root.createTextNode('1.0')
    filename.appendChild(text)
    interpolate_x = root.createElement('interpolate_x')
    text = root.createTextNode('true')
    interpolate_x.appendChild(text)
    animation.appendChild(filename)
    animation.appendChild(scale)
    animation.appendChild(interpolate_x)
    actor.appendChild(animation)

    plugin = root.createElement('plugin')
    plugin.setAttribute('name', name + '_plugin')
    plugin.setAttribute('filename', 'libPedestrianSFMPlugin.so')
    velocity = root.createElement('velocity')
    text = root.createTextNode('0.9')
    velocity.appendChild(text)
    plugin.appendChild(velocity)
    radius = root.createElement('radius')
    text = root.createTextNode('0.4')
    radius.appendChild(text)    
    plugin.appendChild(radius)
    animation_factor = root.createElement('animation_factor')
    text = root.createTextNode('5.1')
    animation_factor.appendChild(text)      
    plugin.appendChild(animation_factor)
    animation_name = root.createElement('animation_name')
    text = root.createTextNode('walking')
    animation_name.appendChild(text)      
    plugin.appendChild(animation_name)
    people_distance = root.createElement('people_distance')
    text = root.createTextNode('6.0')
    people_distance.appendChild(text)      
    plugin.appendChild(people_distance)
    goal_weight = root.createElement('goal_weight')
    text = root.createTextNode('2.0')
    goal_weight.appendChild(text)      
    plugin.appendChild(goal_weight)    
    obstacle_weight = root.createElement('obstacle_weight')
    text = root.createTextNode('80.0')
    obstacle_weight.appendChild(text)      
    plugin.appendChild(obstacle_weight)   
    social_weight = root.createElement('social_weight')
    text = root.createTextNode('15.0')
    social_weight.appendChild(text)      
    plugin.appendChild(social_weight) 
    group_gaze_weight = root.createElement('group_gaze_weight')
    text = root.createTextNode('3.0')
    group_gaze_weight.appendChild(text)      
    plugin.appendChild(group_gaze_weight) 
    group_coh_weight = root.createElement('group_coh_weight')
    text = root.createTextNode('2.0')
    group_coh_weight.appendChild(text)      
    plugin.appendChild(group_coh_weight)     
    group_rep_weight = root.createElement('group_rep_weight')
    text = root.createTextNode('1.0')
    group_rep_weight.appendChild(text)      
    plugin.appendChild(group_rep_weight)    
    group = root.createElement('group')
    model = root.createElement('model')
    text = root.createTextNode(group_actor_names[i])
    model.appendChild(text)      
    group.appendChild(model)    
    plugin.appendChild(group)    
    ignore_obstacles = root.createElement('ignore_obstacles')
    model = root.createElement('model')
    text = root.createTextNode('ground_plane')
    model.appendChild(text)      
    ignore_obstacles.appendChild(model)    
    plugin.appendChild(ignore_obstacles)  
    trajectory = root.createElement('trajectory')
    for w in waypoints[i]:  
        waypoint = root.createElement('waypoint')
        text = root.createTextNode(w)
        waypoint.appendChild(text)      
        trajectory.appendChild(waypoint)    
    cyclic = root.createElement('cyclic')
    text = root.createTextNode(cyclics[i])
    cyclic.appendChild(text)      
    trajectory.appendChild(cyclic)  
    plugin.appendChild(trajectory)   
    actor.appendChild(plugin)

    xml.appendChild(actor)

xml_str = root.toprettyxml(indent ="\t") 
  
save_path_file = 'actors.xml'
  
with open(save_path_file, "w") as f:
    with open('header_world.xml', "r") as header:
        f.write(header.read())
    f.write('\n'.join(xml_str.split('\n')[2:])) 
    with open('footer_world.xml', "r") as footer:
        f.write(footer.read())    


