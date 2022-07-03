# -*- coding: utf-8 -*-
""" Created on Sat Jul  2 18:03:54 2022 @author: henrique hafner ferreira """

import json

def CreateJointTable(MateTable, debug=0):
    JointTable = []    
    for i in range(len(MateTable)):#Run over mates to see links pairs.
        title_prefix = MateTable[i][0]
        title_prefix = title_prefix.lower()
        title_prefix = title_prefix[:5]
        if title_prefix != 'joint': #instruction to ignore this mate, it is not labeled to define joints
            print("  Ignoring mate: ",MateTable[i][0]," no 'joint-' prefix.")
            continue
        mate_properties = MateTable[i]
        body_a = mate_properties[2][0]
        body_b = mate_properties[3][0]
        mate_type = mate_properties[1]
        mate_bodies = [body_a,body_b]
        mate_bodies.sort()
        #Check if theese specific 2 bodies already have a joint entry in JointTable
        create_new_joint = True
        if debug: print("In to mate: ",mate_type,"index:",i)
        for j in range(len(JointTable)):
            if JointTable[j][0] == mate_bodies:
                #Add the mate relation in case of finding the same body-body entry.
                create_new_joint = False
                JointTable[j][1].append(mate_properties) 
                if debug: print("  Added to joint: ",JointTable[j][0])
        if create_new_joint == True:
            JointTable.append([mate_bodies, [mate_properties]])
            if debug: print("  New joint created for: ",mate_bodies)
    return JointTable
 
def gazebo_links_data(doc_features, model_name ):
    gazebo_links = []
    for link in doc_features:
        feature_type = link.GetTypeName2
        if feature_type == 'Reference':
            link_properties = {}
            link_properties['linkname_value']   = link.Name
            link_properties['linkpose_value']   = '0.0 0.0 0.0 0.0 0.0 0.0'
            link_properties['mass_value']       = '1'
            link_properties['visualname_value'] = 'visual_'+link.Name
            link_properties['visualpose_value'] = '0.0 0.0 0.0 0.0 0.0 0.0'
            link_properties['meshpath_value']   =  'model://'+model_name+'//meshes//'+link.Name+'.STL'
            gazebo_links.append(link_properties)
    print('gazebo_links_data: ',len(gazebo_links),' links found.')
    return gazebo_links

def reference_transf(vector_ref1 ):
    vector_ref2 = [0]*len(vector_ref1)
    vector_ref2[0] = vector_ref1[1]
    vector_ref2[1] = vector_ref1[2]
    vector_ref2[2] = vector_ref1[0]
    return vector_ref2

def gazebo_joints_data(joint_table):
    map_nomeclature = {3:'revolute',25:'prismatic',33:'prismatic'}
    gazebo_joints = []
    for joint in joint_table:
        joint_properties = {}

        list_geometric_relations = joint[1]
        joint_type_id = 1
        for geometric_relation in list_geometric_relations:
            type_geometric_relation = int(geometric_relation[1][:2])
            joint_type_id = joint_type_id*type_geometric_relation #Prime number indexing
        joint_type = map_nomeclature.get(joint_type_id)
        if not joint_type:
            joint_type = 'unknow- '+str(joint_type_id)+'-joint_type_id.'
        joint_properties['jointtype_value'] = joint_type

        joint_parent = joint[0][0]
        folder_presence = joint_parent.find('/')
        if folder_presence > 0:
            joint_parent = joint_parent[:folder_presence]

        joint_properties['parent_value'] = joint_parent

        joint_child  = joint[0][1]
        joint_properties['child_value'] = joint_child

        joint_name = joint_type+':'+joint_parent+'&'+joint_child
        joint_properties['jointname_value'] = joint_name

        if joint_type_id: # To Implement , joint cases.
            geometric_relation_1 = joint[1]
            joint_axis = geometric_relation_1[0][2][1][3:6]
            joint_axis = reference_transf(joint_axis)
            joint_axis_str = ' '+str(joint_axis[0])+' '+str(joint_axis[1])+' '+str(joint_axis[2])+' '
            joint_properties['jointaxis_value'] = joint_axis_str
            joint_pose = geometric_relation_1[0][2][1][0:3]
            joint_pose = reference_transf(joint_pose)
            joint_pose_str = ' '+str(joint_pose[0])+' '+str(joint_pose[1])+' '+str(joint_pose[2])+' '
            joint_properties['jointpose_value'] = joint_pose_str

        gazebo_joints.append(joint_properties)
    print('gazebo_joints_data: ',len(gazebo_joints),' joints found.')
    return gazebo_joints

def export_data(data):
    data_stringfied = json.dumps(data)
    with open('gazebo_model_pythondata.json', 'w') as output_file:
        print(data_stringfied, file=output_file)
    return True

