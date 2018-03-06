# may need more imports
import numpy as np
from utils import vec, adj, look_at, length, normalize, hat
from scipy.optimize import nnls

def compute_force_closure(contacts, normals, num_facets, mu, gamma, object_mass, c_of_mass_height = None):
    """ Compute the force closure of some object at contacts, with normal vectors stored in normals
        You can use the line method described in HW2.  if you do you will not need num_facets

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    alpha = np.arctan2(CONTACT_MU, 1)
    l = contacts[0] - contacts[1]
    n1, n2 = normals
    theta1 = np.arccos(np.dot(n1, l)/np.linalg.norm(l))
    theta2 = np.pi - np.arccos(np.dot(n2, l)/np.linalg.norm(l))
    # print("line is:", l)
    # print("theta1 is: ", theta1)
    # print("theta2 is: ", theta2)
    if abs(theta1)>alpha or abs(theta2)>alpha:
        return 0
    return 1



# defined in the book on page 219
def get_grasp_map(contacts, normals, num_facets, mu, gamma):
    """ Compute the grasp map given the contact points and their surface normals

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient

    Returns
    -------
    :obj:`numpy.ndarray` grasp map
    """
    # YOUR CODE HERE
    step = 2*np.pi / num_facets
    up = np.array([0, 0, 1]) 
    grasp_maps = []
    for num in range(len(contacts)):
        R = look_at(normals[num], up)
        p = contacts[num]
        G = np.zeros((num_facets+1, 6))
        for i in range(num_facets):
            f_finger = np.array([np.sin(step*i)*mu, np.cos(step*i)*mu, 1])
            f_object = np.dot(R, f_finger)
            torque = np.dot(np.dot(hat(p), R), f_finger)
            G[i,:3] = f_object
            G[i,3:] = torque
            # print("f_object is: ", f_object)
            # print("torque is: ", torque)
            # print G[i,:]
        G[num_facets,:] = np.append(np.zeros((3,1)),np.dot(R, np.array([0,0,1])))

        G = G.T
        # print(G)
        grasp_maps.append(G)
        # print np.linalg.matrix_rank(G)
    # print(np.concatenate(grasp_maps ,axis = 1))
    return np.concatenate(grasp_maps ,axis = 1)




    #     B.append([np.sin(theta)*mu, 1 , np.cos(theta)*mu, 0, gamma, 0])
    # B = np.reshape(B, (6,num_facets))
    # print(B)






def contact_forces_exist(contacts, normals, num_facets, mu, gamma, desired_wrench):
    """ Compute whether the given grasp (at contacts with surface normals) can produce the desired_wrench.
        will be used for gravity resistance. 

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    desired_wrench : :obj:`numpy.ndarray`
        potential wrench to be produced

    Returns
    -------
    bool : whether contact forces can produce the desired_wrench on the object
    """
    grasp_map = get_grasp_map(contacts, normals, num_facets, mu, gamma)
    try:
        fc, residual = nnls(grasp_map, desired_wrench)
        return fc, residual
    except:
        return None, float('inf')

def compute_gravity_resistance(contacts, normals, num_facets, mu, gamma, object_mass, c_of_mass_height = 0):
    """ Gravity produces some wrench on your object.  Computes whether the grasp can produce and equal and opposite wrench

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE (contact forces exist may be useful here)
    # grasp_map = get_grasp_map(contacts, normals, num_facets, mu, gamma)

    #Linear Force due to gravity
    f = np.array([0, 0, -object_mass*9.8])

    #Torque due to gravity
    l = normalize(contacts[1] - contacts[0])
    up = np.cross(contacts[0] - np.array([0, 0, c_of_mass_height]), l)
    r_length = length(up)
    r = normalize(np.cross(l, up))*r_length
    torque = np.cross(r, f)

    #Total Wrench due to gravity
    Fe = np.append(f, torque).T

    #Get the least square soln
    fc, residual =  contact_forces_exist(contacts, normals, num_facets, mu, gamma, -Fe)

    #Check contacts if torque inside friction cone
    if fc is not None:
        f0 = 0
        f1 = 0
        for i in range(32):
            f0 = f0 + fc[i]
            f1 = f1 + fc[33+i]

        if fc[32] > gamma*f0 or fc[65] > gamma*f1:
            return float('inf')

    return residual



    # return fc, residual
    # return float('inf')
    # print -Fe
    # fc, residual = nnls(grasp_map, -Fe)
    # fc = np.array(fc)
    # print("residual is: ", residual)
    # print(np.linalg.matrix_rank(grasp_map))
    # fc = np.linalg.lstsq(grasp_map, -Fe)[0]
    # G_inv = np.dot(grasp_map.T, np.linalg.inv(np.dot(grasp_map, grasp_map.T)))
    # fc = np.dot(G_inv, -Fe)
    # print fc
    # print np.dot(grasp_map, fc)


def compute_custom_metric(contacts, normals, num_facets, mu, gamma, object_mass, weight_on_force = 0.5):
    """ I suggest Ferrari Canny, but feel free to do anything other metric you find. 

    Parameters
    ----------
    contacts : :obj:`numpy.ndarray`
        obj mesh vertices on which the fingers will be placed
    normals : :obj:`numpy.ndarray`
        obj mesh normals at the contact points
    num_facets : int
        number of vectors to use to approximate the friction cone.  these vectors will be along the friction cone boundary
    mu : float 
        coefficient of friction
    gamma : float
        torsional friction coefficient
    object_mass : float
        mass of the object

    Returns
    -------
    float : quality of the grasp
    """
    # YOUR CODE HERE :)
    alpha = np.arctan2(mu, 1)
    l = contacts[0] - contacts[1]

    if abs(np.dot(np.array([0,0,1]), normalize(l))) > 0.3:
        return float('inf')

    n1, n2 = normals
    theta1 = np.arccos(np.dot(n1, l)/np.linalg.norm(l))
    theta2 = np.pi - np.arccos(np.dot(n2, l)/np.linalg.norm(l))
    if abs(theta1)>alpha or abs(theta2)>alpha:
        return float('inf')
    
    f = np.array([0, 0, -object_mass*9.8])

    #Torque due to gravity
    l = normalize(contacts[1] - contacts[0])
    up = np.cross(contacts[0], l)
    r_length = length(up)
    r = normalize(np.cross(l, up))*r_length
    torque = np.cross(r, f)

    #Total Wrench due to gravity
    Fe = np.append(f, torque).T

    #Get the least square soln
    fc, residual =  contact_forces_exist(contacts, normals, num_facets, mu, gamma, -Fe)

    if residual > 0.01 or fc is None:
        return float('inf')

    #Check contacts if torque inside friction cone
    f0 = 0
    f1 = 0
    if fc is not None:
        for i in range(32):
            f0 = f0 + fc[i]
            f1 = f1 + fc[33+i]

        if fc[32] > gamma*f0 or fc[65] > gamma*f1:
            return float('inf')

    return (1- weight_on_force)*(abs(theta1)+abs(theta2))*180/np.pi + weight_on_force*(abs(f0) + abs(f1))
    