import random
import numpy as np
import torch
import roma
import torch, torch.nn.functional as F

def set_randomness(seed_set):

    random.seed(seed_set)
    np.random.seed(seed_set)
    torch.manual_seed(seed_set)
    torch.cuda.manual_seed(seed_set)
    # # Uncomment the following lines if deterministic behavior is required
    # torch.backends.cudnn.deterministic = True
    # torch.backends.cudnn.benchmark = False

def initialize_device(device_id):
    try:
        device = torch.device(f'cuda:{device_id}' if torch.cuda.is_available() else 'cpu')
        if device.type == 'cuda':
            print(f"Using GPU: {device_id}")
        else:
            print("GPU not available, using CPU instead.")
    except Exception as e:
        print(f"Error with GPU selection: {e}")
        device = torch.device('cpu')
        print("Using CPU instead.")
    return device



def rot6d_to_matrix(x):

    a1 = F.normalize(x[:, 0:3], dim=1)
    a2 = x[:, 3:6]

    b2 = F.normalize(a2 - (a1 * a2).sum(1, keepdim=True) * a1, dim=1)
    b3 = torch.cross(a1, b2, dim=1)
    # pose = compute_rotation_matrix_from_ortho6d(x)

    return torch.stack((a1, b2, b3), dim=2)    


def geodesic_loss(R_pred, R_gt):
    
    dR = R_pred.transpose(-2, -1) @ R_gt
    cos_theta = (dR.diagonal(offset=0, dim1=-2, dim2=-1).sum(-1) - 1) / 2
    cos_theta = cos_theta.clamp(-1.0 + 1e-6, 1.0 - 1e-6)  
    theta = torch.acos(cos_theta)

    return theta.mean()


def geodesic_loss_squared(R_pred, R_gt, eps=1e-7):
    # dR = R_pred.transpose(-2, -1) @ R_gt
    # cos_theta = ((dR.diagonal(offset=0, dim1=-2, dim2=-1).sum(-1) - 1) / 2
    #              ).clamp(-1.0 + eps, 1.0 - eps)

    cos_theta = roma.utils.rotmat_cosine_angle(R_pred.transpose(-2, -1) @ R_gt)
    # theta = roma.utils.rotmat_geodesic_distance(R_pred, R_gt)
    return (1 - cos_theta).mean()



# batch*n
def normalize_vector( v):
    batch=v.shape[0]
    v_mag = torch.sqrt(v.pow(2).sum(1))# batch
    v_mag = torch.max(v_mag, torch.autograd.Variable(torch.FloatTensor([1e-8]).cuda()))
    v_mag = v_mag.view(batch,1).expand(batch,v.shape[1])
    v = v/v_mag
    return v
    
# u, v batch*n
def cross_product( u, v):
    batch = u.shape[0]
    #print (u.shape)
    #print (v.shape)
    i = u[:,1]*v[:,2] - u[:,2]*v[:,1]
    j = u[:,2]*v[:,0] - u[:,0]*v[:,2]
    k = u[:,0]*v[:,1] - u[:,1]*v[:,0]
        
    out = torch.cat((i.view(batch,1), j.view(batch,1), k.view(batch,1)),1)#batch*3
        
    return out
        
    
#poses batch*6
#poses
def compute_rotation_matrix_from_ortho6d(poses):
    x_raw = poses[:,0:3]#batch*3
    y_raw = poses[:,3:6]#batch*3
        
    x = normalize_vector(x_raw) #batch*3
    z = cross_product(x,y_raw) #batch*3
    z = normalize_vector(z)#batch*3
    y = cross_product(z,x)#batch*3
        
    x = x.view(-1,3,1)
    y = y.view(-1,3,1)
    z = z.view(-1,3,1)
    matrix = torch.cat((x,y,z), 2) #batch*3*3
    return matrix
