import numpy as np
import pickle

import torch


class AnatomyModel():
    def __init__(self, model_path, shapedir_path=None):
        """
    SMPL model.

    Parameter:
    ---------
    model_path: Path to the SMPL model parameters, pre-processed by
    `preprocess.py`.

    """
        with open(model_path, 'rb') as f:
            params = pickle.load(f)

            self.weights = params['weights']
            self.v_template = params['v_template']
            self.faces = params['faces']
            self.kintree_table = np.array([params['hierarchies'],
                                           [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                            21, 22, 23]])
            self.J = params['bones'][:, :3, 3]

        id_to_col = {
            self.kintree_table[1, i]: i for i in range(self.kintree_table.shape[1])
        }
        self.parent = {
            i: id_to_col[self.kintree_table[0, i]]
            for i in range(1, self.kintree_table.shape[1])
        }

        self.pose_shape = [24, 3]
        self.trans_shape = [3]
        self.beta_shape = [10]

        self.pose = np.zeros(self.pose_shape)
        self.beta = np.zeros(self.beta_shape)
        self.trans = np.zeros(self.trans_shape)

        self.verts = None
        self.R = None

        if shapedir_path:
            self.shapedirs = np.load(shapedir_path)
        else:
            self.shapedirs = None

        self.update()

    def set_params(self, pose=None, beta=None, trans=None):
        """
    Set pose, shape, and/or translation parameters of SMPL model. Verices of the
    model will be updated and returned.

    Parameters:
    ---------
    pose: Also known as 'theta', a [24,3] matrix indicating child joint rotation
    relative to parent joint. For root joint it's global orientation.
    Represented in a axis-angle format.

    beta: Parameter for model shape. A vector of shape [10]. Coefficients for
    PCA component. Only 10 components were released by MPI.

    trans: Global translation of shape [3].

    Return:
    ------
    Updated vertices.

    """
        if pose is not None:
            self.pose = pose
        if beta is not None:
            self.beta = beta
        if trans is not None:
            self.trans = trans
        self.update()
        return self.verts

    def update(self):
        """
    Called automatically when parameters are updated.

    """
        if self.shapedirs is not None:
            v_shaped = self.v_template + self.shapedirs.dot(self.beta)
        else:
            v_shaped = self.v_template
        # self.pose[0, 1] = 0
        # self.pose[9] = np.array([0,0,0])
        pose_cube = self.pose.reshape((-1, 1, 3))
        # rotation matrix for each joint
        self.R = self.rodrigues(pose_cube)
        I_cube = np.broadcast_to(
            np.expand_dims(np.eye(3), axis=0),
            (self.R.shape[0] - 1, 3, 3)
        )
        lrotmin = (self.R[1:] - I_cube).ravel()
        # how pose affect body shape in zero pose
        v_posed = v_shaped
        # world transformation of each joint
        G = np.empty((self.kintree_table.shape[1], 4, 4))
        G[0] = self.with_zeros(np.hstack((self.R[0], self.J[0, :].reshape([3, 1]))))
        for i in range(1, self.kintree_table.shape[1]):
            G[i] = G[self.parent[i]].dot(
                self.with_zeros(
                    np.hstack(
                        [self.R[i], ((self.J[i, :] - self.J[self.parent[i], :]).reshape([3, 1]))]
                    )
                )
            )
        G = G - self.pack(
            np.matmul(
                G,
                np.hstack([self.J, np.zeros([24, 1])]).reshape([24, 4, 1])
            )
        )
        # transformation of each vertex
        T = np.tensordot(self.weights, G, axes=[[1], [0]])
        rest_shape_h = np.hstack((v_posed, np.ones([v_posed.shape[0], 1])))
        v = np.matmul(T, rest_shape_h.reshape([-1, 4, 1])).reshape([-1, 4])[:, :3]
        self.verts = v + self.trans.reshape([1, 3])

    def rodrigues(self, r):
        """
    Rodrigues' rotation formula that turns axis-angle vector into rotation
    matrix in a batch-ed manner.

    Parameter:
    ----------
    r: Axis-angle rotation vector of shape [batch_size, 1, 3].

    Return:
    -------
    Rotation matrix of shape [batch_size, 3, 3].

    """
        theta = np.linalg.norm(r, axis=(1, 2), keepdims=True)
        # avoid zero divide
        theta = np.maximum(theta, np.finfo(r.dtype).eps)
        r_hat = r / theta
        cos = np.cos(theta)
        z_stick = np.zeros(theta.shape[0])
        m = np.dstack([
            z_stick, -r_hat[:, 0, 2], r_hat[:, 0, 1],
            r_hat[:, 0, 2], z_stick, -r_hat[:, 0, 0],
            -r_hat[:, 0, 1], r_hat[:, 0, 0], z_stick]
        ).reshape([-1, 3, 3])
        i_cube = np.broadcast_to(
            np.expand_dims(np.eye(3), axis=0),
            [theta.shape[0], 3, 3]
        )
        A = np.transpose(r_hat, axes=[0, 2, 1])
        B = r_hat
        dot = np.matmul(A, B)
        R = cos * i_cube + (1 - cos) * dot + np.sin(theta) * m
        return R

    def with_zeros(self, x):
        """
    Append a [0, 0, 0, 1] vector to a [3, 4] matrix.

    Parameter:
    ---------
    x: Matrix to be appended.

    Return:
    ------
    Matrix after appending of shape [4,4]

    """
        return np.vstack((x, np.array([[0.0, 0.0, 0.0, 1.0]])))

    def pack(self, x):
        """
    Append zero matrices of shape [4, 3] to vectors of [4, 1] shape in a batched
    manner.

    Parameter:
    ----------
    x: Matrices to be appended of shape [batch_size, 4, 1]

    Return:
    ------
    Matrix of shape [batch_size, 4, 4] after appending.

    """
        return np.dstack((np.zeros((x.shape[0], 4, 3)), x))

    def save_to_obj(self, path):
        """
    Save the SMPL model into .obj file.

    Parameter:
    ---------
    path: Path to save.

    """
        with open(path, 'w') as fp:
            for v in self.verts:
                fp.write('v %f %f %f\n' % (v[0], v[1], v[2]))
            for f in self.faces + 1:
                fp.write('f %d %d %d\n' % (f[0], f[1], f[2]))


if __name__ == '__main__':
    import smplx
    import trimesh
    skel_file = 'data/skel_params.pkl'
    artery_file = 'data/artery_params.pkl'
    skel_shapedir_file = 'data/skel_shapedir.npy'
    artery_shapedir_file = 'data/artery_shapedir.npy'
    skel_model = AnatomyModel(skel_file, skel_shapedir_file)
    artery_model = AnatomyModel(artery_file, artery_shapedir_file)
    np.random.seed(9608)
    pose = (np.random.rand(*skel_model.pose_shape) - 0.5) * 0
    theta = pose.copy()
    index = [0, 1, 4, 7, 10, 3, 6, 9, 12, 15, 13, 16, 18, 20, 22, 14, 17, 19, 21, 23, 2, 5, 8, 11]
    for i in range(pose.shape[0]):
        pose[i] = theta[index[i]]
    beta = np.zeros([*skel_model.beta_shape])
    beta[1] = 1.5
    beta[1] = 1.5
    beta[2] = 1.5

    smpl = smplx.create('/home/olefine/Desktop/SLP-3Dfits/models')
    # Create the skin model with shape and pose parameters
    skin_model = smpl(
        betas=torch.from_numpy(beta).unsqueeze(0).float(),
        body_pose=torch.from_numpy(theta.reshape(1, 72))[:, 3:].float(),
        global_orient=torch.from_numpy(theta.reshape(1, 72))[:, :3].float()
    )

    # Extract vertices and faces from the skin model
    vertices = skin_model.vertices.detach().cpu().squeeze(0).numpy()
    faces = smpl.faces

    # Create a trimesh object from vertices and faces
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

    # Save the mesh as an OBJ file
    output_path = 'skin_model.obj'
    mesh.export(output_path)

    skel_model.set_params(pose=pose, beta=beta)
    skel_model.save_to_obj('./skel_np.obj')
    artery_model.set_params(pose=pose, beta=beta)
    artery_model.save_to_obj('./artery_np.obj')
