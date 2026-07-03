import cv2
import pyrealsense2 as rs
import torch
import torchvision
import numpy as np
import smplx
import open3d as o3d
from numpy.linalg import svd
import torchgeometry as tgm
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights

import pytorch3d
import pytorch3d.renderer
from scipy.spatial.transform import Rotation

from anatomy_layer import AnatomyModel

from models.cliff_hr48.cliff import CLIFF as cliff_hr48
from common import constants
from common.utils import strip_prefix_if_present, cam_crop2full, video_to_images
from common.utils import estimate_focal_length
from common.imutils import process_image


def render_mesh(vertices, faces, translation, focal_length, height, width, device=None, red=True):
    ''' Render the mesh under camera coordinates
    vertices: (N_v, 3), vertices of mesh
    faces: (N_f, 3), faces of mesh
    translation: (3, ), translations of mesh or camera
    focal_length: float, focal length of camera
    height: int, height of image
    width: int, width of image
    device: "cpu"/"cuda:0", device of torch
    :return: the rgba rendered image
    '''
    if device is None:
        device = vertices.device

    bs = vertices.shape[0]

    # add the translation
    if translation is not None:
        vertices = vertices + translation[:, None, :]

    # upside down the mesh
    # rot = Rotation.from_rotvec(np.pi * np.array([0, 0, 1])).as_matrix().astype(np.float32)
    rot = Rotation.from_euler('z', 180, degrees=True).as_matrix().astype(np.float32)
    rot = torch.from_numpy(rot).to(device).expand(bs, 3, 3)
    faces = faces.expand(bs, *faces.shape).to(device)

    vertices = torch.matmul(rot, vertices.transpose(1, 2)).transpose(1, 2)

    # Initialize each vertex to be white in color.
    verts_rgb = torch.ones_like(vertices)  # (B, V, 3)
    if red:
        verts_rgb[:, :, 1] = 0.0
        verts_rgb[:, :, 2] = 0.0
    textures = pytorch3d.renderer.TexturesVertex(verts_features=verts_rgb)
    mesh = pytorch3d.structures.Meshes(verts=vertices, faces=faces, textures=textures)

    # Initialize a camera.
    cameras = pytorch3d.renderer.PerspectiveCameras(
        focal_length=((2 * focal_length / min(height, width), 2 * focal_length / min(height, width)),),
        device=device,
    )

    # Define the settings for rasterization and shading.
    raster_settings = pytorch3d.renderer.RasterizationSettings(
        image_size=(height, width),   # (H, W)
        # image_size=height,   # (H, W)
        blur_radius=0.0,
        faces_per_pixel=1,
        bin_size=92
    )

    # Define the material
    materials = pytorch3d.renderer.Materials(
        ambient_color=((1, 1, 1),),
        diffuse_color=((1, 1, 1),),
        specular_color=((1, 1, 1),),
        shininess=64,
        device=device
    )

    # Place a directional light in front of the object.
    lights = pytorch3d.renderer.DirectionalLights(device=device, direction=((0, 0, -1),))

    # Create a phong renderer by composing a rasterizer and a shader.
    renderer = pytorch3d.renderer.MeshRenderer(
        rasterizer=pytorch3d.renderer.MeshRasterizer(
            cameras=cameras,
            raster_settings=raster_settings
        ),
        shader=pytorch3d.renderer.SoftPhongShader(
            device=device,
            cameras=cameras,
            lights=lights,
            materials=materials
        )
    )

    # Do rendering
    imgs = renderer(mesh)

    if red:
        # 获取Mesh对象的顶点数据
        vertices = mesh.verts_list()[0]  # 假设只有一个Mesh对象
        # indices = [27538, 7099, 27583, 27584, 7100, 27541, 7101, 27588, 7102, 27590, 7103, 27592, 7104, 27594, 7105, 27595, 27596, 7106]  # 右股动脉

        # indices = [11497, 11520, 11474, 35429, 11432, 35352, 35351] # 右颈动脉
        # indices = [11474, 35429, 11432, 35352, 35351, 11440] # 右颈动脉
        indices = [35429, 11432, 35352, 35351, 11440] # 右颈动脉
        # indices = [16054, 44076, 16008, 43976, 43975, 15973, 43908, 43907, 5163]  # 左颈动脉
        # indices = [44076, 16008, 43976, 43975, 15973, 43908, 43907, 5163]  # 左颈动脉

        vertices = vertices[indices]
        pixels = cameras.transform_points_screen(vertices, image_size=(height, width))
        artery_pixel_coords = pixels[:, :2]

    else:
        artery_pixel_coords = 0
    return imgs, artery_pixel_coords


def kpt_2d_to_3d(kpt_2d, depth_mat, depth_intrin, depth_scale):
    kpt_3d = np.zeros([kpt_2d.shape[0], 3])
    for i in range(kpt_2d.shape[0]):
        x = min(int(kpt_2d[i, 0]), depth_mat.shape[1]-1)
        y = min(int(kpt_2d[i, 1]), depth_mat.shape[0]-1)
        dis = depth_mat[y, x] * depth_scale
        if not dis:  # 如果深度值为0，寻找最近的不为0的深度值
            nonzero = cv2.findNonZero(depth_mat)
            distances = np.sqrt((nonzero[:, :, 0] - x) ** 2 + (nonzero[:, :, 1] - y) ** 2)
            nearest_index = np.argmin(distances)
            nearest_pixel = nonzero[nearest_index]
            x = int(nearest_pixel[0][0])
            y = int(nearest_pixel[0][1])
            dis = depth_mat[y, x] * depth_scale

        kpt_3d[i, 0] = (kpt_2d[i, 0] - depth_intrin.ppx) / depth_intrin.fx * dis
        kpt_3d[i, 1] = (kpt_2d[i, 1] - depth_intrin.ppy) / depth_intrin.fy * dis
        kpt_3d[i, 2] = dis
    return kpt_3d


def get_one_box(det_output, thrd=0.9):
    max_area = 0
    max_bbox = None

    if det_output['boxes'].shape[0] == 0 or thrd < 1e-5:
        return None

    for i in range(det_output['boxes'].shape[0]):
        bbox = det_output['boxes'][i]
        score = det_output['scores'][i]
        if float(score) < thrd:
            continue
        area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        if float(area) > max_area:
            max_bbox = [float(x) for x in bbox]
            max_area = area

    if max_bbox is None:
        return get_one_box(det_output, thrd=thrd - 0.1)

    return max_bbox


if __name__ == '__main__':

    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

    # load human detection model
    det_model = fasterrcnn_resnet50_fpn(weights=FasterRCNN_ResNet50_FPN_Weights.DEFAULT)
    det_model.cuda()
    det_model.eval()
    det_transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])

    # Create the model instance
    cliff = eval("cliff_" + 'hr48')
    cliff_model = cliff(constants.SMPL_MEAN_PARAMS).cuda()
    # Load the pretrained model
    state_dict = torch.load("data/ckpt/hr48-PA43.0_MJE69.0_MVE81.2_3dpw.pt")['model']
    state_dict = strip_prefix_if_present(state_dict, prefix="module.")
    cliff_model.load_state_dict(state_dict, strict=True)
    cliff_model.eval()

    # Setup the SMPL model
    smpl_model = smplx.create(constants.SMPL_MODEL_DIR, "smpl").cuda()

    # realsense
    align = rs.align(rs.stream.color)
    # Create pipeline
    pipeline = rs.pipeline()

    # Create a config object
    config = rs.config()

    # bag file
    # rs.config.enable_device_from_file(config, '/home/olefine/Documents/male_1.bag')
    # profile = pipeline.start(config)
    # playback = profile.get_device().as_playback()
    # playback.set_real_time(False)

    # # real-time
    profile = pipeline.start(config)

    intr = profile.get_stream(
        rs.stream.color).as_video_stream_profile().get_intrinsics()
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)


    extrinsic = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    depth_scale_inv = 1 / depth_scale

    # Configure the pipeline to stream the depth stream
    # Change this parameters according to the recorded bag file resolution
    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.color)

    skel_file = 'data/skel_params.pkl'
    artery_file = 'data/artery_params.pkl'
    skel_shapedir_file = 'data/skel_shapedir.npy'
    artery_shapedir_file = 'data/artery_shapedir.npy'
    skel_model = AnatomyModel(skel_file, skel_shapedir_file)
    artery_model = AnatomyModel(artery_file, artery_shapedir_file)

    while True:

        rs_frames = pipeline.wait_for_frames()
        aligned_frames = align.process(rs_frames)
        rs_depth_frame = aligned_frames.get_depth_frame()
        np_depth = np.asanyarray(rs_depth_frame.get_data())

        rs_color_frame = aligned_frames.get_color_frame()
        image = np.asanyarray(rs_color_frame.get_data())  # 原始图像

        input_image = image.copy()
        det_input = det_transform(input_image).cuda()
        det_output = det_model([det_input])[0]
        bbox = np.array(get_one_box(det_output))  # list

        img_h, img_w, _ = input_image.shape
        focal = estimate_focal_length(img_h, img_w)
        norm_img, center, scale, crop_ul, crop_br, _ = process_image(input_image, bbox)

        norm_img = torch.from_numpy(norm_img).to(device).float().unsqueeze(0)
        center = center.to(device).float().unsqueeze(0)
        scale = torch.from_numpy(np.array(scale)).to(device).float().unsqueeze(0)
        img_h = torch.from_numpy(np.array(img_h)).to(device).float().unsqueeze(0)
        img_w = torch.from_numpy(np.array(img_w)).to(device).float().unsqueeze(0)
        focal_length = torch.from_numpy(np.array(focal)).to(device).float().unsqueeze(0)

        cx, cy, b = center[:, 0], center[:, 1], scale * 200
        bbox_info = torch.stack([cx - img_w / 2., cy - img_h / 2., b], dim=-1)
        # The constants below are used for normalization, and calculated from H36M data.
        # It should be fine if you use the plain Equation (5) in the paper.
        bbox_info[:, :2] = bbox_info[:, :2] / focal_length.unsqueeze(-1) * 2.8  # [-1, 1]
        bbox_info[:, 2] = (bbox_info[:, 2] - 0.24 * focal_length) / (0.06 * focal_length)  # [-1, 1]

        with torch.no_grad():
            pred_rotmat, pred_betas, pred_cam_crop = cliff_model(norm_img, bbox_info)

        # convert the camera parameters from the crop camera to the full camera
        full_img_shape = torch.stack((img_h, img_w), dim=-1)
        pred_cam_full = cam_crop2full(pred_cam_crop, center, scale, full_img_shape, focal_length)

        pred_output = smpl_model(betas=pred_betas,
                                 body_pose=pred_rotmat[:, 1:],
                                 global_orient=pred_rotmat[:, [0]],
                                 pose2rot=False,
                                 transl=pred_cam_full)
        skin_verts = pred_output.vertices.detach()
        skin_faces = torch.from_numpy(smpl_model.faces.astype(np.int32)).to(device)

        rot_pad = torch.tensor([0, 0, 1], dtype=torch.float32, device=device).view(1, 3, 1)
        rot_pad = rot_pad.expand(pred_rotmat.shape[0] * 24, -1, -1)
        rotmat = torch.cat((pred_rotmat.view(-1, 3, 3), rot_pad), dim=-1)
        pose = tgm.rotation_matrix_to_angle_axis(rotmat).contiguous().view(-1, 72)  # N*72
        pose = pose.detach().cpu().numpy().reshape(24, 3)
        theta = pose.copy()
        index = [0, 1, 4, 7, 10, 3, 6, 9, 12, 15, 13, 16, 18, 20, 22, 14, 17, 19, 21, 23, 2, 5, 8, 11]
        for i in range(pose.shape[0]):
            pose[i] = theta[index[i]]

        beta = pred_betas.detach().cpu().numpy().reshape(10,)

        skel_model.set_params(pose=pose, beta=beta)
        skel_verts = torch.from_numpy(skel_model.verts).float().to(device).unsqueeze(0)
        skel_faces = torch.from_numpy(skel_model.faces).to(device)

        artery_model.set_params(pose=pose, beta=beta)
        artery_verts = torch.from_numpy(artery_model.verts).float().to(device).unsqueeze(0)
        artery_faces = torch.from_numpy(artery_model.faces).to(device)

        color_batch, _ = render_mesh(
            vertices=skel_verts, faces=skel_faces,
            translation=pred_cam_full,
            focal_length=focal, height=image.shape[0], width=image.shape[1], red=False)

        valid_mask_batch = (color_batch[:, :, :, [-1]] > 0)
        image_vis_batch = color_batch[:, :, :, :3] * valid_mask_batch
        image_vis_batch = (image_vis_batch * 255).cpu().numpy()
        color = image_vis_batch[0]
        valid_mask = valid_mask_batch[0].cpu().numpy()
        input_img = image
        alpha = 0.9
        image_vis = alpha * color[:, :, :3] * valid_mask + (
                1 - alpha) * input_img * valid_mask + (1 - valid_mask) * input_img

        image_vis = image_vis.astype(np.uint8)

        color_batch, artery_pixel_coords = render_mesh(
            vertices=artery_verts, faces=artery_faces,
            translation=pred_cam_full,
            focal_length=focal, height=image.shape[0], width=image.shape[1])

        valid_mask_batch = (color_batch[:, :, :, [-1]] > 0)
        image_vis_batch = color_batch[:, :, :, :3] * valid_mask_batch
        image_vis_batch = (image_vis_batch * 255).cpu().numpy()

        color = image_vis_batch[0]
        valid_mask = valid_mask_batch[0].cpu().numpy()
        input_img = image_vis
        alpha = 0.9
        image_vis = alpha * color[:, :, :3] * valid_mask + (
                1 - alpha) * input_img * valid_mask + (1 - valid_mask) * input_img

        image_vis = image_vis.astype(np.uint8)
        image_vis = cv2.cvtColor(image_vis, cv2.COLOR_RGB2BGR)

        artery_pixel_coords = artery_pixel_coords.cpu().numpy()

        # 像素坐标转换为相机坐标
        artery_camera_coords = kpt_2d_to_3d(artery_pixel_coords, np_depth, intr, depth_scale)
        # 求初始点法向量
        x_range = np.arange(artery_pixel_coords[0, 0] - 2, artery_pixel_coords[0, 0] + 2)
        y_range = np.arange(artery_pixel_coords[0, 1] - 2, artery_pixel_coords[0, 1] + 2)
        xx, yy = np.meshgrid(x_range, y_range)
        neighborhood = np.column_stack((xx.ravel(), yy.ravel()))
        neighborhood = kpt_2d_to_3d(neighborhood, np_depth, intr, depth_scale)
        # 拟合平面
        centroid = np.mean(neighborhood, axis=0)
        centered_points = neighborhood - centroid
        _, _, V = svd(centered_points)
        normal_vector = V[-1]

        ### Add by Guanglin to prevent the normal vector opposite
        if normal_vector[2] > 0:
            normal_vector = -normal_vector

        # 绘制每个点
        pixel_coords = artery_pixel_coords.astype(int)
        for i in range(pixel_coords.shape[0]):
            x, y = pixel_coords[i]
            cv2.circle(image, (x, y), radius=3, color=(255, 0, 0), thickness=-1)
        image_vis = cv2.resize(image_vis,(800,600))
        cv2.imshow("result", image_vis)
        # cv2.moveWindow("result", 100, 2200)

        # cv2.imshow("depth Stream", np_depth)

        img_show_pcd = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        img_pcd_part = img_show_pcd#[0:1080,400:800]
        img_pcd_part = cv2.resize(img_pcd_part,(800,600))
        cv2.imshow("color stream", img_pcd_part)

        # cv2.moveWindow("color stream", 1500, 2200)
        
        key = cv2.waitKey(1)
        if key & 0xFF == ord('s') or key == 27:
            # np.savetxt('artery_path.txt', artery_camera_coords, fmt='%.6f')
            with open("artery_path.txt", "w") as f:
                data_save = "{:.3f} {:.3f} {:.3f}\n".format(normal_vector[0],normal_vector[1],normal_vector[2])
                f.write(data_save)
                np.savetxt(f, artery_camera_coords, fmt='%.6f')