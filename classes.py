import numpy as np
import cv2
import os
import glob

class StereoPair:
    def __init__(self, cam1, cam2):
        self.cameras = [cam1, cam2]
        self.stereo_images = [[], []] # 2xN list, N = number of stereo-matching images
        self.stereo_image_names = [] # 1xN list with image name strings (without extension)

    def load_checkboards_of_indices(self, indices):
        self.stereo_images = [[], []]

        for i in indices:
            image_name = self.stereo_image_names[i]
            for cam_idx, _ in enumerate(self.cameras):
                folder_dir = Params.root_path + self.cameras[cam_idx].name
                chb = Checkboard(image_name)
                chb.load_from_textfile(folder_dir + "\\corners\\" + image_name[0:5] + '.txt')
                img = Image(image_name[0:5] + '.pgm', folder_dir)
                img.checkboard = chb
                self.stereo_images[cam_idx].append(img)


class Camera:
    #
    # the focal lengths (fx, fy) are expressed in pixel units.
    # to fix fx/fy ratio, use CV_CALIB_FIX_ASPECT_RATIO. (ideally, fx == fy)

    # a principal point (cx, cy) is usually at the image center.

    #dim = np.array([600, 880]) # w, h [mm]
    # dim = np.array([300, 300])  # w, h [mm]
    sensor_size = (22, 11.88) # w, h [mm]
    image_res = (4000, 2160)

    def __init__(self, idx, name, avb):
        self.index = idx
        self.is_available = avb
        self.images = []
        self.name = name
        self.render = True

        # w.r.t. global coordinates system
        self.SE3 = None

        # ret: the mean reprojection error (it should be as close to zero as possible);
        # mtx: the matrix of intrisic parameters;
        # dist: the distortion parameters: 3 in radial distortion, 2 in tangential distortion;
        self.rms_err = -1.0
        self.M = None
        self.d = None
        self.focal_lengths = (0, 0)

        # 3d-2d projection pair (frame x points x 2d)
        self.measured_2d_points = []

    def set_render(self, boolv):
        self.render = boolv
    def get_render(self):
        return self.render

    def get_image(self, img_idx):
        for image in self.images:
            if image.index == img_idx:
                return image
        return None


    def set_calibration_data(self, ret, mtx, dist):
        self.rms_err = ret
        self.set_M(mtx)
        self.set_d(dist)

    def set_M(self, M_):
        self.M = M_
        self.focal_lengths = (self.M[0, 0] * self.sensor_size[0] / self.image_res[0], self.M[1, 1] * self.sensor_size[1] / self.image_res[1])

    def set_d(self, d_):
        self.d = d_
    def set_SE3(self, SE3_):
        self.SE3 = SE3_

    def introduce_yourself(self):
        print("Camera[{}] ({})".format(self.index, self.name))

    def load_image(self, image_name, image_dir):
        img = Image(image_name, image_dir)
        self.images.append(img)
        return img
    def unproject_2d_to_3d(self, uv):
        E = np.linalg.inv(self.SE3)[0:3, :]
        K = self.M
        ray_start = self.SE3[0:3, 3]
        uv3 = np.array([uv[0], uv[1], 1]).astype('float32')
        d_ray = np.linalg.inv(K.dot(E[0:3, 0:3])).dot(uv3)
        return ray_start, d_ray

class Image:
    pixel_w = 4000
    pixel_h = 2160

    def __init__(self, name, directory):
        # name includes file extension
        self.index = int(name.split('.')[0])
        self.name = name
        self.dir = directory
        self.checkboard = Checkboard(name)
        self.data = None
        self.shape = None
        self.data_init = False

    def load_image_data(self):
        os.chdir(self.dir)
        self.data = cv2.imread(self.name, cv2.IMREAD_UNCHANGED)
        if self.data is None:
            print('ERROR: image file ({}\{}) is None.'.format(self.dir, self.name))
        else:
            self.shape = self.data.shape
            self.data_init = True
        return self.data

    def find_checkboard(self, use_preprocessed_chb):
        if use_preprocessed_chb:
            chb_name = os.path.splitext(self.name)[0]
            chb_dir = self.dir + '\\corners\\' + chb_name + '.txt'
            found = self.checkboard.load_from_textfile(chb_dir)
            corners = self.checkboard.corners
        else:
            if self.data is None:
                self.load_image_data()

            print('  finding checkboard... ', end='')
            found, corners = cv2.findChessboardCorners(self.data, Checkboard.corners_dim)
            if found:
                print('found')
                self.checkboard = Checkboard(self.name)
            else:
                print('not found')
                self.checkboard = None

        return found, corners

class Checkboard:
    # checkboard: number of corners (# at the longer side, # at the shorter side)
    corners_dim = (11, 8)
    square_size = 60
    objp = None
    dim = np.array([800, 600, 10]) # w, h, d [m]

    def __init__(self, name):
        self.corners = np.empty([Checkboard.corners_dim[0]*Checkboard.corners_dim[1], 1, 2], dtype=np.float32)
        self.name = name
        self.SE3 = np.identity(4)

        # create corner positions w.r.t world coordinate
        Checkboard.objp = np.zeros((Checkboard.corners_dim[0] * Checkboard.corners_dim[1], 3), np.float32)

        # original coordinates of checkboard (top-left = [0, 0], bottom-right = [8, 11])
        # Checkboard.objp[:, :2] = Checkboard.square_size * np.mgrid[0:Checkboard.corners_dim[0], 0:Checkboard.corners_dim[1]].T.reshape(-1, 2)

        # changed coordinates of checkboard (top-left = [11, 0], bottom-right = [8, 0])
        for x in range(Checkboard.corners_dim[0]):
            for y in range(Checkboard.corners_dim[1]):
                i = y * Checkboard.corners_dim[0] + x
                Checkboard.objp[i, 0] = -x * Checkboard.square_size
                Checkboard.objp[i, 1] = y * Checkboard.square_size

    def load_from_textfile(self, chb_dir):
        # fetch .txt checkboard data
        success = False
        with open(chb_dir, 'r') as file:
            file.readline() # skip the first line

            cnt = 0
            for r in range(Checkboard.corners_dim[0]):
                for c in range(Checkboard.corners_dim[1]):
                    line = file.readline()
                    p = line.split()
                    self.corners[cnt, 0, 0] = p[0]
                    self.corners[cnt, 0, 1] = p[1]
                    cnt += 1
            file.close()
            success = True
        return success


class Params:
    # image directory
    # img_path = r'C:\Users\joont\Desktop\ImgSeqData'
    root_path = r'G:\My Drive\Converted'

    visualize_single = False
    visualize_stereo = False

    resize_scale = 0.15
    # set true to use camera at that index
    # camera_pairs = [[5, 6]]
    camera_pairs = [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6]]
    images_count_stereo = 5

    # for intrinsic calibration
    cameras = [Camera(0, r'\A001_05290330_C001', False), Camera(1, r'\B001_07072140_C001', False),
               Camera(2, r'\C001_06070320_C001', False), Camera(3, r'\D001_05061515_C001', False),
               Camera(4, r'\E001_04210958_C001', False), Camera(5, r'\F001_05151133_C001', True),
               Camera(6, r'\G001_05052127_C001', True), Camera(7, r'\H001_05191504_C001', False)]
    imgs_to_use_per_cam = 5
    chb_idx_mins = {'0': 4900, '1': 4900, '2': 4600, '3': 4200, '4': 4200, '5': 580, '6': 150, '7': 150}
    chb_idx_maxs = {'0': -1, '1': -1, '2': -1, '3': -1, '4': -1, '5': -1, '6': -1, '7': -1}

    xml_dir_export_intrinsics = r"C:\Users\joont\OneDrive\HJ\PhD\PycharmProjects\MultiViewCalib\xml\intrinsics"
    xml_dir_calib = r"C:\Users\joont\OneDrive\HJ\PhD\PycharmProjects\MultiViewCalib\GoogleDriveCopy\mocap\2019_08_09_AllPoseCapture\CalibGlobal"
    matched_img_pair_dir = r'C:\Users\joont\OneDrive\HJ\PhD\PycharmProjects\MultiViewCalib\matched_image_pairs\\'