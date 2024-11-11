
from cheetahgym.utils.heightmaps import RandomizedGapGenerator
import numpy as np
import cv2

class HeightmapSensor:
    def __init__(self, terrain_cfg_file):

        #input(("INITIALIZE", terrain_cfg_file))

        self.heightmap_array = None
        self.use_fixed_terrain = False
        self.hmap_cfg = {}

        # initialize heightmap generator 
        self.hmap_generator = RandomizedGapGenerator()
        print(terrain_cfg_file)
        self.set_terrain_parameters_from_json(terrain_cfg_file)

    def set_terrain_parameters_from_json(self, terrain_cfg_file):
        self.terrain_cfg_file = terrain_cfg_file
        try:
            self.hmap_generator.load_cfg(self.terrain_cfg_file)
        except FileNotFoundError:
            self.hmap_generator.load_cfg("/workspace/cheetah-gym/cheetahgym/" + self.terrain_cfg_file[1:])
        self.hmap_cfg = vars(self.hmap_generator.cfg)

    def set_terrain_parameters_from_dict(self, terrain_parameters):
        for key in terrain_parameters.keys():
            self.hmap_cfg[key] = terrain_parameters[key]
            self.hmap_generator.set_cfg(self.hmap_cfg)

    def set_fixed_terrain(self, gap_infos):
        self.use_fixed_terrain = True
        self.heightmap_array = np.zeros((self.hmap_cfg["hmap_width_px"],self.hmap_cfg["hmap_length_px"]))
        for gap_info in gap_infos:
            gap_start_px = int(gap_info["gap_start_cm"] / (100 * self.hmap_cfg["resolution"]) + 1.0 / self.hmap_cfg["resolution"])
            gap_width_px = int(gap_info["gap_width_cm"] / (100 * self.hmap_cfg["resolution"]))
            print(gap_start_px, gap_width_px)
            self.heightmap_array[:, gap_start_px:(gap_start_px + gap_width_px)] = -0.2

        return self.heightmap_array


    def set_fixed_terrain_array(self, heightmap):
        self.body_pos = self.hmap_cfg["origin_x_loc"], self.hmap_cfg["origin_y_loc"]
        print(self.body_pos)
         
        self.use_fixed_terrain = True
        self.hmap_cfg["hmap_width_px"] = heightmap.shape[0]
        self.hmap_cfg["hmap_length_px"] = heightmap.shape[1]
        self.heightmap_array = heightmap

        print(self.body_pos)
    
        return self.heightmap_array

    def load_new_heightmap(self):
        if self.use_fixed_terrain:
            return self.heightmap_array

        self.heightmap_array = self.hmap_generator.get_hmap()
        self.body_pos = self.hmap_cfg["origin_x_loc"], self.hmap_cfg["origin_y_loc"]
        
        self.update_cumulative_gap_counts()

        return self.heightmap_array

    def get_heightmap_as_array(self):
        return self.heightmap_array

    def convert_abs_pos_to_hmap_pixel(self, x, y):
        pos = np.array([x, y])
        px, py = self.hmap_cfg["hmap_length_px"] / 2. - (self.hmap_cfg["origin_x_loc"] - x) / self.hmap_cfg["resolution"] , (y + self.hmap_cfg["origin_y_loc"]) / self.hmap_cfg["resolution"] + self.hmap_cfg["hmap_width_px"] / 2.
        return [px, py]

    def update_cumulative_gap_counts(self):
        # compute cumulative gap counts
        #print(self.heightmap_array.shape)
        self.cum_gap_counts = np.zeros(self.heightmap_array.shape[1])
        py = self.heightmap_array.shape[0] // 2
        for i in range(1, self.heightmap_array.shape[1]):
            self.cum_gap_counts[i] = self.cum_gap_counts[i-1]
            if self.heightmap_array[py, i] == 0 and self.heightmap_array[py, i-1] < 0: # back edge of gap
                self.cum_gap_counts[i] += 1

    def get_current_elevation(self, x,y):
        px, py = self.convert_abs_pos_to_hmap_pixel(x, y)
        if int(px) < 0 or int(px) >= self.heightmap_array.shape[1] or int(py) < 0 or int(py) >= self.heightmap_array.shape[0]:
            return 0
        else:
            return self.heightmap_array[int(py),int(px)]

    def get_num_gaps_before(self, x):
        px, _ = self.convert_abs_pos_to_hmap_pixel(x, 0)
        if 0 <= int(px) < self.cum_gap_counts.shape[0]:
            return self.cum_gap_counts[int(px)]
        elif int(px) < 0:
            return 0
        elif int(px) >= self.cum_gap_counts.shape[0]:
            return self.cum_gap_counts[self.cum_gap_counts.shape[0]-1]

        return -1

    def get_gap_state(self, pos, num_gaps=1):
        px, py = self.convert_abs_pos_to_hmap_pixel(pos[0], pos[1])

        if 1 <= int(px) <= self.heightmap_array.shape[1]:
            gap_info = np.zeros(num_gaps*2)
            py_nom = self.heightmap_array.shape[0] // 2
            for i in range(int(px - 0.5 * self.hmap_cfg["resolution"]), self.heightmap_array.shape[1]):
                #print(i)
                if self.heightmap_array[py_nom, i-1] == 0 and self.heightmap_array[py_nom, i] < 0: # front edge of gap
                    gap_info[num_gaps*2-2] = (i - px) * self.hmap_cfg["resolution"]
                elif gap_info[num_gaps*2-2] != 0 and self.heightmap_array[py_nom, i] == 0 and self.heightmap_array[py_nom, i-1] < 0: # front edge of gap
                    gap_info[num_gaps*2-1] = (i - px) * self.hmap_cfg["resolution"]
                    if num_gaps == 1: 
                        break
                    else:
                        num_gaps -= 1
            return gap_info

        else:
            return np.array([-1, -1])



    def get_heightmap_ob(self, robot_xyz, robot_rpy, x_shift, y_shift, im_height, im_width, im_x_resolution, im_y_resolution, cfg):
        
        # if vision is turned off
        '''
        if not cfg.use_vision:
            return np.zeros((im_height, im_width))
        '''

        x_width, y_width = im_width * im_x_resolution, im_height * im_y_resolution
        x_lim = [x_shift - x_width / 2,
                 x_shift + x_width / 2]
        y_lim = [y_shift - y_width / 2,
                 y_shift + y_width / 2]
        self.corners = np.array([[x_lim[0], y_lim[0]],
                                 [x_lim[1], y_lim[0]],
                                 [x_lim[0], y_lim[1]],
                                 [x_lim[1], y_lim[1]]]) / self.hmap_cfg["resolution"]
        xlimp = np.array([0, x_width]) / im_x_resolution
        ylimp = np.array([0, y_width]) / im_y_resolution
        self.target_point = np.array(
            [[xlimp[0], ylimp[0]],
             [xlimp[1], ylimp[0]],
             [xlimp[0], ylimp[1]],
             [xlimp[1], ylimp[1]]]).astype(np.float32)


        x_pos_world, y_pos_world = robot_xyz[0], robot_xyz[1]

        robot_yaw_angle = robot_rpy[2]
        body_ht = robot_xyz[2]

        R = np.array(
            [[np.cos(robot_yaw_angle), np.sin(robot_yaw_angle)], [-np.sin(robot_yaw_angle), np.cos(robot_yaw_angle)]])

        robot_loc = self.convert_abs_pos_to_hmap_pixel(x_pos_world, y_pos_world)
        origin_point = np.matmul(R, self.corners.T).T + robot_loc

        origin_point = origin_point.astype(np.float32)

        map_matrix = cv2.getPerspectiveTransform(origin_point, self.target_point)
        heightmap_ob = cv2.warpPerspective(self.heightmap_array, map_matrix, (im_width, im_height))

        ''' optional noise parameters
        if cfg.apply_motion_blur:
            # loading library
            # Specify the kernel size. 
            # The greater the size, the more the motion. 
            kernel_size_v = np.random.randint(3, 7)
            kernel_size_h = np.random.randint(3, 7)
            # Create the vertical kernel. 
            kernel_v = np.zeros((kernel_size_v, kernel_size_v))
            kernel_h = np.zeros((kernel_size_h, kernel_size_h))
            # Fill the middle row with ones. 
            kernel_v[:, int((kernel_size_v - 1) / 2)] = np.ones(kernel_size_v)
            kernel_h[int((kernel_size_h - 1) / 2), :] = np.ones(kernel_size_h)
            # Normalize. 
            kernel_v /= kernel_size_v
            kernel_h /= kernel_size_h
            # Apply the vertical kernel. 
            vertical_mb = cv2.filter2D(heightmap_ob, -1, kernel_v)
            # Apply the horizontal kernel. 
            horizontal_mb = cv2.filter2D(heightmap_ob, -1, kernel_h)

        if cfg.apply_heightmap_noise:
            unstructured_noise = np.random.rand(*heightmap_ob.shape) * 0.02 - 0.01

            heightmap_ob = heightmap_ob + unstructured_noise

        if cfg.dilation_px > 0:
            size = cfg.dilation_px  # constrols gap enlargement
            kernel = np.ones((size, size), np.uint8)
            heightmap_ob = cv2.dilate(heightmap_ob,kernel,iterations = 1)

        if cfg.erosion_px > 0:
            size = cfg.erosion_px  # constrols gap enlargement
            kernel = np.ones((size, size), np.uint8)
            heightmap_ob = cv2.erode(heightmap_ob,kernel,iterations = 1)

        heightmap_ob = heightmap_ob - body_ht  # - self.ground_height_est
        '''

        #if cfg.scale_heightmap:
        heightmap_ob = (heightmap_ob + 0.45) * 8

        return heightmap_ob
