import numpy as np
import json
#import raisimpy as raisim
import cv2
import os
import argparse

#RESOLUTION = 1/30 #1/100.
#HMAP_LENGTH = 1000
#HMAP_WIDTH = 100


# main heightmap generator classes

class FileReader:

    def __init__(self, dataset_size=100, destination="../img/dataset"):
        self.i = np.random.randint(dataset_size)
        self.dataset_size=dataset_size #np.random.randint(dataset_size)
        self.destination = destination
        self.sample = True

    def next_filename(self):
        if self.sample:
            self.i = np.random.randint(self.dataset_size)
        img_name = self.destination + "/hmap{}.png".format(self.i)
        #self.i = (self.i + 1) % self.dataset_size
        return img_name

    def _filename(self):
        return self.destination + "/hmap{}.png".format(self.i)
    
    def fix_heightmap_idx(self, idx):
        self.i = idx
        self.sample = False
   
class MapGenerator:

    def __init__(self, cfg=None):
        self.cfg = cfg

    def set_cfg(self, cfg):
        self.cfg = argparse.Namespace(**cfg)

    def load_cfg(self, cfg_file):
        #parser = argparse.ArgumentParser()
        print(f"load cfg {cfg_file}")
        try:
            with open(cfg_file, 'rt') as f:
                t_args = argparse.Namespace()
                t_args.__dict__.update(json.load(f))
                #print(t_args)
                self.cfg = t_args

        except FileNotFoundError:
            with open("/workspace/jumping-from-pixels/cheetahgym/" + cfg_file[1:]) as f:
                t_args = argparse.Namespace()
                t_args.__dict__.update(json.load(f))
                #print(t_args)
                self.cfg = t_args


        if not hasattr(self.cfg, "platform_center_noise"):
            self.cfg.platform_center_noise = 0.0

    def get_hmap(self):
        raise NotImplementedError

'''
class RandomizedGapGenerator(MapGenerator):
    def __init__(self, cfg=None):
        super().__init__(cfg=cfg)

    def get_hmap(self):
        param_generators = {"gap": generate_gap_params}#, "stair": generate_stair_params}
        #print(self.cfg)
        if self.cfg.linear:
            transforms = {"gap": add_gap}#, "stair": add_stairs}
        else:
            transforms = {"gap": add_gap_uniform_random, "stair": add_stairs}
        feature_list = []
        hmap = np.zeros((self.cfg.hmap_width_px, self.cfg.hmap_length_px))

        idx = int((1.0 + self.cfg.first_gap_loc + self.cfg.first_gap_loc_range * np.random.random()) / self.cfg.resolution)
        start_height = 0.0
        if self.cfg.linear:
            while idx < self.cfg.hmap_length_px:
                feature_name = np.random.choice(list(transforms.keys()))
                params = param_generators[feature_name](start = idx, cfg=self.cfg)
                hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
                idx = int(params[4] / self.cfg.resolution)
                #print(params[4], params[1] * 0.01, int((params[4] / self.cfg.resolution + params[1] * 0.01 / self.cfg.resolution)))
            
            if self.cfg.use_narrow_platforms:
                right_gap_loc = self.cfg.hmap_width_px/2 * self.cfg.resolution + self.cfg.platform_width/2 + (np.random.random() - 0.5) * self.cfg.platform_width_noise
                right_gap_width = self.cfg.platform_side_width + (np.random.random() - 0.5) * self.cfg.platform_side_width_noise * 2
                hmap[int((right_gap_loc) / self.cfg.resolution):int((right_gap_loc + right_gap_width) / self.cfg.resolution), :] = -1 * ( np.random.rand() * .05 + .1 )
                left_gap_loc = self.cfg.hmap_width_px/2  * self.cfg.resolution - self.cfg.platform_width/2 + (np.random.random() - 0.5) * self.cfg.platform_width_noise
                left_gap_width = self.cfg.platform_side_width + (np.random.random() - 0.5) * self.cfg.platform_side_width_noise * 2
                hmap[int((left_gap_loc-left_gap_width) / self.cfg.resolution):int((left_gap_loc) / self.cfg.resolution), :] = -1 * ( np.random.rand() * .05 + .1 )
                #print(right_gap_loc, left_gap_loc, right_gap_width, left_gap_width)
            if self.cfg.additive_noise_magnitude > 0:
                hmap = hmap + np.random.random(hmap.shape) * self.cfg.additive_noise_magnitude

            return hmap
        else:
            for i in range(self.cfg.num_gaps):
                feature_name = np.random.choice(list(transforms.keys()))
                params = param_generators[feature_name](start = idx, cfg=self.cfg)
                hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
                idx += int((params[4] / self.cfg.resolution + params[1] * 0.01 / self.cfg.resolution))
            return hmap
'''
class RandomizedGapGenerator(MapGenerator):
    def __init__(self, cfg=None):
        super().__init__(cfg=cfg)

    def get_hmap(self):
        param_generators, transforms = {}, {}
        if "gap" in self.cfg.terrain_features_list: 
            param_generators["gap"] = generate_gap_params
            transforms["gap"] = add_gap
        if "stairs" in self.cfg.terrain_features_list:
            param_generators["stairs"] = generate_stair_params
            transforms["stairs"] = add_stairs
        #print(self.cfg)

        #if self.cfg.linear:
        #    transforms = {"gap": add_gap}
        #else:
        #    transforms = {"gap": add_gap_uniform_random}
        feature_list = []
        hmap = np.zeros((self.cfg.hmap_width_px, self.cfg.hmap_length_px))
        
        start_height = 0.0

        if isinstance(self.cfg.first_gap_loc, float):
            self.cfg.first_gap_loc = [self.cfg.first_gap_loc]

        # read list of initial gaps
        for gl in self.cfg.first_gap_loc: 
            idx = int((1.0 + gl + self.cfg.first_gap_loc_range * np.random.random()) / self.cfg.resolution)
            feature_name = np.random.choice(list(transforms.keys()))
            params = param_generators[feature_name](start = idx, cfg=self.cfg)
            hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
        idx = int(params[4] / self.cfg.resolution)

        # begin randomized generation        
        if self.cfg.linear:
            while idx < self.cfg.hmap_length_px:
                feature_name = np.random.choice(list(transforms.keys()))
                params = param_generators[feature_name](start = idx, cfg=self.cfg)
                hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
                idx = int(params[4] / self.cfg.resolution)
                #print(params[4], params[1] * 0.01, int((params[4] / self.cfg.resolution + params[1] * 0.01 / self.cfg.resolution)))
            
            if self.cfg.use_narrow_platforms:
                platform_center_offset = self.cfg.platform_center_noise * (np.random.random() * 2 - 1)
                #input(f"PCO {platform_center_offset}")
                right_gap_loc = platform_center_offset + self.cfg.hmap_width_px/2 * self.cfg.resolution + self.cfg.platform_width/2 + (np.random.random() - 0.5) * self.cfg.platform_width_noise
                right_gap_width = self.cfg.platform_side_width + (np.random.random() - 0.5) * self.cfg.platform_side_width_noise * 2
                hmap[int((right_gap_loc) / self.cfg.resolution):int((right_gap_loc + right_gap_width) / self.cfg.resolution), :] = -1 * ( np.random.rand() * .05 + .1 )
                left_gap_loc = platform_center_offset + self.cfg.hmap_width_px/2  * self.cfg.resolution - self.cfg.platform_width/2 + (np.random.random() - 0.5) * self.cfg.platform_width_noise
                left_gap_width = self.cfg.platform_side_width + (np.random.random() - 0.5) * self.cfg.platform_side_width_noise * 2
                hmap[int((left_gap_loc-left_gap_width) / self.cfg.resolution):int((left_gap_loc) / self.cfg.resolution), :] = -1 * ( np.random.rand() * .05 + .1 )
                #print(right_gap_loc, left_gap_loc, right_gap_width, left_gap_width)
            if self.cfg.additive_noise_magnitude > 0:
                hmap = hmap + np.random.random(hmap.shape) * self.cfg.additive_noise_magnitude

            return hmap
        else:
            for i in range(self.cfg.num_gaps):
                feature_name = np.random.choice(list(transforms.keys()))
                params = param_generators[feature_name](start = idx, cfg=self.cfg)
                hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
                idx += int((params[4] / self.cfg.resolution + params[1] * 0.01 / self.cfg.resolution))
            return hmap


class StaircaseGenerator(MapGenerator):
    def __init__(self, cfg=None):
        super().__init__(cfg=cfg)

    def get_hmap(self):
        
        param_generators = {"stair": generate_stair_params}
        #print(self.cfg)
        if self.cfg.linear:
            transforms = {"stair": add_stairs}
        else:
            transforms = {"stair": add_stairs}
        
        feature_list = []
        hmap = np.zeros((self.cfg.hmap_width_px, self.cfg.hmap_length_px))
        #max_step_width_local = np.random.random() * (self.cfg.max_step_width - self.cfg.min_step_width) + self.cfg.min_step_width

        idx = int((1.0 + self.cfg.first_stairs_loc + self.cfg.first_stairs_loc_range * np.random.random()) / self.cfg.resolution)
        start_height = 0.0
        if self.cfg.linear:
            while idx < self.cfg.hmap_length_px:
                feature_name = np.random.choice(list(transforms.keys()))
                params = param_generators[feature_name](start=idx, cfg=self.cfg)
                hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
                idx = int(params[4] / self.cfg.resolution)
                #print(params[4], params[1] * 0.01, int((params[4] / self.cfg.resolution + params[1] * 0.01 / self.cfg.resolution)))
            
            if self.cfg.use_narrow_platforms:
                right_gap_loc = self.cfg.hmap_width_px/2 * self.cfg.resolution + self.cfg.platform_width/2 + (np.random.random() - 0.5) * self.cfg.platform_width_noise
                right_gap_width = self.cfg.platform_side_width + (np.random.random() - 0.5) * self.cfg.platform_side_width_noise * 2
                hmap[int((right_gap_loc) / self.cfg.resolution):int((right_gap_loc + right_gap_width) / self.cfg.resolution), :] = -1 * ( np.random.rand() * .05 + .1 )
                left_gap_loc = self.cfg.hmap_width_px/2  * self.cfg.resolution - self.cfg.platform_width/2 + (np.random.random() - 0.5) * self.cfg.platform_width_noise
                left_gap_width = self.cfg.platform_side_width + (np.random.random() - 0.5) * self.cfg.platform_side_width_noise * 2
                hmap[int((left_gap_loc-left_gap_width) / self.cfg.resolution):int((left_gap_loc) / self.cfg.resolution), :] = -1 * ( np.random.rand() * .05 + .1 )
                #print(right_gap_loc, left_gap_loc, right_gap_width, left_gap_width)
            if self.cfg.additive_noise_magnitude > 0:
                hmap = hmap + np.random.random(hmap.shape) * self.cfg.additive_noise_magnitude

            return hmap
        else:
            for i in range(self.cfg.num_gaps):
                feature_name = np.random.choice(list(transforms.keys()))
                params = param_generators[feature_name](start = idx, cfg=self.cfg)
                hmap, start_height = transforms[feature_name](hmap, params=params, cfg=self.cfg, start_height=start_height)
                idx += int((params[4] / self.cfg.resolution + params[1] * 0.01 / self.cfg.resolution))
            return hmap


# define features
def generate_flat_params(start, end):
    return [start, end]

def add_flat(hmap, params):
    start, end = params
    hmap[:, start:] = hmap[0, start]
    return hmap
    
def generate_stair_params(start, cfg):
    #start_height = hmap[0, start]
    stair_width = np.random.rand() * (cfg.max_step_width - cfg.min_step_width) * np.random.rand() + cfg.min_step_width # np.random.rand() * 0.14 - 0.07
    stair_height = np.random.rand() * (cfg.max_step_height - cfg.min_step_height) * np.random.rand() + cfg.min_step_height
    num_stairs = np.random.randint(cfg.min_num_steps, cfg.max_num_steps+1)
    next_stair_start = stair_width * num_stairs + (cfg.max_stairs_spacing - cfg.min_stairs_spacing) * np.random.rand() + cfg.min_stairs_spacing
    return [start, stair_height, stair_width, num_stairs, next_stair_start]

def add_stairs(hmap, params, cfg, start_height):
    start, stair_height, stair_width, num_stairs, next_stair_start = params
    #start_height = hmap[0, start]
    for i in range(num_stairs):
        hmap[:, (start + i * int(stair_width * 0.01 / cfg.resolution)):] = start_height + stair_height * (i+1)
    return hmap, start_height + num_stairs * stair_height

def generate_noise_params(start, end):
    noise_magnitude = np.random.rand() * 0.10
    noise_width = np.random.randint(int(0.25/RESOLUTION), end-start-1)
    noise_loc = np.random.randint(start, end-noise_width-1)
    return [start, end, noise_magnitude, noise_width, noise_loc]

def add_noise(hmap, params):
    start, end, noise_magnitude, noise_width, noise_loc = params
    start_height = hmap[0, start]
    hmap[:, noise_loc:(noise_loc+noise_width)] = start_height + np.random.rand(hmap.shape[0], noise_width) * noise_magnitude
    hmap[:, (noise_loc+noise_width):] = start_height
    return hmap

def generate_slope_params(start, end):
    slope_magnitude = np.random.rand() * 0.2 - 0.1
    slope_width = np.random.randint(int(0.25/RESOLUTION), end-start-1)
    slope_loc = np.random.randint(start, end-slope_width-1)
    return [start, end, slope_magnitude, slope_width, slope_loc]

def add_slope(hmap, params):
    start, end, slope_magnitude, slope_width, slope_loc = params
    start_height = hmap[0, start]
    slope_end_height = start_height + slope_magnitude * slope_width / int(1./RESOLUTION)
    hmap[:, slope_loc:(slope_loc+slope_width)] = np.linspace(tuple(start_height for i in range(hmap.shape[0])),
                                                             tuple(slope_end_height for i in range(hmap.shape[0])),
                                                             slope_width).T
    hmap[:, (slope_loc+slope_width):] = slope_end_height
    return hmap

def generate_gap_params(start, cfg):
    gap_width = np.random.rand() * (cfg.max_gap_width - cfg.min_gap_width) + cfg.min_gap_width #(1, 7) # (2, 6)
    gap_depth = np.random.rand() * .2 + .2
    gap_height_difference = 0. #np.random.rand() * 0.10 - 0.05
    next_gap_start = np.random.rand() * (cfg.max_gap_dist - cfg.min_gap_dist) + cfg.min_gap_dist + start * cfg.resolution + gap_width * 0.01
    gap_slant = (np.random.rand() * 2 - 1) * cfg.max_gap_slant
    gap_stagger = np.random.randint(cfg.stagger_size_min, cfg.stagger_size_max)
    if np.random.random() > 0.5: gap_stagger *= -1
    return [start, gap_width, gap_depth, gap_height_difference, next_gap_start, gap_slant, gap_stagger]

def add_gap(hmap, params, cfg, start_height):
    start, gap_width, gap_depth, gap_height_difference, next_gap_start, gap_slant, gap_stagger = params
    #print(start, gap_width, gap_depth, gap_height_difference, gap_loc)
    #start_height = hmap[0, start]
    hmap_width_px = hmap.shape[0]
    stagger_interval = hmap_width_px // (cfg.num_stagger + 1) 
    for i in range(hmap_width_px):
        #print(f"diff: {(i - (hmap_width_px / 2.)) * gap_slant}")
        #close_edge = int((gap_loc / cfg.resolution + (i - (hmap_width_px / 2.)) * gap_slant))
        close_edge = int((i - (hmap_width_px / 2.)) * gap_slant)
        if cfg.num_stagger > 0 and (i // stagger_interval) % 2 == 0: # stagger
            close_edge = close_edge + gap_stagger

        #print(close_edge, close_edge+int(gap_width * 0.01 / cfg.resolution))
        
        hmap[i, start+close_edge:start+close_edge+int(gap_width * 0.01 / cfg.resolution)] = start_height - gap_depth

        #print(f'start: {start}, close_edge: {close_edge}, gap_slant: {gap_slant}, gap_stagger: {gap_stagger}')
        #hmap[i, start+close_edge+int(gap_width * 0.01 / cfg.resolution):] = start_height + gap_height_difference

        #hmap[:, start+int(gap_loc / cfg.resolution):] = start_height-gap_depth
        #hmap[:, start+int(gap_loc / cfg.resolution) + int(gap_width * 0.01 / cfg.resolution):] = start_height + gap_height_difference
    #print(int(gap_width * 0.01 / cfg.resolution))
    return hmap, start_height + gap_height_difference

def add_gap_uniform_random(hmap, params, cfg, start_height):
    start, gap_width, gap_depth, gap_height_difference, gap_loc, gap_slant, gap_stagger = params
    #print(start, gap_width, gap_depth, gap_height_difference, gap_loc)
    #start_height = hmap[0, start]
    start_x = np.random.randint(0, hmap.shape[0])
    start_y = np.random.randint(0, hmap.shape[1])

    hmap_width_px = hmap.shape[0]
    stagger_interval = hmap_width_px // (cfg.num_stagger + 1) 
    for i in range(hmap_width_px):

        #print(f"diff: {(i - (hmap_width_px / 2.)) * gap_slant}")
        close_edge = int((start_y + (i - start_x) * gap_slant))

        #print((close_edge, start_y), (i, start_x), ((close_edge - start_y)**2 + (i - start_x)**2)**0.5 , cfg.gap_length / cfg.resolution / 2.)
        if ((close_edge - start_y)**2 + (i - start_x)**2)**0.5 > cfg.gap_length / cfg.resolution / 2.: continue

        if (i // stagger_interval) % 2 == 0: # stagger
            close_edge = close_edge + gap_stagger

        #print(close_edge, close_edge+int(gap_width * 0.01 / cfg.resolution))
        
        hmap[i, start_y+close_edge:start_y+close_edge+int(gap_width * 0.01 / cfg.resolution)] = start_height - gap_depth
        #hmap[i, start+close_edge+int(gap_width * 0.01 / cfg.resolution):] = start_height + gap_height_difference

        #hmap[:, start+int(gap_loc / cfg.resolution):] = start_height-gap_depth
        #hmap[:, start+int(gap_loc / cfg.resolution) + int(gap_width * 0.01 / cfg.resolution):] = start_height + gap_height_difference
    #print(int(gap_width * 0.01 / cfg.resolution))
    return hmap, start_height + gap_height_difference



def build_json_dataset(filename, size=100):
    get_hmap_params = randomized_composite_world_params()
    data = {}
    data['terrains'] = []
    for i in range(size):
        feature_list = get_hmap_params()

        data['terrains'].append({
            'id': i,
            'features': feature_list
            })

    
    #json.dumps(data, indent=4)

    with open(filename, 'w') as outfile:
        json.dump(data, outfile, indent=4)


class CompositeWorldReader:
    def __init__(self, filename):
        with open(filename, 'r') as json_file:
            self.data = json.load(json_file)
        self.idx = 0

    # hmap generation function
    def get_hmap(self):
        if(self.idx >= len(self.data["terrains"])):
            print("Reached end of terrains dataset! Starting again from beginning.")
            self.idx = 0
        terrain = self.data["terrains"][self.idx]
        hmap = np.zeros((120, 600))
        transforms = {"flat": add_flat, "stairs": add_stairs, "gap": add_gap, "noise": add_noise, "slope": add_slope}
        for feature in terrain['features']:
            name = feature["name"]
            params = feature["params"]
            hmap = transforms[name](hmap, params)    
        
        self.idx += 1
        return hmap
        #print(terrain)
        #hmap = np.zeros((120, 300))
        #transforms = [add_flat, add_stairs, add_gap, add_noise, add_slope]
        #for idx in range(100, 250, 30):
            # add a random feature here
        #    hmap, params = np.random.choice(transforms)(hmap, start=idx, end=idx+50)

def build_composite_dataset(size=100):
    get_hmap = randomized_composite_world()
    datas = []
    for i in range(size):
        datas += [get_hmap()]

    return datas

def write_image_dataset(hmap_generator, dataset_size=100, destination="../img/dataset", cfg=None):
    #from PIL import Image
    for i in range(dataset_size):
        depth = hmap_generator.get_hmap()
        sdepth = np.clip(np.flip(depth, axis=1) * cfg.scale + cfg.shift, 0, 2**16-1)
        img_name = destination + "/hmap{}.png".format(i)
        print(img_name)
        cv2.imwrite(img_name, sdepth.astype(np.uint16))
        if i % 200 == 0:
            print("Wrote {} terrains".format(i))

        #re_depth = cv2.imread(img_name, cv2.IMREAD_UNCHANGED)
        #sre_depth = (re_depth - shift) / scale
        #matplotlib.image.imsave("./img/heightmaps/hmap{}.png".format(i), data)
        #im = Image.fromarray(data)
        #im.save("./img/heightmaps/hmap{}.png".format(i))




def plot_heightmaps_mpl(datas):
    #dataset_size=10
    #datas = build_composite_dataset(size=dataset_size)
    fig = plt.figure()
    fig.tight_layout()
    for i in range(dataset_size):
        hmap = datas[i]
        x, y = np.meshgrid(range(hmap.shape[1]), range(hmap.shape[0]))
        print(x.shape, y.shape, hmap.shape)
        ax = fig.add_subplot(dataset_size//5, 5, i+1, projection='3d')
        ax.plot_surface(x, y, hmap)
        #plt.subplot(dataset_size // 5, 5, i+1)
        #plt.imshow(datas[i])

    plt.savefig("img/composite_environments.png")


def normalize(array):
    return np.asarray(array) / np.linalg.norm(array)

def setup_callback():
    vis = raisim.OgreVis.get()

    # light
    light = vis.get_light()
    light.set_diffuse_color(1, 1, 1)
    light.set_cast_shadows(True)
    light.set_direction(normalize([-3., -3., -0.5]))
    vis.set_camera_speed(300)

    # load textures
    vis.add_resource_directory(vis.get_resource_dir() + "/material/checkerboard")
    vis.load_material("checkerboard.material")

    # shadow setting
    manager = vis.get_scene_manager()
    manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
    manager.set_shadow_texture_settings(2048, 3)

    # scale related settings!! Please adapt it depending on your map size
    # beyond this distance, shadow disappears
    manager.set_shadow_far_distance(3)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.03, 0.2)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)

def visualize_heightmaps_raisim(datas):
    raisim.OgreVis.get().start_recording_video("video/heightmaps_display.mp4")
    
    world = raisim.World()
    world.set_time_step(0.02)

    vis = raisim.OgreVis.get()
    vis.set_world(world)
    vis.set_window_size(1800, 1200)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(2)
    #raisim.gui.manual_stepping = True

    vis.init_app()

    vis.set_desired_fps(10)

    world.set_erp(0., 0.)
  
    
    for hmap in datas:
        heights = hmap.flatten()
        ground = world.add_heightmap(x_samples=hmap.shape[1],
                                                    y_samples=hmap.shape[0],
                                                    x_scale=20.0,
                                                    y_scale=4.0,
                                                    x_center=0.0,
                                                    y_center=0.0,
                                                    heights=heights,
                                                    material="checkerboard_green")

        ground_graph = vis.create_graphical_object(ground, name="floor", material="checkerboard_green")
        vis.select(ground_graph[0], False)
        vis.get_camera_man().set_yaw_pitch_dist(3.14, -0.6, 20)
    
        for i in range(10):
            world.integrate()
            vis.render_one_frame()
        #print("rendering")
        vis.remove(ground)
        #vis.render_one_frame()

    raisim.OgreVis.get().stop_recording_video_and_save()
    raisim.OgreVis.get().close_app() 


if __name__ == "__main__":
    import matplotlib
    #matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    
    import raisimpy as raisim


    parser = argparse.ArgumentParser()
    parser.add_argument('--use_linear', dest='linear', action="store_true")
    parser.add_argument('--num_gaps', type=int, default=100) # for non-linear map
    parser.add_argument('--use_narrow_platforms', dest='use_narrow_platforms', action='store_true')
    parser.add_argument('--platform_width', type=float, default=0.5)
    parser.add_argument('--platform_width_noise', type=float, default=0.2)
    parser.add_argument('--platform_center_noise', type=float, default=0.0)
    parser.add_argument('--platform_side_width', type=float, default=0.3)
    parser.add_argument('--platform_side_width_noise', type=float, default=0.2)
    parser.add_argument('--gap_length', type=float, default=4)
    parser.add_argument('--train_size', type=int, default=10000)
    parser.add_argument('--test_size', type=int, default=1000)
    parser.add_argument('--scale', type=float, default=500.0)
    parser.add_argument('--shift', type=float, default=5000.0)
    parser.add_argument('--save_dir', type=str, default='./terrains/default')
    parser.add_argument('--resolution', type=float, default=1/30.)
    parser.add_argument('--hmap_length_px', type=int, default=1000)
    parser.add_argument('--hmap_width_px', type=int, default=100)
    parser.add_argument('--origin_x_loc', type=float, default=1.0)
    parser.add_argument('--origin_y_loc', type=float, default=0.0)
    parser.add_argument('--first_gap_loc', type=float, default=1.0, nargs="+")
    parser.add_argument('--first_gap_loc_range', type=float, default=0.0)
    parser.add_argument('--min_gap_width', type=int, default=5)
    parser.add_argument('--max_gap_width', type=int, default=16)
    parser.add_argument('--min_gap_dist', type=float, default=0.7)
    parser.add_argument('--max_gap_dist', type=float, default=1.5)
    parser.add_argument('--max_gap_slant', type=float, default=0.0)
    parser.add_argument('--num_stagger', type=int, default=0)
    parser.add_argument('--stagger_size_min', type=int, default=0)
    parser.add_argument('--stagger_size_max', type=int, default=1)
    parser.add_argument('--use_skewed_dist_prob', dest='skewed_dist_prob', action="store_true")
    parser.add_argument('--skew_high_prob', type=float, default=0.3)
    parser.add_argument('--skew_thresh', type=float, default=1.0)
    parser.add_argument('--legacy', dest='legacy', action='store_true')
    parser.add_argument('--additive_noise_magnitude', type=float, default=0.0)
    #parser.add_argument('--use_asymmetric_probs')

    cfg = parser.parse_args()

    os.makedirs(cfg.save_dir)
    train_dir = cfg.save_dir + "/train"
    test_dir = cfg.save_dir + "/test"
    os.mkdir(train_dir)
    os.mkdir(test_dir)

    train_size, test_size = 10000, 1000
    randomized_gap_world = RandomizedGapGenerator(cfg)

    write_image_dataset(hmap_generator=randomized_gap_world, dataset_size=cfg.train_size, destination = train_dir, cfg=cfg)
    write_image_dataset(hmap_generator=randomized_gap_world, dataset_size=cfg.test_size, destination = test_dir, cfg=cfg)
    # write json file with terrain info

    with open(cfg.save_dir + '/params.json', 'w') as json_file:
        json.dump(vars(cfg), json_file)

    #build_json_dataset(filename="terrains.txt", size=dataset_size)

    #cwr = CompositeWorldReader(filename="terrains.txt")
    #get_hmap = cwr.get_hmap

    #get_hmap = randomized_composite_world()
    #print(get_hmap())
    #print(get_hmap())
    #dataset_size=5
    #datas = build_composite_dataset(size=dataset_size)
    #visualize_heightmaps_raisim(datas)




'''
def randomized_stairs_world():
    def get_hmap():
        param_generators = {"stairs": generate_stair_params}
        transforms = {"stairs": add_stairs}
        feature_list = []
        hmap = np.zeros((HMAP_WIDTH, HMAP_LENGTH))
        for idx in range(int(2. / RESOLUTION), HMAP_LENGTH, int(2. / RESOLUTION)):
            feature_name = np.random.choice(list(transforms.keys()))
            params = param_generators[feature_name](start=idx, end=idx+2./RESOLUTION)
            hmap = transforms[feature_name](hmap, params=params)
        return hmap
    return get_hmap

def static_gap_world():
    def get_hmap():
        param_generators = {"gap": generate_gap_params}
        transforms = {"gap": add_gap}
        feature_list = []
        hmap = np.zeros((HMAP_WIDTH, HMAP_LENGTH))
        params = [(10, 3, -0.3), (17, 2, -0.4), (19, 5, -0.2), (5, 4, -0.2), (14, 5,-0.3), (7, 6, -0.4)]
        for i in range(len(params)):
            pos = 60 + 90 * i + params[i][0]
            hmap[:, pos:pos+params[i][1]] = params[i][2]
        return hmap
    return get_hmap

def flat_world():
    def get_hmap():
        return np.zeros((HMAP_WIDTH, HMAP_LENGTH))
    return get_hmap

def randomized_noise_world():
    def get_hmap():
        param_generators = {"noise": generate_noise_params}
        transforms = {"noise": add_noise}
        feature_list = []
        hmap = np.zeros((HMAP_WIDTH, HMAP_LENGTH))
        for idx in range(60, 1700, 40):
            feature_name = np.random.choice(list(transforms.keys()))
            params = param_generators[feature_name](start=idx, end=idx+30)
            hmap = transforms[feature_name](hmap, params=params)
        return hmap
    return get_hmap

def randomized_composite_world():
    def get_hmap():
        param_generators = {"flat": generate_flat_params, "stairs": generate_stair_params, "gap": generate_gap_params, "noise": generate_noise_params, "slope": generate_slope_params}
        transforms = {"flat": add_flat, "stairs": add_stairs, "gap": add_gap, "noise": add_noise, "slope": add_slope}
        feature_list = []
        hmap = np.zeros((HMAP_WIDTH, HMAP_LENGTH))
        for idx in range(int(2./RESOLUTION), HMAP_LENGTH, int(1./RESOLUTION)):
            feature_name = np.random.choice(list(transforms.keys()))
            params = param_generators[feature_name](start=idx, end=idx+30)
            hmap = transforms[feature_name](hmap, params=params)
        return hmap
    return get_hmap

def randomized_composite_world_params():
    def get_hmap_params():
        param_generators = {"flat": generate_flat_params, "stairs": generate_stair_params, "gap": generate_gap_params, "noise": generate_noise_params, "slope": generate_slope_params}
        #transforms = {"flat": add_flat, "stairs": add_stairs, "gap": add_gap, "noise": add_noise, "slope": add_slope}
        feature_list = []
        for idx in range(int(2./RESOLUTION), HMAP_LENGTH, int(1./RESOLUTION)):
            feature_name = np.random.choice(list(param_generators.keys()))
            params = param_generators[feature_name](start=idx, end=idx+int(1./RESOLUTION))
            #hmap = transforms[feature_name](hmap, params=params)
            feature_list += [{"name": feature_name, "params": params}]
        return feature_list
    return get_hmap_params

'''
