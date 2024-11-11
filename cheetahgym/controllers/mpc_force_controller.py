

class MPCForceController:
    def __init__(self, dt):
        self.dt = dt

    def initialize(self):
        pass

    def solve_forces(self, low_level_state, rot_w_b, wbc_level_cmd, mpc_table, iters_list, trajAll, foot_locations):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError