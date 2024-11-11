import numpy as np

class DynamicsParameters:
    def __init__(self, cfg=None):

        #self.cfg=cfg

        if cfg is not None:
            # robot dynamics
            self.joint_dampings = np.ones(12) * cfg.nominal_joint_damping
            self.joint_frictions = np.ones(12) * cfg.nominal_joint_friction
            self.motor_strength = cfg.nominal_motor_strength
            self.link_masses = np.zeros(19)
            self.link_inertias = np.array([[0.1, 0.1, 0.1] for i in range(19)])
            self.control_latency_seconds = cfg.nominal_control_latency
            self.pd_latency_seconds = cfg.nominal_pd_latency
            self.pdtau_command_latency_seconds = cfg.nominal_pdtau_command_latency
            self.force_lowpass_filter_window = cfg.force_lowpass_filter_window

            # contact dynamics
            self.ground_friction = cfg.nominal_ground_friction
            self.ground_restitution = cfg.nominal_ground_restitution # KEEP LESS THAN 1, PREFERABLY CLOSER TO 0
            self.ground_spinning_friction = cfg.nominal_ground_spinning_friction 
            self.ground_rolling_friction = cfg.nominal_ground_rolling_friction # KEEP CLOSE TO ZERO
            self.contact_processing_threshold = cfg.nominal_contact_processing_threshold

            
            # robot dynamics
            self.joint_dampings_std = cfg.std_joint_damping
            self.joint_frictions_std = cfg.std_joint_friction
            self.motor_strength_std = cfg.std_motor_strength_pct * self.motor_strength
            self.link_masses_std = cfg.std_link_mass_pct * self.link_masses
            self.link_inertias_std = cfg.std_link_inertia_pct * self.link_inertias
            self.control_latency_seconds_std = cfg.std_control_latency
            self.pd_latency_seconds_std = cfg.std_pd_latency
            self.pdtau_command_latency_seconds_std = cfg.std_pdtau_command_latency
            
            # contact dynamics
            self.ground_friction_std = cfg.std_ground_friction
            self.ground_restitution_std = cfg.std_ground_restitution 
            self.ground_spinning_friction_std = cfg.std_ground_spinning_friction
            self.ground_rolling_friction_std= cfg.std_ground_rolling_friction 
            self.contact_processing_threshold_std = cfg.std_contact_processing_threshold


            self.dynamics_randomization_rate = cfg.dynamics_randomization_rate
            self.skew_rate = 0

        '''
        self.joint_dampings = np.zeros(12)
        self.joint_frictions = np.zeros(12)
        self.motor_strength = 17
        self.link_masses = np.zeros(19)
        #self.link_CoM = None
        self.link_inertias = np.array([[0.1, 0.1, 0.1] for i in range(19)])
        self.control_latency_seconds = 0.0
        self.pd_latency_seconds = 0.0
        #self.control_step = 0.11
        self.ground_friction = 0.875
        #self.ground_restitution = 0.2

        self.joint_dampings_std = 0.05
        self.joint_frictions_std = 0.05
        self.motor_strength_std = 0.20 * self.motor_strength
        self.link_masses_std = 0.50 * self.link_masses
        #self.link_CoM_std = 0.05
        self.link_inertias_std = 0.05 * self.link_inertias
        self.control_latency_seconds_std = 0.040
        self.pd_latency_seconds_std = 0.001
        #self.control_step_std = 0.08
        self.ground_friction_std = 0.375
        #self.ground_restitution_std = 0.05
        
        self.dynamics_randomization_rate = 1.0
        '''

        #self.print()
        #self.ground_restitution = 0
        #self.ground_rolling_friction = 0
        #self.ground_spinning_friction = 0
        #input()

    def apply_skew(self, skew_rate):
        '''
        self.joint_dampings += self.joint_dampings_std * skew_rate
        self.joint_frictions += self.joint_frictions_std * skew_rate
        self.motor_strength += self.motor_strength_std * skew_rate
        self.ground_friction += self.ground_friction_std * skew_rate
        self.link_masses += self.link_masses_std * skew_rate
        self.link_inertias += self.link_inertias_std * skew_rate
        self.pd_latency_seconds += self.pd_latency_seconds_std * skew_rate
        self.control_latency_seconds += self.control_latency_seconds_std * skew_rate
        '''
        self.skew_rate = skew_rate

        print(f'skewed dynamics by skew_rate {skew_rate}')

    def apply_randomization(self):
        new_dynamics = DynamicsParameters()

        # by std
        new_dynamics.joint_dampings = self.joint_dampings + (2 * np.random.rand(*self.joint_dampings.shape) - 1) * self.joint_dampings_std * self.dynamics_randomization_rate + self.joint_dampings_std * self.skew_rate
        new_dynamics.joint_frictions = self.joint_frictions + (2 * np.random.rand(*self.joint_frictions.shape) - 1) * self.joint_frictions_std * self.dynamics_randomization_rate + self.joint_frictions_std * self.skew_rate
        new_dynamics.motor_strength = self.motor_strength + (2 * np.random.rand() - 1) * self.motor_strength_std * self.dynamics_randomization_rate + self.motor_strength_std * self.skew_rate
        new_dynamics.ground_friction = self.ground_friction + (2 * np.random.rand() - 1) * self.ground_friction_std * self.dynamics_randomization_rate + self.ground_friction_std * self.skew_rate
        #new_dynamics.ground_restitution = self.ground_restitution + np.random.rand(*self.ground_restitution.shape) * self.ground_restitution_std
        
        # by %
        new_dynamics.link_masses = self.link_masses + (2 * np.random.rand(*self.link_masses.shape) - 1) * self.link_masses_std * self.dynamics_randomization_rate + self.link_masses_std * self.skew_rate
        #new_dynamics.link_CoM = self.link_CoM + np.multiply(np.random.rand(*self.link_CoM.shape) * self.link_CoM_std, self.link_CoM)
        new_dynamics.link_inertias = self.link_inertias + (2 * np.random.rand(*self.link_inertias.shape) - 1) * self.link_inertias_std * self.dynamics_randomization_rate + self.link_inertias_std * self.skew_rate

        new_dynamics.pd_latency_seconds = self.pd_latency_seconds + np.random.rand() * self.pd_latency_seconds_std * self.dynamics_randomization_rate + self.pd_latency_seconds_std * self.skew_rate
        new_dynamics.pdtau_command_latency_seconds = self.pdtau_command_latency_seconds + np.random.rand() * self.pdtau_command_latency_seconds_std * self.dynamics_randomization_rate + self.pdtau_command_latency_seconds_std * self.skew_rate
        new_dynamics.control_latency_seconds = self.control_latency_seconds + np.random.rand() * self.control_latency_seconds_std * self.dynamics_randomization_rate + self.control_latency_seconds_std * self.skew_rate
        new_dynamics.force_lowpass_filter_window = self.force_lowpass_filter_window

        new_dynamics.ground_restitution = self.ground_restitution
        new_dynamics.ground_spinning_friction = self.ground_spinning_friction
        new_dynamics.ground_rolling_friction = self.ground_rolling_friction
        new_dynamics.contact_processing_threshold = self.contact_processing_threshold

        #print("ctrllatency", new_dynamics.control_latency_seconds)
        
        #self.print()
        #print(self.dynamics_randomization_rate)
        #new_dynamics.print()

        return new_dynamics

    def apply_contact_randomization(self):
        new_dynamics = DynamicsParameters()

        new_dynamics.joint_dampings = self.joint_dampings
        new_dynamics.joint_frictions = self.joint_frictions
        new_dynamics.motor_strength = self.motor_strength
        new_dynamics.link_masses = self.link_masses
        new_dynamics.link_inertias = self.link_inertias
        new_dynamics.pd_latency_seconds = self.pd_latency_seconds
        new_dynamics.pdtau_command_latency_seconds = self.pdtau_command_latency_seconds
        new_dynamics.control_latency_seconds = self.control_latency_seconds
        new_dynamics.force_lowpass_filter_window = self.force_lowpass_filter_window


        # by std
        new_dynamics.ground_friction = max(0.25, self.ground_friction + (2 * np.random.rand() - 1) * self.ground_friction_std * self.dynamics_randomization_rate + self.ground_friction_std * self.skew_rate)
        new_dynamics.ground_restitution = min(0.8, max(0.0, self.ground_restitution + (2 * np.random.rand() - 1) * self.ground_restitution_std * self.dynamics_randomization_rate + self.ground_restitution_std * self.skew_rate))
        new_dynamics.ground_spinning_friction = max(0.0, self.ground_spinning_friction + (2 * np.random.rand() - 1) * self.ground_spinning_friction_std * self.dynamics_randomization_rate + self.ground_spinning_friction_std * self.skew_rate)
        new_dynamics.ground_rolling_friction = max(0.0, self.ground_rolling_friction + (2 * np.random.rand() - 1) * self.ground_rolling_friction_std * self.dynamics_randomization_rate + self.ground_rolling_friction_std * self.skew_rate)
        new_dynamics.contact_processing_threshold = self.contact_processing_threshold + (2 * np.random.rand() - 1) * self.contact_processing_threshold_std * self.dynamics_randomization_rate + self.contact_processing_threshold_std * self.skew_rate

        return new_dynamics

    def print(self):
        print(f"joint dampings: {self.joint_dampings}, \
                joint frictions: {self.joint_frictions}, \
                motor strength: {self.motor_strength}, \
                ground friction: {self.ground_friction} \
                ground restitution: {self.ground_restitution} \
                ground spinning friction: {self.ground_spinning_friction} \
                ground rolling friction: {self.ground_rolling_friction} \
                link masses: {self.link_masses} \
                link inertias: {self.link_inertias} \
                ")

        print(f"pd latency: {self.pd_latency_seconds}, \
                pd command application latency: {self.pdtau_command_latency_seconds}, \
                control latency: {self.control_latency_seconds}, \
                dynamics randomization rate: {self.dynamics_randomization_rate}, \
                skew rate: {self.skew_rate} \
                ")
