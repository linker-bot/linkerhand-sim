import sys
import os
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QFileDialog, QGroupBox, QSplitter, QMessageBox,
    QProgressBar, QFrame, QCheckBox, QScrollArea, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QIcon

# NumPy compatibility fix for libraries using deprecated np.int
if not hasattr(np, 'int'):
    np.int = int
# NumPy compatibility fix for libraries using deprecated np.float
if not hasattr(np, 'float'):
    np.float = float

from urdfpy import URDF
from isaacgym import gymapi
import torch

class UrdfJointViewer(QMainWindow):
    def __init__(self, gym_interface):
        super().__init__()
        self.gym_interface = gym_interface
        self.joint_values = {}      # {joint_name: current_value}
        self.joint_limits = {}      # {joint_name: (low, high)}
        self.joint_widgets = {}     # {joint_name: {'slider': slider, 'label': label}}
        self.current_urdf_path = None
        self.setup_ui()
        
    def setup_ui(self):
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        # Main layout with splitter
        main_layout = QHBoxLayout(self.central_widget)
        self.splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(self.splitter)
        
        # Left panel - Controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        self.splitter.addWidget(left_panel)
        
        # File operations group
        file_group = QGroupBox("URDF File")
        file_layout = QVBoxLayout(file_group)
        
        # Horizontal layout for file operations
        file_btn_layout = QHBoxLayout()
        
        self.load_button = QPushButton("Load URDF")
        file_btn_layout.addWidget(self.load_button)
        self.load_button.clicked.connect(self.load_urdf)
        
        self.reset_button = QPushButton("Reset Positions")
        self.reset_button.setEnabled(False)
        file_btn_layout.addWidget(self.reset_button)
        self.reset_button.clicked.connect(self.reset_joint_positions)
        
        file_layout.addLayout(file_btn_layout)
        
        self.urdf_path_label = QLabel("No URDF loaded")
        self.urdf_path_label.setWordWrap(True)
        file_layout.addWidget(self.urdf_path_label)
        
        self.joint_count_label = QLabel("Joints: 0")
        file_layout.addWidget(self.joint_count_label)
        
        left_layout.addWidget(file_group)
        
        # Joints group
        self.joints_scroll_area = QScrollArea()
        self.joints_scroll_area.setWidgetResizable(True)
        self.joints_scroll_area.setFixedHeight(400)  # 控制最大高度

        self.joints_group = QGroupBox("Joint Controls")
        self.joints_layout = QVBoxLayout(self.joints_group)
 
        # Scroll area for joints
        self.joints_scroll = QWidget()
        self.joints_scroll_layout = QVBoxLayout(self.joints_scroll)
        self.joints_scroll_layout.setAlignment(Qt.AlignTop)
        
        self.joints_layout.addWidget(self.joints_scroll)

        self.joints_scroll_area.setWidget(self.joints_group)
        left_layout.addWidget(self.joints_scroll_area)
        
        # Right panel - Simulation status and joint positions
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self.splitter.addWidget(right_panel)
        
        # Simulation status group
        sim_group = QGroupBox("Simulation Status")
        sim_layout = QVBoxLayout(sim_group)
        
        self.sim_status_label = QLabel("Simulation initialized")
        sim_layout.addWidget(self.sim_status_label)
        
        self.fps_label = QLabel("FPS: 0")
        sim_layout.addWidget(self.fps_label)
        
        # Simulation options
        self.auto_update_checkbox = QCheckBox("Auto-update simulation")
        self.auto_update_checkbox.setChecked(True)
        sim_layout.addWidget(self.auto_update_checkbox)
        
        right_layout.addWidget(sim_group)
        
        # Joint positions display group
        positions_group = QGroupBox("Current Joint Positions")
        positions_layout = QVBoxLayout(positions_group)
        
        # Create a scroll area for joint positions
        self.positions_scroll_area = QScrollArea()
        self.positions_scroll_area.setWidgetResizable(True)
        self.positions_scroll_area.setMinimumHeight(300)
        
        # Widget to contain joint position labels
        self.positions_widget = QWidget()
        self.positions_layout = QVBoxLayout(self.positions_widget)
        self.positions_layout.setAlignment(Qt.AlignTop)
        
        self.positions_scroll_area.setWidget(self.positions_widget)
        positions_layout.addWidget(self.positions_scroll_area)
        
        # Dictionary to store position labels
        self.position_labels = {}
        
        # Update button for manual refresh
        self.update_positions_button = QPushButton("Update Positions")
        self.update_positions_button.setEnabled(False)
        self.update_positions_button.clicked.connect(self.update_joint_positions_display)
        positions_layout.addWidget(self.update_positions_button)
        
        right_layout.addWidget(positions_group)
        
        # Spacer to push everything to the top
        right_layout.addStretch()
        
        # Set the splitter's initial position (40% left, 60% right)
        self.splitter.setSizes([400, 600])
        
        # Status bar
        self.statusBar().showMessage("Ready")
        
        # Window settings
        self.setWindowTitle("URDF Joint Viewer with Isaac Gym")
        self.resize(1200, 600)
        
        # Timer for updating FPS and joint positions
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self.update_fps_display)
        self.fps_timer.start(1000)  # Update every second
        self.frame_count = 0
        
        # Timer for updating joint positions display
        self.position_update_timer = QTimer()
        self.position_update_timer.timeout.connect(self.update_joint_positions_display)
        self.position_update_timer.start(100)  # Update every 100ms

    def update_fps_display(self):
        self.fps_label.setText(f"FPS: {self.frame_count}")
        self.frame_count = 0
        
    def update_joint_positions_display(self):
        """Update the joint positions display with current values from Isaac Gym"""
        # if not self.gym_interface.robot_handle or not self.gym_interface.env:
        #     print('111111')
        #     return
            
        try:
            # Get current joint positions from Isaac Gym
            current_positions = self.gym_interface.get_joint_positions()
            
            if current_positions is not None:
                # Update position labels
                for i, joint_name in enumerate(self.gym_interface.dof_names):
                    if joint_name in self.position_labels:
                        position = current_positions[i]
                        self.position_labels[joint_name].setText(f"{position:.4f}")
                        
                        # Color coding based on joint limits
                        # if joint_name in self.joint_limits:
                        #     low, high = self.joint_limits[joint_name]
                        #     if position < low * 0.95 or position > high * 0.95:
                        #         self.position_labels[joint_name].setStyleSheet("color: red; font-weight: bold;")
                        #     elif position < low * 0.8 or position > high * 0.8:
                        #         self.position_labels[joint_name].setStyleSheet("color: orange; font-weight: bold;")
                        #     else:
                        #         self.position_labels[joint_name].setStyleSheet("color: green;")
                        # else:
                        #     self.position_labels[joint_name].setStyleSheet("color: black;")
                            
        except Exception as e:
            print(f"Error updating joint positions display: {e}")
        
    def load_urdf(self):
        file_name, _ = QFileDialog.getOpenFileName(
            self, "Open URDF File", "", "URDF Files (*.urdf);;All Files (*)"
        )
        if not file_name:
            return
            
        # Clear existing joint controls
        self.clear_joint_controls()
        
        try:
            self.statusBar().showMessage(f"Loading URDF: {file_name}...")
            QApplication.processEvents()  # Update UI
            
            # Parse URDF
            robot = URDF.load(file_name)
            
            # Load into Isaac Gym
            try:
                self.gym_interface.load_urdf(file_name)
                self.current_urdf_path = file_name
                self.urdf_path_label.setText(f"Loaded: {os.path.basename(file_name)}")
                self.reset_button.setEnabled(True)
                self.update_positions_button.setEnabled(True)
                
                # Create joint sliders
                joint_count = self.create_joint_sliders(robot)
                self.joint_count_label.setText(f"Joints: {joint_count}")
                
                # Create joint position display
                self.create_joint_position_display()
                
                self.statusBar().showMessage(f"URDF loaded successfully with {joint_count} controllable joints")
                self.sim_status_label.setText("Simulation running")
                
            except Exception as e:
                QMessageBox.critical(self, "Simulation Error", 
                                     f"Failed to load URDF in Isaac Gym: {str(e)}")
                self.statusBar().showMessage(f"Error: {str(e)}")
                
        except Exception as e:
            QMessageBox.critical(self, "URDF Parse Error", 
                                 f"Error parsing URDF file: {str(e)}")
            self.statusBar().showMessage(f"Error: {str(e)}")
    
    def create_joint_position_display(self):
        """Create labels to display current joint positions"""
        # Clear existing position labels
        for label_data in self.position_labels.values():
            label_data.deleteLater()
        self.position_labels.clear()
        
        # Create new labels for each DOF
        for joint_name in self.gym_interface.dof_names:
            # Create frame for each joint position
            joint_frame = QFrame()
            joint_frame.setFrameShape(QFrame.StyledPanel)
            joint_layout = QHBoxLayout(joint_frame)
            
            # Joint name label
            name_label = QLabel(f"<b>{joint_name}:</b>")
            name_label.setMinimumWidth(120)
            joint_layout.addWidget(name_label)
            
            # Position value label
            position_label = QLabel("0.0000")
            position_label.setMinimumWidth(80)
            position_label.setAlignment(Qt.AlignRight)
            position_label.setStyleSheet("font-family: monospace; font-size: 12px;")
            joint_layout.addWidget(position_label)
            
            # Units label
            units_label = QLabel("rad" if joint_name in self.joint_limits else "")
            units_label.setMinimumWidth(30)
            joint_layout.addWidget(units_label)
            
            # Add limits info if available
            if joint_name in self.joint_limits:
                low, high = self.joint_limits[joint_name]
                limits_label = QLabel(f"[{low:.3f}, {high:.3f}]")
                limits_label.setStyleSheet("color: gray; font-size: 10px;")
                joint_layout.addWidget(limits_label)
            
            # Store the position label reference
            self.position_labels[joint_name] = position_label
            
            # Add to positions layout
            self.positions_layout.addWidget(joint_frame)
    
    def create_joint_sliders(self, robot):
        joint_count = 0
        
        for joint in robot.joints:
            if joint.joint_type in ['revolute', 'prismatic'] and joint.limit:
                # Get joint limits with proper handling of None values
                low = joint.limit.lower if joint.limit.lower is not None else -1.0
                high = joint.limit.upper if joint.limit.upper is not None else 1.0
                
                joint_count += 1
                self.joint_limits[joint.name] = (low, high)
                
                # Create a frame for each joint
                joint_frame = QFrame()
                joint_frame.setFrameShape(QFrame.StyledPanel)
                joint_layout = QVBoxLayout(joint_frame)
                
                # Joint name and limits
                header_layout = QHBoxLayout()
                name_label = QLabel(f"<b>{joint.name}</b>")
                header_layout.addWidget(name_label)
                
                limits_label = QLabel(f"[{low:.3f} … {high:.3f}]")
                limits_label.setAlignment(Qt.AlignRight)
                header_layout.addWidget(limits_label)
                
                joint_layout.addLayout(header_layout)
                
                # Slider layout
                slider_layout = QHBoxLayout()
                
                # Value label
                value_label = QLabel("0.000")
                value_label.setMinimumWidth(60)
                value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                
                # Slider
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(0)
                slider.setMaximum(1000)
                
                # Initial value set to 0 or middle point if 0 is out of range
                if low <= 0 <= high:
                    init_val = int((0.0 - low) / (high - low) * 1000)
                else:
                    init_val = int(500)  # Middle of the range
                
                init_val = max(0, min(1000, init_val))
                slider.setValue(init_val)
                
                slider_layout.addWidget(slider)
                slider_layout.addWidget(value_label)
                joint_layout.addLayout(slider_layout)
                
                # Store widget references
                self.joint_widgets[joint.name] = {
                    'slider': slider,
                    'value_label': value_label,
                    'frame': joint_frame
                }
                
                # Connect slider to update function
                slider.valueChanged.connect(
                    lambda v, name=joint.name, lo=low, hi=high:
                        self.update_joint_value(name, lo, hi, v)
                )
                
                # Add joint frame to scrollable area
                self.joints_scroll_layout.addWidget(joint_frame)
                
                # Trigger initial value update
                self.update_joint_value(joint.name, low, high, init_val)
        
        return joint_count
    
    def update_joint_value(self, joint_name, low, high, slider_val):
        if joint_name not in self.joint_widgets:
            return
            
        # Map slider value to actual joint range
        real_val = low + (high - low) * (slider_val / 1000.0)
        self.joint_values[joint_name] = real_val
        
        # Update value label
        self.joint_widgets[joint_name]['value_label'].setText(f"{real_val:.3f}")
        
        # Only update simulation if auto-update is enabled
        if self.auto_update_checkbox.isChecked():
            # Send to Isaac Gym
            self.gym_interface.update_joint_targets(self.joint_values)
    
    def reset_joint_positions(self):
        if not self.joint_widgets:
            return
            
        # Reset all sliders to initial positions
        for joint_name, widgets in self.joint_widgets.items():
            low, high = self.joint_limits[joint_name]
            
            # Set to 0 or middle value if 0 is out of range
            if low <= 0 <= high:
                init_val = int((0.0 - low) / (high - low) * 1000)
            else:
                init_val = 500
                
            init_val = max(0, min(1000, init_val))
            
            # Block signals to prevent multiple updates
            widgets['slider'].blockSignals(True)
            widgets['slider'].setValue(init_val)
            widgets['slider'].blockSignals(False)
            
            # Update joint value
            self.update_joint_value(joint_name, low, high, init_val)
        
        # Force update to Isaac Gym
        self.gym_interface.update_joint_targets(self.joint_values)
        self.statusBar().showMessage("Joint positions reset")
    
    def clear_joint_controls(self):
        # Remove all joint widgets
        for joint_data in self.joint_widgets.values():
            joint_data['frame'].deleteLater()
            
        self.joint_widgets.clear()
        self.joint_values.clear()
        self.joint_limits.clear()
        
        # Clear position labels
        for label_data in self.position_labels.values():
            label_data.deleteLater()
        self.position_labels.clear()
        
        # Reset labels
        self.urdf_path_label.setText("No URDF loaded")
        self.joint_count_label.setText("Joints: 0")
        self.reset_button.setEnabled(False)
        self.update_positions_button.setEnabled(False)
        
    def increment_frame_count(self):
        self.frame_count += 1


class IsaacGymInterface:
    def __init__(self):
        self.gym = gymapi.acquire_gym()
        self.sim = None
        self.env = None
        self.robot_handle = None
        self.dof_names = []
        self.dof_targets = None
        self.viewer_pos = gymapi.Vec3(2, 2, 2) 
        self.viewer_target = gymapi.Vec3(0, 0, 0) 
        self.setup_sim()
        
    def setup_sim(self):
        # Physics simulation parameters
        params = gymapi.SimParams()
        params.dt = 1.0 / 60.0
        params.substeps = 2
        params.up_axis = gymapi.UP_AXIS_Z
        params.gravity = gymapi.Vec3(0, 0, -9.8)
        
        # PhysX-specific parameters
        params.physx.use_gpu = True
        params.physx.solver_type = 1
        params.physx.num_position_iterations = 4
        params.physx.num_velocity_iterations = 1
        params.physx.contact_offset = 0.01
        params.physx.rest_offset = 0.0
        
        # Create simulation
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, params)
        if self.sim is None:
            raise RuntimeError("Failed to create PhysX simulation")
            
        # Create viewer
        cam_props = gymapi.CameraProperties()
        cam_props.width = 1280
        cam_props.height = 720
        self.viewer = self.gym.create_viewer(self.sim, cam_props)
        if self.viewer is None:
            raise RuntimeError("Failed to create viewer")
            
        # Add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        plane_params.distance = 0
        plane_params.static_friction = 1.0
        plane_params.dynamic_friction = 1.0
        plane_params.restitution = 0
        self.gym.add_ground(self.sim, plane_params)
        
        # Set initial camera position
        self.gym.viewer_camera_look_at(
            self.viewer, None, self.viewer_pos, self.viewer_target
        )
        
    def load_urdf(self, urdf_path: str):
        # Destroy existing environment if it exists
        if self.env is not None:
            self.gym.destroy_env(self.env)
            self.robot_handle = None
            self.dof_names = []
            self.dof_targets = None
            self.gym.prepare_sim(self.sim)
            self.close()  
            self.setup_sim() 
            
        # Split path and filename
        asset_root, urdf_file = os.path.split(urdf_path)
        
        # Create environment
        env_spacing = 2.0
        envs_per_row = 1
        env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
        env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)
        
        self.env = self.gym.create_env(
            self.sim, env_lower, env_upper, envs_per_row
        )
        
        # Asset loading options
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True  # Fix the base by default
        asset_options.flip_visual_attachments = False
        asset_options.use_mesh_materials = True
        asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX
        asset_options.override_com = False
        asset_options.override_inertia = False
        asset_options.armature = 0.01
        
        # Load URDF asset
        robot_asset = self.gym.load_urdf(
            self.sim, asset_root, urdf_file, asset_options
        )
        
        if robot_asset is None:
            raise RuntimeError(f"Failed to load URDF: {urdf_path}")
            
        # Create actor
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 0.0)  # Position above ground
        pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)  # Identity quaternion
        
        self.robot_handle = self.gym.create_actor(
            self.env, robot_asset, pose, "robot", 0, 0
        )
        
        # Get DOF properties
        num_dofs = self.gym.get_actor_dof_count(self.env, self.robot_handle)
        names = self.gym.get_actor_dof_names(self.env, self.robot_handle)
        self.dof_names = list(names)
        
        # Create GPU tensor for targets
        self.dof_targets = torch.zeros(num_dofs, dtype=torch.float32, device="cuda:0")
        
        # Set joint drive properties
        props = self.gym.get_actor_dof_properties(self.env, self.robot_handle)
        for i in range(num_dofs):
            props["driveMode"][i] = gymapi.DOF_MODE_POS
            props["stiffness"][i] = 1000.0
            props["damping"][i] = 100.0
            props["armature"][i] = 0.01
            
        self.gym.set_actor_dof_properties(self.env, self.robot_handle, props)
        
        # Set viewer camera to look at the robot
        self.viewer_pos = gymapi.Vec3(0.5, 0.0, 0.5)
        self.viewer_target = gymapi.Vec3(0.0, 0.0, 0.0)
        self.gym.viewer_camera_look_at(
            self.viewer, None, self.viewer_pos, self.viewer_target
        )
        
        # Prepare simulation
        self.gym.prepare_sim(self.sim)
        
    def get_joint_positions(self):
        """Get current joint positions from Isaac Gym"""
        if self.robot_handle is None or self.env is None:
            return None
            
        try:
            # Get DOF states
            dof_states = self.gym.get_actor_dof_states(self.env, self.robot_handle, gymapi.STATE_ALL)
            # Extract positions (dof_states is a structured array with 'pos' and 'vel' fields)
            positions = dof_states['pos']
            return positions
        except Exception as e:
            print(f"Error getting joint positions: {e}")
            return None
        
    def update_joint_targets(self, joint_values):
        if self.robot_handle is None or self.dof_targets is None:
            return
            
        # Match joint names and set target values
        for i, name in enumerate(self.dof_names):
            if name in joint_values:
                self.dof_targets[i] = joint_values[name]
                
        # Apply targets to simulation
        targets_np = self.dof_targets.cpu().numpy()
        self.gym.set_actor_dof_position_targets(
            self.env, self.robot_handle, targets_np
        )
        
    def step(self):
        if self.sim is None:
            return   
        # Step physics and render
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, True)
        
        # Don't sleep here, let Qt handle timing
        
    def close(self):
        if self.viewer:
            self.gym.destroy_viewer(self.viewer)
            self.viewer = None
        if self.env:
            self.gym.destroy_env(self.env)
            self.env = None
        if self.sim:
            self.gym.destroy_sim(self.sim)
            self.sim = None


def main():
    # Create application
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Use Fusion style for better look across platforms
    
    try:
        # Create Isaac Gym interface
        gym_interface = IsaacGymInterface()
        
        # Create main window
        win = UrdfJointViewer(gym_interface)
        win.show()
        
        # Set up simulation timer
        sim_timer = QTimer()
        sim_timer.timeout.connect(lambda: [gym_interface.step(), win.increment_frame_count()])
        sim_timer.start(int(1000 / 60))  # Target 60 FPS
        
        # Start event loop
        sys.exit(app.exec_())
        
    except Exception as e:
        QMessageBox.critical(None, "Error", f"Failed to initialize: {str(e)}")
        
    finally:
        if 'gym_interface' in locals():
            gym_interface.close()


if __name__ == "__main__":
    main()
