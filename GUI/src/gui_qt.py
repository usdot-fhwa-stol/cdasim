# Copyright (C) 2025 LEIDOS.

# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import sys
import os
import subprocess
import threading
import shutil
from datetime import datetime
import yaml
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QComboBox, QLabel, QTextEdit, QFileDialog, QVBoxLayout, QHBoxLayout, QToolBar, QDialog, QWidget, QDialogButtonBox
)
from PySide6.QtCore import Qt
from multiprocessing import Process, Queue

class FileSelectDialog(QDialog):
    def __init__(self, title, files, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        layout = QVBoxLayout()
        
        self.combo = QComboBox()
        self.combo.addItems(files)
        layout.addWidget(self.combo)
        
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        self.setLayout(layout)
        
    def get_selected(self):
        return self.combo.currentText() if self.result() == QDialog.Accepted else None

class SimulatorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CDA Simulator GUI")
        self.setMinimumSize(600, 400)
        
        self.repo_folder = None
        self.selected_folder = None
        self.config_folder = None
        self.start_script = None
        self.stop_script = None
        self.docker_compose_file = None
        self.build_script = None
        self.log_queue = Queue()  # For multiprocessing log messages
        
        # Check and create /tmp/.env if it doesn't exist
        env_file = "/tmp/.env"
        if not os.path.exists(env_file):
            try:
                with open(env_file, "w") as f:
                    f.write("# Dummy .env file for stop_simulation\n")
                print(f"[GUI Log] Created {env_file}")
            except Exception as e:
                print(f"[GUI Log] Failed to create {env_file}: {e}")
        
        # Main widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout()
        self.central_widget.setLayout(self.main_layout)
        
        # Toolbar
        self.toolbar = QToolBar("Main Toolbar")
        self.addToolBar(self.toolbar)
        
        # Browse button
        self.browse_button = QPushButton("Browse Repo Folder")
        self.browse_button.clicked.connect(self.browse_folder)
        self.toolbar.addWidget(self.browse_button)
        
        # Open CARMA Logs button
        self.carma_logs_button = QPushButton("Open CARMA Logs")
        self.carma_logs_button.clicked.connect(lambda: self.open_logs("/opt/carma/logs"))
        self.toolbar.addWidget(self.carma_logs_button)
        
        # Open CARMA-Simulation Logs button
        self.carma_sim_logs_button = QPushButton("Open CARMA-Simulation Logs")
        self.carma_sim_logs_button.clicked.connect(lambda: self.open_logs("/opt/carma-simulation/logs"))
        self.toolbar.addWidget(self.carma_sim_logs_button)
        
        # Config dropdown
        self.config_label = QLabel("Select Config:")
        self.main_layout.addWidget(self.config_label)
        self.config_combo = QComboBox()
        self.config_combo.setEnabled(False)
        self.config_combo.currentTextChanged.connect(self.on_config_selected)
        self.main_layout.addWidget(self.config_combo)
        
        # Horizontal layout for Start and Stop buttons
        self.button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_simulation)
        self.button_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_simulation)
        self.button_layout.addWidget(self.stop_button)
        
        self.main_layout.addLayout(self.button_layout)
        
        # Log text area
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.main_layout.addWidget(self.log_text)
        
        # Styling for colored log messages
        self.log_text.setStyleSheet("QTextEdit { color: black; }")
        
        # Debug environment
        self.log_message(f"DISPLAY: {os.environ.get('DISPLAY', 'Not set')}")
        self.log_message(f"QT_QPA_PLATFORM: {os.environ.get('QT_QPA_PLATFORM', 'Not set')}")
        
        # Start log queue processor
        self.process_log_queue()

    def log_message(self, message, tag=None):
        color = "black"
        if tag == "error":
            color = "red"
        elif tag == "warning":
            color = "orange"
        self.log_text.append(f'<span style="color:{color}">{message}</span>')
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
        print(f"[GUI Log] {message}")  # Print to terminal for debugging

    def process_log_queue(self):
        try:
            while not self.log_queue.empty():
                message, tag = self.log_queue.get_nowait()
                self.log_message(message, tag)
        except:
            pass
        self.log_text.repaint()  # Ensure GUI updates
        self.after(100, self.process_log_queue)  # Check queue periodically

    def after(self, ms, func):
        from PySide6.QtCore import QTimer
        QTimer.singleShot(ms, func)

    def open_logs(self, path):
        try:
            if not os.path.exists(path):
                self.log_message(f"Error: Log directory {path} does not exist.", "error")
                return
            self.log_message(f"Opening log directory: {path}")
            subprocess.run(["xdg-open", path], check=True)
        except subprocess.CalledProcessError as e:
            self.log_message(f"Error opening log directory {path}: {e}", "error")
        except Exception as e:
            self.log_message(f"Unexpected error opening log directory {path}: {e}", "error")

    def browse_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select cdasim-config Repo Folder")
        if not folder:
            self.log_message("No folder selected.")
            return
        
        self.repo_folder = folder
        valid_configs = []
        
        for subdir in os.listdir(folder):
            sub_path = os.path.join(folder, subdir)
            if os.path.isdir(sub_path):
                docker_compose = os.path.join(sub_path, "docker-compose.yml")
                build_sh = os.path.join(sub_path, "build-image.sh")
                config_dir = os.path.join(sub_path, "cdasim_config")
                start_sh = os.path.join(config_dir, "start_simulation")
                stop_sh = os.path.join(config_dir, "stop_simulation")
                map_dir = os.path.join(config_dir, "MAP")
                route_dir = os.path.join(config_dir, "route_config")
                
                if (os.path.isfile(docker_compose) and
                    os.path.isfile(build_sh) and
                    os.path.isdir(config_dir) and
                    os.path.isfile(start_sh) and
                    os.path.isfile(stop_sh) and
                    os.path.isdir(map_dir) and os.listdir(map_dir) and
                    os.path.isdir(route_dir) and os.listdir(route_dir)):
                    valid_configs.append(subdir)
        
        if not valid_configs:
            self.log_message("Error: No valid configs found in the repo.", "error")
            self.config_combo.setEnabled(False)
            self.config_combo.clear()
            self.start_button.setEnabled(False)
            return
        
        self.config_combo.blockSignals(True)  # Block signals to prevent automatic trigger
        self.config_combo.clear()
        self.config_combo.addItems(sorted(valid_configs))
        self.config_combo.setCurrentIndex(-1)  # No default selection
        self.config_combo.blockSignals(False)
        self.config_combo.setEnabled(True)
        self.log_message(f"Found {len(valid_configs)} valid config(s): {', '.join(valid_configs)}")

    def on_config_selected(self, config):
        if not config:
            self.log_message("No config selected.")
            return
        
        self.selected_folder = os.path.join(self.repo_folder, config)
        self.log_message(f"Selected config: {config}")
        
        self.docker_compose_file = os.path.join(self.selected_folder, "docker-compose.yml")
        self.build_script = os.path.join(self.selected_folder, "build-image.sh")
        self.config_folder = os.path.join(self.selected_folder, "cdasim_config")
        self.start_script = os.path.join(self.config_folder, "start_simulation")
        self.stop_script = os.path.join(self.config_folder, "stop_simulation")
        
        if self.setup_files() and self.pull_images() and self.build_and_set_config():
            self.start_button.setEnabled(True)
        else:
            self.start_button.setEnabled(False)
            self.selected_folder = None
            self.config_combo.setEnabled(True)

    def select_file(self, title, files):
        if len(files) == 1:
            return files[0]
        
        dialog = FileSelectDialog(title, files, self)
        dialog.exec()
        return dialog.get_selected()

    def get_selected_route(self):
        try:
            with open(self.docker_compose_file, 'r') as f:
                content = yaml.safe_load(f)
            services = content.get('services', {})
            carla_integration = services.get('carma-carla-integration_1', {})
            command = carla_integration.get('command', '')
            
            for cmd_part in command.split():
                if cmd_part.startswith("selected_route:="):
                    return cmd_part.split("selected_route:=", 1)[1].strip("'\"")
            
            self.log_message("Error: selected_route not found in docker-compose.yml", "error")
            return None
        except Exception as e:
            self.log_message(f"Error reading docker-compose.yml: {e}", "error")
            return None

    def setup_files(self):
        map_folder = os.path.join(self.config_folder, "MAP")
        map_files = [f for f in os.listdir(map_folder) if os.path.isfile(os.path.join(map_folder, f))]
        
        if not map_files:
            self.log_message("Error: No files found in MAP folder.", "error")
            return False
        
        selected_map = self.select_file("Select Map File", map_files)
        if not selected_map:
            self.log_message("Error: No map file selected.", "error")
            return False
        
        selected_route_base = self.get_selected_route()
        if not selected_route_base:
            return False
        
        route_folder = os.path.join(self.config_folder, "route_config")
        selected_route_file = f"{selected_route_base}.csv"
        selected_route_path = os.path.join(route_folder, selected_route_file)
        
        if not os.path.isfile(selected_route_path):
            self.log_message(f"Error: Route file {selected_route_file} not found in {route_folder}.", "error")
            return False
        
        try:
            # Copy map
            dest_maps_dir = "/opt/carma/maps"
            os.makedirs(dest_maps_dir, exist_ok=True)
            dest_map = os.path.join(dest_maps_dir, "vector_map.osm")
            if os.path.exists(dest_map):
                now = datetime.now().strftime("%m%d%Y%H%M%S")
                backup = os.path.join(dest_maps_dir, f"vector_map_{now}.osm.backup")
                os.rename(dest_map, backup)
                self.log_message(f"Backed up existing map to {backup}")
            
            selected_map_path = os.path.join(map_folder, selected_map)
            shutil.copy(selected_map_path, dest_map)
            self.log_message(f"Copied map file to {dest_map}")
            
            # Copy route
            dest_routes_dir = "/opt/carma/routes"
            os.makedirs(dest_routes_dir, exist_ok=True)
            dest_route = os.path.join(dest_routes_dir, selected_route_file)
            shutil.copy(selected_route_path, dest_route)
            self.log_message(f"Copied route file to {dest_route}")
        except Exception as e:
            self.log_message(f"Error copying files: {e}", "error")
            return False
        return True

    def pull_images(self):
        try:
            self.log_message(f"Running docker-compose pull in {self.selected_folder}")
            result = subprocess.run(
                ["docker-compose", "pull", "--ignore-pull-failures"],
                cwd=self.selected_folder,
                check=True,
                capture_output=False,
                text=True
            )
            self.log_message("Success: Docker images pulled successfully.")
            return True
        except subprocess.CalledProcessError as e:
            self.log_message(f"Pull Error: Failed to pull Docker images: {e}", "error")
            return False
        except Exception as e:
            self.log_message(f"Unexpected error in pull_images: {e}", "error")
            return False

    def build_and_set_config(self):
        try:
            self.log_message(f"Running build-image.sh in {self.selected_folder}")
            build_result = subprocess.run(
                [self.build_script],
                cwd=self.selected_folder,
                check=True,
                capture_output=True,
                text=True
            )
            build_output = (build_result.stdout + build_result.stderr).strip()
            
            image_name = None
            for line in build_output.splitlines():
                if line.startswith("Final image name:"):
                    image_name = line.split("Final image name:", 1)[1].strip()
                    break
                elif "naming to" in line:
                    parts = line.split("naming to", 1)
                    if len(parts) > 1:
                        image_name = parts[1].strip()
            
            if build_output:
                self.log_message(f"Build Output: {build_output}")
            
            if image_name:
                self.log_message(f"Build Success: Successfully built image: {image_name}")
                self.log_message(f"Running carma config set {image_name} in {self.selected_folder}")
                set_result = subprocess.run(
                    ["carma", "config", "set", image_name],
                    cwd=self.selected_folder,
                    capture_output=True,
                    text=True
                )
                if set_result.stdout:
                    self.log_message(f"Config Set Output: {set_result.stdout.strip()}")
                if set_result.stderr:
                    self.log_message(f"Config Set Warnings/Errors: {set_result.stderr.strip()}", "warning")
                if set_result.returncode != 0:
                    self.log_message(f"Note: Config Set returned non-zero exit code {set_result.returncode}, but proceeding.", "warning")
                else:
                    self.log_message(f"Config Set: Successfully set config with image: {image_name}")
            else:
                self.log_message("Build Success: Image built successfully, but no image name found in output.", "warning")
            return True
        except subprocess.CalledProcessError as e:
            output = (e.stdout + e.stderr).strip() if e.stdout or e.stderr else str(e)
            self.log_message(f"Build Error: Failed to build image: {output}", "error")
            return False
        except Exception as e:
            self.log_message(f"Unexpected error in build_and_set_config: {e}", "error")
            return False

    def start_simulation(self):
        if not self.start_script:
            self.log_message("Error: No start script defined.", "error")
            return
        
        def run_start(log_queue):
            try:
                log_queue.put((f"Running start_simulation in {self.config_folder}", None))
                result = subprocess.run(
                    [self.start_script],
                    cwd=self.config_folder,
                    check=True,
                    capture_output=False,
                    text=True
                )
                log_queue.put(("Simulation started successfully.", None))
            except subprocess.CalledProcessError as e:
                log_queue.put((f"Error: Failed to start simulation: {e}", "error"))
                log_queue.put(("Resetting buttons due to start failure.", None))
                self.reset_buttons()
            except Exception as e:
                log_queue.put((f"Unexpected error in start_simulation: {e}", "error"))
                log_queue.put(("Resetting buttons due to start failure.", None))
                self.reset_buttons()
        
        # Run in process to isolate potential crashes
        p = Process(target=run_start, args=(self.log_queue,))
        p.start()
        
        # Update buttons immediately
        self.start_button.setEnabled(False)
        self.browse_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_simulation(self):
        if not self.stop_script:
            self.log_message("Error: No stop script defined.", "error")
            return
        
        def run_stop(log_queue):
            try:
                log_queue.put((f"Running stop_simulation in {self.config_folder}", None))
                result = subprocess.run(
                    [self.stop_script],
                    cwd=self.config_folder,
                    check=True,
                    capture_output=False,
                    text=True
                )
                log_queue.put(("Simulation stopped successfully.", None))
            except subprocess.CalledProcessError as e:
                log_queue.put((f"Error: Failed to stop simulation: {e}", "error"))
            except Exception as e:
                log_queue.put((f"Unexpected error in stop_simulation: {e}", "error"))
            log_queue.put(("Resetting buttons after stop attempt.", None))
        
        # Run in separate process to isolate potential crashes
        p = Process(target=run_stop, args=(self.log_queue,))
        p.start()
        # Don't join immediately to avoid blocking; rely on log queue for updates
        self.after(1000, self.reset_buttons)  # Delay reset to allow process to complete

    def reset_buttons(self):
        self.start_button.setEnabled(True)
        self.browse_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.log_queue.put(("Buttons reset to initial state.", None))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SimulatorGUI()
    window.show()
    sys.exit(app.exec())