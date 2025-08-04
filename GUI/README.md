# CDA Simulator GUI

This is a graphical user interface (GUI) built with PySide6 for managing CDA (Cooperative Driving Automation) simulations. It allows users to browse a `cdasim-config` repository folder, select valid configurations, set up map and route files, pull Docker images, build and set configurations, and start/stop simulations. The GUI includes logging, toolbar buttons for quick access to log directories, and robust error handling.

## Features
- Browse and select a `cdasim-config` repo folder.
- Automatically detect and list valid configs in a dropdown (based on presence of required files/folders like `docker-compose.yml`, `build-image.sh`, `cdasim_config/start_simulation`, etc.).
- Setup map and route files by copying them to `/opt/carma/maps` and `/opt/carma/routes`.
- Pull Docker images with `docker-compose pull --ignore-pull-failures`.
- Build images and set configurations using `build-image.sh` and `carma config set`.
- Start and stop simulations using scripts in the selected config.
- View logs in the GUI and open log directories (`/opt/carma/logs` and `/opt/carma-simulation/logs`) via toolbar buttons.
- Colored logging for errors (red) and warnings (orange).
- Handles non-zero exit codes from commands like `carma config set` gracefully.
- Isolates potentially crashing operations (e.g., stop simulation) in separate processes to prevent GUI crashes.

## Dependencies
### Python Packages
- Python 3.8 or higher.
- PySide6: For the Qt-based GUI (`pip install PySide6`).
- PyYAML: For parsing YAML files like `docker-compose.yml` (`pip install PyYAML`).

Install all Python dependencies:
```bash
pip install PySide6 PyYAML
```

### System Dependencies (for Qt on Linux)
Qt requires certain system libraries to run properly, especially the `xcb` platform plugin. On Ubuntu/Debian-based systems:
```bash
sudo apt update
sudo apt install libxcb-cursor0 libxcb-xinerama0 libxcb-xinput0 libxkbcommon-x11-0 libfontconfig1 libxrender1 libxi6 libx11-xcb1 libsm6 libxext6 libgl1-mesa-glx xdg-utils
```

For other distributions:
- Fedora/Red Hat: `sudo dnf install libxcb libxcb-devel xcb-util-cursor libXinerama libxkbcommon-x11 fontconfig libXrender libXi libXext mesa-libGL xdg-utils`
- Ensure `xdg-open` is installed for opening log directories.

### Other Requirements
- Docker and Docker Compose: For pulling images and managing containers.
- The `carma` command-line tool: For setting configurations.
- Access to directories like `/opt/carma` and `/opt/carma-simulation` (may require sudo or proper permissions).
- The `cdasim-config` repository with valid configs (e.g., containing `docker-compose.yml`, `build-image.sh`, `cdasim_config/MAP`, `cdasim_config/route_config`, etc.).

## Installation
1. Clone or download this repository/script.
2. Install Python dependencies:
   ```bash
   pip install PySide6 PyYAML
   ```
3. Install system dependencies as above.
4. Ensure Docker is installed and running, and your user has permissions (add to `docker` group if needed: `sudo usermod -aG docker $USER` and log out/in).

## Usage
1. Run the script:
   ```bash
   python3 gui_qt.py
   ```
   - If you encounter Qt platform plugin errors (e.g., "xcb"), verify system dependencies and environment variables like `DISPLAY`.

2. **Browse Repo Folder**:
   - Click "Browse Repo Folder" in the toolbar to select the `cdasim-config` repository folder.
   - Valid configs will be listed in the "Select Config" dropdown.

3. **Select Config**:
   - Choose a config from the dropdown. This will:
     - Prompt for map file selection if multiple maps are available.
     - Automatically detect and copy the route file based on `selected_route` from `docker-compose.yml`.
     - Pull Docker images.
     - Build the image and set the config.

4. **Start Simulation**:
   - Click "Start" to run the `start_simulation` script.

5. **Stop Simulation**:
   - Click "Stop" to run the `stop_simulation` script.
   - Handles errors gracefully without crashing the GUI.

6. **View Logs**:
   - Click "Open CARMA Logs" or "Open CARMA-Simulation Logs" in the toolbar to open the respective directories in your file explorer.

7. **Logging**:
   - All actions, outputs, and errors are logged in the GUI text area (with colors) and printed to the terminal for debugging.

## Troubleshooting
- **Qt Platform Plugin "xcb" Error**:
  - Install missing libraries (see Dependencies).
  - Set `QT_QPA_PLATFORM=offscreen` or `wayland` if needed: `export QT_QPA_PLATFORM=offscreen && python3 gui_qt.py`.
  - For WSL, use an X server like VcXsrv and set `DISPLAY` accordingly.

- **Segmentation Fault on Stop**:
  - The GUI runs `stop_simulation` in a separate process to isolate crashes. If it persists, inspect the `stop_simulation` script for problematic commands (e.g., Docker volume removal) and add error handling.
  - Run the script directly: `cd path/to/config/cdasim_config && ./stop_simulation` to test.

- **Config Set Non-Zero Exit Code**:
  - The GUI proceeds despite non-zero exit codes from `carma config set`, as it works in the terminal. Check logged output for details.

- **Missing `/tmp/.env`**:
  - A dummy file is created automatically at startup. If issues persist, ensure the script doesn't require specific content in `.env`.

- **Docker Permissions**:
  - If Docker commands fail, add your user to the `docker` group and restart your session.

- **Log Directories Not Opening**:
  - Ensure `xdg-open` is installed and the directories exist with proper permissions.

If you encounter issues, run with debug logging:
```bash
export QT_LOGGING_RULES="qt5.*=true"
python3 gui_qt.py
```
Provide the output for further assistance.

## License
This project is open-source under the MIT License (or specify your license).

## Contributors
- Your Name/Team

</readme>