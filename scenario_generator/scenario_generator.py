# Copyright (C) 2025 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
import yaml
import os
from jinja2 import Template


class ScenarioGenerator:
    """Class to generate scenario files for CDASim simulation deployment."""
    
    def __init__(self, config_path='config/parameters/parameter.yaml', template_path='config/templates/sim_launch_template.sh.j2'):
        """Initialize the ScenarioGenerator with config and template file paths."""
        self.config_path = config_path
        self.template_path = template_path
        self.config = None

    def load_config(self):
        """Load the YAML configuration from the specified path."""
        with open(self.config_path, 'r') as file:
            self.config = yaml.safe_load(file)

    def generate_env_file(self, dir_path, env_filename, env_settings):
        """Generate an .env file with the specified settings in the given directory."""
        env_content = "\n".join(f"{key}={value}" for key, value in env_settings.items())
        os.makedirs(dir_path, exist_ok=True)
        env_path = os.path.join(dir_path, env_filename)
        with open(env_path, 'w') as f:
            f.write(env_content)
        print(f"Generated {env_path}")

    def generate_bash_script(self):
        """Generate the sim_launch.sh bash script from a template and YAML config."""
        # Read the template file
        with open(self.template_path, 'r') as file:
            template_content = file.read()
        
        # Create a Jinja2 template
        template = Template(template_content)
        
        # Render the template with config values
        bash_script = template.render(
            config=self.config,
            cdasim_config_dir=self.config['cdasim_config_dir'],
            street_config_dir=self.config['street_config_dir'],
            carma_config_dir=self.config['carma_config_dir'],
            vehicles=self.config['env_settings']['vehicles'],
            streets=self.config['env_settings']['streets']
        )
        
        return bash_script

    def generate(self):
        """Generate all required files: .env files and sim_launch.sh."""
        if self.config is None:
            self.load_config()

        # Generate .env file for CDASim
        self.generate_env_file(
            self.config['cdasim_config_dir'],
            '.env.cdasim',
            self.config['env_settings']['cdasim']
        )

        # Generate .env files for vehicles
        for i, vehicle in enumerate(self.config['env_settings']['vehicles'], 1):
            self.generate_env_file(
                self.config['carma_config_dir'],
                f'.env.vehicle_{i}',
                vehicle
            )

        # Generate .env files for streets
        for i, street in enumerate(self.config['env_settings']['streets'], 1):
            self.generate_env_file(
                self.config['street_config_dir'],
                f'.env.street_{i}',
                street
            )

        # Generate bash script from template
        bash_script_content = self.generate_bash_script()

        # Write bash script to file
        with open('sim_launch.sh', 'w') as f:
            f.write(bash_script_content)

        # Make the script executable
        os.chmod('sim_launch.sh', 0o755)
        print("Generated sim_launch.sh successfully.")