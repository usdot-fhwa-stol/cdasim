#  Copyright (C) 2025 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

import yaml
import os
from jinja2 import Template

def generate_env_file(config, dir_path, env_filename, env_settings):
    """Generate an .env file with the specified settings in the given directory."""
    env_content = "\n".join(f"{key}={value}" for key, value in env_settings.items())
    os.makedirs(dir_path, exist_ok=True)
    env_path = os.path.join(dir_path, env_filename)
    with open(env_path, 'w') as f:
        f.write(env_content)
    print(f"Generated {env_path}")

def generate_bash_script(config, template_path='sim_launch_template.sh.j2'):
    """Generate the sim_launch.sh bash script from a template and YAML config."""
    # Read the template file
    with open(template_path, 'r') as file:
        template_content = file.read()
    
    # Create a Jinja2 template
    template = Template(template_content)
    
    # Render the template with config values
    bash_script = template.render(
        num_vehicles=len(config['env_settings']['vehicles']),
        num_streets=len(config['env_settings']['streets']),
        cdasim_config_dir=config['cdasim_config_dir'],
        street_config_dir=config['street_config_dir'],
        carma_config_dir=config['carma_config_dir']
    )
    
    return bash_script

def main():
    # Read YAML configuration
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    # Generate .env file for CDASim
    generate_env_file(
        config,
        config['cdasim_config_dir'],
        '.env.cdasim',
        config['env_settings']['cdasim']
    )

    # Generate .env files for vehicles
    for i, vehicle in enumerate(config['env_settings']['vehicles'], 1):
        generate_env_file(
            config,
            config['carma_config_dir'],
            f'.env.vehicle_{i}',
            vehicle
        )

    # Generate .env files for streets
    for i, street in enumerate(config['env_settings']['streets'], 1):
        generate_env_file(
            config,
            config['street_config_dir'],
            f'.env.street_{i}',
            street
        )

    # Generate bash script from template
    bash_script_content = generate_bash_script(config)

    # Write bash script to file
    with open('sim_launch.sh', 'w') as f:
        f.write(bash_script_content)

    # Make the script executable
    os.chmod('sim_launch.sh', 0o755)
    print("Generated sim_launch.sh successfully.")

if __name__ == '__main__':
    main()