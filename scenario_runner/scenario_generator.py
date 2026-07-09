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
import subprocess
from jinja2 import Template
from pathlib import Path
from typing import Dict, Any, List


class ScenarioGenerator:
    """
    Generate sim_start.sh + sim_stop.sh from parameter.yaml.
    All files go into ./tmp/ (full paths).
    """

    def __init__(
        self,
        config_path='config/parameters/parameter.yaml',
        start_template='config/templates/sim_start_template.sh.j2',
        stop_template='config/templates/sim_stop_template.sh.j2'
    ):
        self.config_path = Path(config_path)
        self.start_template = Path(start_template)
        self.stop_template = Path(stop_template)
        self.config: Dict[str, Any] = {}
        self.tmp_dir: Path = Path("tmp").resolve()
        self.data: Dict[str, Any] = {}  # will hold scenario + temp_dir

    # --------------------------------------------------------------------- #
    # 1. Load config + create ./tmp/
    # --------------------------------------------------------------------- #
    def load_config(self) -> None:
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config not found: {self.config_path}")
        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.tmp_dir.mkdir(exist_ok=True)
        print(f"Using tmp directory: {self.tmp_dir}")

    # --------------------------------------------------------------------- #
    # 2. Generate .env file (flattened)
    # --------------------------------------------------------------------- #
    def generate_env_file(self, env_filename: str, env_settings: Dict) -> str:
        project_name = env_settings.get('PROJECT_NAME')
        runtime_org = env_settings.get('RUNTIME_IMAGE_ORG')
        runtime_tag = env_settings.get('RUNTIME_IMAGE_TAG')
        settings = env_settings.get('settings', {})

        base = {}
        if project_name: base['PROJECT_NAME'] = project_name
        if runtime_org:  base['RUNTIME_IMAGE_ORG'] = runtime_org
        if runtime_tag:  base['RUNTIME_IMAGE_TAG'] = runtime_tag

        def flatten(d: Dict, prefix: str = "") -> List[str]:
            items = []
            for k, v in d.items():
                key = f"{prefix}{k.upper()}"
                if isinstance(v, dict):
                    items.extend(flatten(v, f"{key}_"))
                else:
                    val = "" if v is None else str(v)
                    items.append(f"{key}={val}")
            return items

        content = "\n".join(flatten(base) + flatten(settings))
        env_path = self.tmp_dir / env_filename
        env_path.write_text(content)
        print(f"Generated {env_path}")
        return str(env_path)

    # --------------------------------------------------------------------- #
    # 3. Extract docker-compose.yml (auto-find, once per project)
    # --------------------------------------------------------------------- #
    def extract_compose_from_image(self, full_image: str, project_name: str) -> str:
        if not full_image:
            raise ValueError(f"Missing CONFIG_IMAGE_FULL for {project_name}")

        print(f"Checking config image: {full_image}")
        pull = subprocess.run(["docker", "pull", full_image], capture_output=True, text=True)
        if pull.returncode != 0:
            if subprocess.run(["docker", "image", "inspect", full_image], capture_output=True).returncode == 0:
                print(f"Using local image: {full_image}")
            else:
                raise RuntimeError(f"Image not found: {full_image}")
        else:
            print(f"Pulled: {full_image}")

        cname = f"inspect-{project_name}-{os.urandom(4).hex()}"
        print(f"Starting inspection container: {cname}")

        try:
            subprocess.run(
                ["docker", "run", "--rm", "-d", "--name", cname,
                 full_image, "sleep", "infinity"],
                check=True
            )

            result = subprocess.run(
                ["docker", "exec", cname,
                 "find", "/", "-type", "f", "-name", "docker-compose.yml"],
                capture_output=True, text=True, check=True
            )
            files = [f.strip() for f in result.stdout.splitlines() if f.strip()]
            if len(files) == 0:
                raise FileNotFoundError(f"No docker-compose.yml in {full_image}")
            if len(files) > 1:
                raise RuntimeError(f"Multiple docker-compose.yml in {full_image}: {files}")

            src = files[0]
            dest = self.tmp_dir / f"docker-compose-{project_name}.yml"
            subprocess.run(["docker", "cp", f"{cname}:{src}", str(dest)], check=True)
            print(f"Extracted {src} → {dest}")
            return str(dest)

        finally:
            subprocess.run(["docker", "kill", cname], capture_output=True)

    # --------------------------------------------------------------------- #
    # 4. Build scenario data ONCE
    # --------------------------------------------------------------------- #
    def _prepare_scenario_data(self) -> Dict:
        es = self.config['env_settings']
        scenario = []

        # CDASim
        cd = es['cdasim']
        compose = self.extract_compose_from_image(cd['CONFIG_IMAGE_FULL'], 'cdasim')
        env_file = str(self.tmp_dir / '.env.cdasim')
        scenario.append({
            'PROJECT_NAME': cd['PROJECT_NAME'],
            'compose_file': compose,
            'env_file': env_file,
            'platform_net': None,
            'street_net': None
        })

        # Vehicles
        for i, v in enumerate(es.get('vehicles', []), 1):
            compose = self.extract_compose_from_image(v['CONFIG_IMAGE_FULL'], v['PROJECT_NAME'])
            env_file = str(self.tmp_dir / f'.env.vehicle_{i}')
            scenario.append({
                'PROJECT_NAME': v['PROJECT_NAME'],
                'compose_file': compose,
                'env_file': env_file,
                'platform_net': f"{v['PROJECT_NAME']}_platform_net",
                'street_net': None
            })

        # Streets
        for i, s in enumerate(es.get('streets', []), 1):
            compose = self.extract_compose_from_image(s['CONFIG_IMAGE_FULL'], s['PROJECT_NAME'])
            env_file = str(self.tmp_dir / f'.env.street_{i}')
            scenario.append({
                'PROJECT_NAME': s['PROJECT_NAME'],
                'compose_file': compose,
                'env_file': env_file,
                'platform_net': None,
                'street_net': f"{s['PROJECT_NAME']}_street_net"
            })

        return {
            'scenario': scenario,
            'temp_dir': str(self.tmp_dir)
        }

    # --------------------------------------------------------------------- #
    # 5. Generate sim_start.sh
    # --------------------------------------------------------------------- #
    def generate_start_script(self) -> str:
        with open(self.start_template, 'r') as f:
            tmpl = Template(f.read())
        content = tmpl.render(**self.data)

        start_path = self.tmp_dir / "sim_start.sh"
        start_path.write_text(content)
        start_path.chmod(0o755)
        print(f"Generated {start_path}")
        return str(start_path)

    # --------------------------------------------------------------------- #
    # 6. Generate sim_stop.sh
    # --------------------------------------------------------------------- #
    def generate_stop_script(self) -> str:
        with open(self.stop_template, 'r') as f:
            tmpl = Template(f.read())
        content = tmpl.render(**self.data)

        stop_path = self.tmp_dir / "sim_stop.sh"
        stop_path.write_text(content)
        stop_path.chmod(0o755)
        print(f"Generated {stop_path}")
        return str(stop_path)

    # --------------------------------------------------------------------- #
    # 7. Public generate() — ONE CALL to _prepare_scenario_data()
    # --------------------------------------------------------------------- #
    def generate(self) -> Dict[str, str]:
        if not self.config:
            self.load_config()

        es = self.config['env_settings']

        # Generate .env files
        self.generate_env_file('.env.cdasim', es['cdasim'])
        for i, v in enumerate(es.get('vehicles', []), 1):
            self.generate_env_file(f'.env.vehicle_{i}', v)
        for i, s in enumerate(es.get('streets', []), 1):
            self.generate_env_file(f'.env.street_{i}', s)

        # ONE CALL: prepare scenario data
        self.data = self._prepare_scenario_data()
        print("Prepared scenario data (cached)")

        # Generate scripts using cached data
        start_script = self.generate_start_script()
        stop_script = self.generate_stop_script()

        return {
            'start_script': start_script,
            'stop_script': stop_script
        }


# # ------------------------------------------------------------------------- #
# # CLI entry point
# # ------------------------------------------------------------------------- #
# if __name__ == "__main__":
#     gen = ScenarioGenerator()
#     scripts = gen.generate()
#     print(f"\nStart: bash {scripts['start_script']}")
#     print(f"Stop:  bash {scripts['stop_script']}")