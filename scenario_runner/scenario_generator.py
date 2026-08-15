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
import re
import subprocess
from jinja2 import Template
from pathlib import Path
from typing import Dict, Any, List, Optional


class ScenarioGenerator:
    """
    Generate sim_start.sh + sim_stop.sh from parameter.yaml.
    All files go into ./tmp/ (full paths).
    """

    CONFIG_INIT_COMMAND = (
        "cp -a /root/vehicle/config/. /opt/carma/vehicle/config/"
    )
    LEGACY_SERVICE_ALIASES = {
        "carma-simulation": "cdasim",
        "platform": "platform_ros1",
        "msger_roscore": "messenger_roscore",
        "msger_ros1_bridge": "messenger_ros1_bridge",
    }

    def __init__(
        self,
        config_path='config/parameters/parameter.yaml',
        start_template='config/templates/sim_start_template.sh.j2',
        stop_template='config/templates/sim_stop_template.sh.j2',
        tmp_dir='tmp',
        compose_root=None
    ):
        self.config_path = Path(config_path)
        self.start_template = Path(start_template)
        self.stop_template = Path(stop_template)
        self.config: Dict[str, Any] = {}
        self.tmp_dir: Path = Path(tmp_dir).resolve()
        self.compose_root = Path(
            compose_root if compose_root is not None else self.config_path.parent
        ).resolve()
        self.data: Dict[str, Any] = {}  # will hold scenario + temp_dir
        self._config_containers: Dict[str, Dict[str, str]] = {}

    # --------------------------------------------------------------------- #
    # 1. Load config + create ./tmp/
    # --------------------------------------------------------------------- #
    def load_config(self) -> None:
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config not found: {self.config_path}")
        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
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
    # 3. Extract docker-compose.yml (once per project)
    # --------------------------------------------------------------------- #
    @classmethod
    def _service_name(cls, name: str) -> str:
        base_name = re.sub(r"_\d+$", "", name)
        return cls.LEGACY_SERVICE_ALIASES.get(base_name, base_name)

    @staticmethod
    def _service_reference(value: str, renames: Dict[str, str]) -> str:
        if value in renames:
            return renames[value]
        parts = value.split(":")
        if len(parts) >= 2 and parts[0] in ("service", "container"):
            parts[1] = renames.get(parts[1], parts[1])
            return ":".join(parts)
        return value

    @classmethod
    def normalize_compose_services(cls, compose_path: Path) -> None:
        """Normalize known legacy service names and direct service references."""

        compose = yaml.safe_load(compose_path.read_text(encoding="utf-8"))
        services = compose.get("services", {})
        renames = {name: cls._service_name(name) for name in services}
        if all(name == renamed for name, renamed in renames.items()):
            return
        if len(set(renames.values())) != len(renames):
            raise ValueError("Legacy Compose service names normalize to duplicates")

        compose["services"] = {
            renames[name]: service for name, service in services.items()
        }
        for service in compose["services"].values():
            depends_on = service.get("depends_on")
            if isinstance(depends_on, list):
                service["depends_on"] = [
                    renames.get(name, name) for name in depends_on
                ]
            elif isinstance(depends_on, dict):
                service["depends_on"] = {
                    renames.get(name, name): settings
                    for name, settings in depends_on.items()
                }

            network_mode = service.get("network_mode")
            if isinstance(network_mode, str):
                service["network_mode"] = cls._service_reference(
                    network_mode, renames
                )

            volumes_from = service.get("volumes_from")
            if isinstance(volumes_from, list):
                service["volumes_from"] = [
                    cls._service_reference(reference, renames)
                    for reference in volumes_from
                ]

        compose_path.write_text(
            yaml.safe_dump(compose, sort_keys=False), encoding="utf-8"
        )

    def extract_compose_from_image(
        self,
        full_image: str,
        project_name: str,
        compose_path: Optional[str] = None,
        pull_policy: str = 'missing'
    ) -> str:
        if not full_image:
            raise ValueError(f"Missing CONFIG_IMAGE_FULL for {project_name}")

        print(f"Checking config image: {full_image}")
        inspect = subprocess.run(
            ["docker", "image", "inspect", full_image],
            capture_output=True,
            text=True
        )
        if pull_policy == 'always' or (
            pull_policy == 'missing' and inspect.returncode != 0
        ):
            subprocess.run(["docker", "pull", full_image], check=True)
            print(f"Pulled: {full_image}")
        elif inspect.returncode != 0:
            raise RuntimeError(f"Image not found: {full_image}")
        else:
            print(f"Using local image: {full_image}")

        cname = f"inspect-{project_name}-{os.urandom(4).hex()}"
        print(f"Starting inspection container: {cname}")

        try:
            if compose_path:
                # Populate the config volume recursively instead of relying
                # on older config-image commands that only copy regular files.
                subprocess.run(
                    [
                        "docker", "run", "--name", cname,
                        "--entrypoint", "sh", full_image,
                        "-c", self.CONFIG_INIT_COMMAND
                    ],
                    check=True
                )
                src = compose_path
            else:
                subprocess.run(
                    ["docker", "run", "-d", "--name", cname,
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
                    raise RuntimeError(
                        f"Multiple docker-compose.yml in {full_image}: {files}"
                    )
                src = files[0]

            if compose_path:
                dest_dir = self.tmp_dir / f"config-{project_name}"
                dest_dir.mkdir(parents=True, exist_ok=True)
                subprocess.run(
                    ["docker", "cp", f"{cname}:{Path(src).parent}/.", str(dest_dir)],
                    check=True
                )
                dest = dest_dir / Path(src).name
            else:
                dest = self.tmp_dir / f"docker-compose-{project_name}.yml"
                subprocess.run(
                    ["docker", "cp", f"{cname}:{src}", str(dest)], check=True
                )

            self.normalize_compose_services(dest)

            self._config_containers[project_name] = {
                'name': f'{project_name}-config',
                'image': full_image,
                'init_command': self.CONFIG_INIT_COMMAND
            }
            print(f"Extracted {src} → {dest}")
            return str(dest)

        finally:
            subprocess.run(["docker", "rm", "-fv", cname], capture_output=True)

    def _resolve_compose_path(self, configured_path: str) -> str:
        path = Path(configured_path)
        if not path.is_absolute():
            path = self.compose_root / path
        return str(path.resolve())

    def _base_compose(self, component: Dict, project_name: str) -> str:
        if component.get('COMPOSE_FILE'):
            return self._resolve_compose_path(component['COMPOSE_FILE'])
        return self.extract_compose_from_image(
            component.get('CONFIG_IMAGE_FULL'),
            project_name,
            component.get('CONFIG_COMPOSE_PATH'),
            component.get('CONFIG_IMAGE_PULL_POLICY', 'missing')
        )

    def _compose_files(self, component: Dict, project_name: str) -> List[str]:
        compose_files = [self._base_compose(component, project_name)]
        compose_files.extend(
            self._resolve_compose_path(path)
            for path in component.get('COMPOSE_OVERRIDES', [])
        )
        compose_files.extend(
            self._resolve_compose_path(path)
            for path in component.get('INTERNAL_COMPOSE_OVERRIDES', [])
        )

        return compose_files

    # --------------------------------------------------------------------- #
    # 4. Build scenario data ONCE
    # --------------------------------------------------------------------- #
    def _prepare_scenario_data(self) -> Dict:
        es = self.config['env_settings']
        scenario = []

        # CDASim
        cd = es['cdasim']
        compose_files = self._compose_files(cd, cd['PROJECT_NAME'])
        env_file = str(self.tmp_dir / '.env.cdasim')
        scenario.append({
            'PROJECT_NAME': cd['PROJECT_NAME'],
            'compose_file': compose_files[0],
            'compose_files': compose_files,
            'env_file': env_file,
            'services': cd.get('SERVICES', []),
            'platform_net': None,
            'street_net': None
        })

        # Vehicles
        for i, v in enumerate(es.get('vehicles', []), 1):
            compose_files = self._compose_files(v, v['PROJECT_NAME'])
            env_file = str(self.tmp_dir / f'.env.vehicle_{i}')
            scenario.append({
                'PROJECT_NAME': v['PROJECT_NAME'],
                'compose_file': compose_files[0],
                'compose_files': compose_files,
                'env_file': env_file,
                'services': v.get('SERVICES', []),
                'platform_net': f"{v['PROJECT_NAME']}_platform_net",
                'street_net': None
            })

        # Streets
        for i, s in enumerate(es.get('streets', []), 1):
            compose_files = self._compose_files(s, s['PROJECT_NAME'])
            env_file = str(self.tmp_dir / f'.env.street_{i}')
            scenario.append({
                'PROJECT_NAME': s['PROJECT_NAME'],
                'compose_file': compose_files[0],
                'compose_files': compose_files,
                'env_file': env_file,
                'services': s.get('SERVICES', []),
                'platform_net': None,
                'street_net': f"{s['PROJECT_NAME']}_street_net"
            })

        return {
            'scenario': scenario,
            'networks': es.get('runner_networks', []),
            'config_containers': list(self._config_containers.values()),
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
