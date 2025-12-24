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
import subprocess
import shutil
from pathlib import Path
from scenario_generator import ScenarioGenerator
from data_collector import DataCollector

class ScenarioRunner:
    """Run every scenario defined in parameters.yaml."""

    def __init__(self, parameters_path: str = "config/parameters/parameters.yaml"):
        self.parameters_path = Path(parameters_path)
        self.test_cases = []
        self.tmp_dir = Path("tmp").resolve()
        self.collector = DataCollector()

    def load_parameters(self):
        if not self.parameters_path.exists():
            raise FileNotFoundError(f"{self.parameters_path} not found")
        with open(self.parameters_path, 'r') as f:
            data = yaml.safe_load(f)
        self.test_cases = data.get("test_cases", [])
        if not self.test_cases:
            raise ValueError("No 'test_cases' found in parameters.yaml")
        print(f"Loaded {len(self.test_cases)} scenario(s)")

    def _run_one(self, idx: int, case: dict):
        label = case.get("label", f"scenario_{idx}")
        print(f"\n=== Scenario {idx}: {label} ===")

        # 1. Clean tmp/ (fresh start)
        if self.tmp_dir.exists():
            shutil.rmtree(self.tmp_dir)
        self.tmp_dir.mkdir()

        # 2. Write parameter.yaml to tmp/
        param_path = self.tmp_dir / "parameter.yaml"
        with open(param_path, "w") as f:
            yaml.dump({"env_settings": case["env_settings"]}, f, default_flow_style=False)
        print(f"Generated {param_path}")

        # 3. Generate scripts
        gen = ScenarioGenerator(config_path=str(param_path))
        scripts = gen.generate()
        start_sh = scripts["start_script"]
        stop_sh = scripts["stop_script"]

        # 4. Start
        print(f"Launching: {start_sh}")
        proc = subprocess.Popen(["bash", start_sh])

        # 5. Wait
        runtime = case.get("runtime_seconds", 60)
        print(f"Running for {runtime} seconds...")
        try:
            proc.wait(timeout=runtime)
        except subprocess.TimeoutExpired:
            print("Timeout — stopping.")
        finally:
            # 6. Stop
            print(f"Stopping: {stop_sh}")
            ## check=True: blocks until the shell script is fully done
            subprocess.run(["bash", stop_sh], check=True)
        
        print("Collecting data outputs...")
        self.collector.collect(idx, case)

        # 7. ALWAYS clean tmp/
        shutil.rmtree(self.tmp_dir)
        print(f"Cleaned {self.tmp_dir}")

        print(f"Scenario {idx} complete.\n")

    def run(self):
        if not self.test_cases:
            self.load_parameters()

        for i, case in enumerate(self.test_cases, start=1):
            try:
                self._run_one(i, case)
            except Exception as e:
                print(f"Scenario {i} failed: {e}")
                if self.tmp_dir.exists():
                    shutil.rmtree(self.tmp_dir)


if __name__ == "__main__":
    ScenarioRunner().run()