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

from pathlib import Path
import shutil

class DataCollector:

    def latest_subdir(self, base: Path):
        if not base.exists():
            return None
        subs = [p for p in base.iterdir() if p.is_dir()]
        return max(subs, key=lambda p: p.stat().st_mtime) if subs else None

    def collect(self, index: int, config: dict):
        data_output = config.get("data_output")
        if not data_output:
            print("No data_output section. Skipping.")
            return

        out_dir = Path(data_output["output_directory"])
        out_dir.mkdir(parents=True, exist_ok=True)

        label = config.get("label", f"scenario_{index}")
        case_dir = out_dir / data_output.get(
            "rename_format", "{label}"
        ).format(index=index, label=label)
        case_dir.mkdir(parents=True, exist_ok=True)

        collect_cfg = data_output.get("collect", {})

        for key, value in collect_cfg.items():
            print(key, value)
            self._collect_folder(Path(value), case_dir / key, key == "mosaic_logs")

    def _collect_folder(self, src_base: Path, dest: Path, latest_only: bool = False):
        src = (self.latest_subdir(src_base) or src_base) if latest_only else src_base
        print(src)
        if src and src.exists():
            shutil.copytree(src, dest, dirs_exist_ok=True, symlinks=True)
            print(f"Copied {src} → {dest}")
        else:
            print(f"No logs found in: {src_base}")
