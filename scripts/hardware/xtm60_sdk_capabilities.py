#!/usr/bin/env python3
"""Print read-only XT SDK Python binding capability metadata."""

import json
import platform
import sys
from pathlib import Path


SDK_ROOT = Path(__file__).resolve().parents[2] / "xtsdk_py"
ARCH = platform.machine()
if ARCH == "aarch64":
    sys.path.insert(0, str(SDK_ROOT / "lib" / "linux" / "aarch64"))
elif ARCH == "x86_64":
    sys.path.insert(0, str(SDK_ROOT / "lib" / "linux" / "x86_64"))
else:
    raise RuntimeError(f"unsupported architecture: {ARCH}")

import xintan_sdk  # noqa: E402


def main():
    result = {
        "architecture": ARCH,
        "modulation_frequency_strings": list(
            xintan_sdk.get_modulation_freq_strings()
        ),
        "image_type_members": {
            name: int(value)
            for name, value in getattr(xintan_sdk.ImageType, "__members__", {}).items()
        },
        "modulation_frequency_members": {
            name: int(value)
            for name, value in getattr(
                xintan_sdk.ModulationFreq, "__members__", {}
            ).items()
        },
        "sdk_frequency_methods": [
            name
            for name in dir(xintan_sdk.XtSdk)
            if any(
                token in name.lower()
                for token in ("freq", "channel", "sync", "trigger", "time")
            )
        ],
        "sdk_set_methods": [
            name for name in dir(xintan_sdk.XtSdk) if name.startswith("set")
        ],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
