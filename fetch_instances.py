"""Download the 75 official example instances (public, MIT-licensed).

Source: CG-SHOP/pyutils27 on GitHub -- example_instances.zip sits next to the
README. This script fetches the repo archive from codeload.github.com and
extracts only example_instances.zip into ./instances/ (kept out of git by
.gitignore; agents re-run this after cloning).
"""

import io
import os
import sys
import urllib.request
import zipfile

# Full repo archive (codeload); we extract only example_instances.zip from it.
URL = "https://codeload.github.com/CG-SHOP/pyutils27/zip/refs/heads/main"
INNER_ZIP_SUFFIX = "example_instances.zip"
DEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "instances")


def main() -> None:
    os.makedirs(DEST_DIR, exist_ok=True)
    print(f"downloading {URL} ...")
    with urllib.request.urlopen(URL) as resp:
        repo_zip = zipfile.ZipFile(io.BytesIO(resp.read()))
    inner_name = next(n for n in repo_zip.namelist()
                      if n.endswith(INNER_ZIP_SUFFIX))
    print(f"extracting {inner_name} ...")
    with repo_zip.open(inner_name) as f:
        instances_zip = zipfile.ZipFile(io.BytesIO(f.read()))
        instances_zip.extractall(DEST_DIR)
    n = sum(1 for root, _, files in os.walk(DEST_DIR) for f in files
            if f.endswith(".instance.json"))
    print(f"done: {n} instances in {DEST_DIR}")


if __name__ == "__main__":
    sys.exit(main())
