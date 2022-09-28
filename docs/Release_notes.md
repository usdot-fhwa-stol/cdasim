CARMA Simulation Release Notes
----------------------------

Version 1.0.1
--------------------------------------------------------

**Summary:**
carma-simulation release version 1.0.1 is a hotfix to correct an artifacts problem in the build environment for 1.0.0.

Features in this release:
-   Added carma-simulation dockerization feature

Fixes in this release:
-   Updated [pom.xml](https://github.com/usdot-fhwa-stol/carma-simulation/blob/carma-simulation-1.0.0/co-simulation/pom.xml) file all version tags to 22.1-SNAPSHOT. The mismatch version tags cause maven build process failed.
-   Removed [bin/](https://github.com/usdot-fhwa-stol/carma-simulation/blob/carma-simulation-1.0.0/co-simulation/.gitignore#:~:text=%23%20Eclipse-,bin/,-.metadata) from .gitignore. Ignore bin/ causes the mosaic.sh cannot be push/pull from GitHub
