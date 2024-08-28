<!--
Copyright 2024 Till Blaha (Delft University of Technology)

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option)
any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
more details.

You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>.
-->

# Indiflight Support Package

[![Github](https://img.shields.io/badge/Github-indiflight_support-blue?logo=github)](https://github.com/tudelft/indiflightSupport)
[![Github](https://img.shields.io/badge/Github-indiflight-blue?logo=github)](https://github.com/tudelft/indiflight)


This mono-repo contains all supporting applications and documentation for 
the [Indiflight](https://github.com/tudelft/indiflight) project. Also, all code and instructions to reproduce the results from 2 related conference papers are found here:

1. [![Paper](http://img.shields.io/badge/Paper-arXiv.2405.11723-B3181B?logo=arXiv)](https://arxiv.org/abs/2406.11723) [Documentation/Papers/IROS2024/README.md](Documentation/Papers/IROS2024/README.md)<br>T. M. Blaha, E. J. J. Smeur, and B. D. W. Remes, “Control of Unknown Quadrotors from a Single Throw,” 2024. Accepted at IROS 2024
2. [![Paper](http://img.shields.io/badge/Paper-in_this_repo-B3181B)](LogAnalysis/IMAV2024/IMAV2024_Fly_with_uncertain_motors_and_IMU_draftSubmission.pdf) [Documentation/Papers/IMAV2024/README.md](Documentation/Papers/IMAV2024/README.md)<br>T. M. Blaha, E. J. J. Smeur, B. D. W. Remes and C. C. de Visser “Flying a Quadrotor with Unknown Actuators and Sensor Configuration,” 2024. Accepted at IMAV 2024


## Description

The repository constains simulation code for multirotor flight with 
Indiflight, including data decoding and analysis tools, as well as
configuration files and dockerfiles to reproduce the conference paper results.

Additionally, a utility-bundle is provided to operate automatic flight for 
Indiflight multicopters with a raspberry-pi companion computer.

## Structure

See the individual `README.md`s for more details.

```shell
.
├── Documentation/
│   ├── ...         # Work in Progress
│   ├── Drones      # Info on drone builds and setups
│   └── Papers      # Contains info to reproduce the outputs of our papers
├── Groundstation/  # apps to control and write companion computer apps
├── LogAnalysis/    # Tools for decoding and looking at logs
├── Simulation/     # Soft/Hardware-in-the-loop simulation in python
├── LICENSE
└── README.md
```


## History

* 2024-08-27 0.0.1 First Release


## Authors

* Till Blaha (
[@tblaha](https://github.com/tblaha),
[![ORCID logo](https://info.orcid.org/wp-content/uploads/2019/11/orcid_16x16.png) 0009-0006-0881-1002](https://orcid.org/0009-0006-0881-1002),
Delft University of Technology,
t.m.blaha - @ - tudelft.nl
)
* Robin Ferede [@robinferede](https://github.com/robinferede) (Delft University of Technology)
* Stavrow Bahnam   [@sbahnam](https://github.com/sbahnam) (Delft University of Technology)

## License

[![License](https://img.shields.io/badge/License-GPL--3.0--or--later-4398cc.svg?logo=spdx)](https://spdx.org/licenses/GPL-3.0-or-later.html)

The contents of this repository, with the exception of submodules, are licensed
under a **GNU General Public License v3.0** (see `LICENSE` file). 

Technische Universiteit Delft hereby disclaims all copyright interest in the
program “Indiflight Support” (one line description of the content or function)
written by the Author(s).

Henri Werij, Dean of the Faculty of Aerospace Engineering

© 2024, The Indiflight Support Authors

## Citation

TBD

## Related Projects

### Indiflight
[![Github](https://img.shields.io/badge/Github-indiflight-blue?logo=github)](https://github.com/tudelft/indiflight)

Flight controller firmware based on Betaflight. It includes a completely new controller stack, and interfacing extensions to make it useful for flight control research.


### Indiflight Configurator
[![Github](https://img.shields.io/badge/Github-indiflight_configurator-blue?logo=github)](https://github.com/tudelft/indiflight-configurator)

Our fork supports configuring the new modes in Indiflight and adapted the visualisation to the new coordinate frames.

Caveats: INDI page doesnt work yet; loading firmware online doesnt work


### Blackbox Log Explorer
[![Github](https://img.shields.io/badge/Github-blackbox--log--viewer-blue?logo=github)](https://github.com/tudelft/blackbox-log-viewer)

Our fork has some additional display features and fixes issues that allow parsing the huge Indiflight logs


### Racebian

[![Github](https://img.shields.io/badge/Github-racebian-blue?logo=github)](https://github.com/tudelft/racebian)

A fork of the Raspberry Pi OS that comes with many pre-installed utilities that make a good companion computer.
The Cross-Compiler in the `Groundstation/` directory can be used to push software to a Raspberry Pi Zero 2W running this.


### Pi-protocol

[![Github](https://img.shields.io/badge/Github-pi--protocol-blue?logo=github)](https://github.com/tudelft/pi-protocol)

Minimalist messaging protocol used to communicate with the flight controller. Simpler, less overhead, and easier to include in other software than MSP.

