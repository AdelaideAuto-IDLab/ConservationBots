# ConservationBots

ConservationBots Project Code

## Introduction

ConservationBots is a fully autonomous system for Unmanned Aerial Vehicles (UAVs) to track multiple radio-tagged objects. This repository is provided as part of the following paper :

Chen F., Nguyen H. V., Taggart D. A., Falkner K., Rezatofighi S. H., Ranasinghe D. C. (2023) *ConservationBots: Autonomous aerial robot for fast robust wildlife tracking in complex terrains*. Journal of Field Robotics. [Paper](https://onlinelibrary.wiley.com/doi/10.1002/rob.22270).

Cite using:

```
@article{https://doi.org/10.1002/rob.22270,
author = {Chen, Fei and Nguyen, Hoa Van and Taggart, David A. and Falkner, Katrina and Rezatofighi, S. Hamid and Ranasinghe, Damith C.},
title = {ConservationBots: Autonomous aerial robot for fast robust wildlife tracking in complex terrains},
journal = {Journal of Field Robotics},
volume = {n/a},
number = {n/a},
pages = {},
keywords = {aerial robotics, autonomous UAV, position tracking, radio-collared animals, remote sensing, VHF telemetry},
doi = {https://doi.org/10.1002/rob.22270},
url = {https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.22270},
}
```


## Abstract

Abstract Radio tagging and tracking are fundamental to understanding the movements and habitats of wildlife in their natural environments.Today, the most widespread, widely applicable technology for gathering data relies on experienced scientists armed with handheld radio-telemetry equipment to locate low-power radio transmitters attached to wildlife from the ground.Although aerial robots can transform labor-intensive conservation tasks, the realization of autonomous systems for tackling task complexities under real-world conditions remains a challenge. We developed ConservationBots—small aerial robots for tracking multiple, dynamic, radio-tagged wildlife. The aerial robot achieves robust localization performance and fast task completion times—significant for energy-limited aerial systems while avoiding close encounters with potential, counterproductive disturbances to wildlife. Our approach overcomes the technical and practical problems posed by combining a lightweight sensor with new concepts: (i) planning to determine both trajectory and measurement actions guided by an information-theoretic objective, which allows the robot to strategically select near-instantaneous range-only measurements to achieve faster localization, and time-consuming sensor rotation actions to acquire bearing measurements and achieve robust tracking performance; (ii) a bearing detector more robust to noise; and (iii) a tracking algorithm formulation robust to missed and false detections experienced in real-world conditions. We conducted extensive studies: simulations built upon complex signal propagation over high-resolution elevation data on diverse geographical terrains; field testing; studies with wombats (Lasiorhinus latifrons; nocturnal, vulnerable species dwelling in underground warrens) and tracking comparisons with a highly experienced biologist to validate the effectiveness of our aerial robot and demonstrate the significant advantages over the manual method.



## Built With

* [MATLAB](https://mathworks.com/) - Tracking and Planning Algorithm.

  

## Run Simulation

- Goto `ConservationBots/Main/Simulation/Single_Run/` and select a measurement method to run in MATLAB

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
