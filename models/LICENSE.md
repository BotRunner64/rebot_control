# Model provenance

## reBot Arm RS

`rebot_rs.xml` and `rebot_rs_meshes/` are a visualization-only extraction of the local `reBot-DevArm/Rebot_Arm_description/RS/mjcf/rs_arm.xml` archive: original arm transforms, inertias and visual meshes are retained; camera assets, collision meshes, actuators and equality constraints are omitted. Material metallic/roughness attributes are omitted for classic MuJoCo compatibility. The dual-arm base placement follows the dimensions documented in [ABC-130K](https://github.com/amazon-far/abc/blob/main/abc_sim/models/yam_bimanual_empty.xml); no YAM meshes are included. The scene includes a white tabletop using the documented workstation dimensions.

The RS archive identifies [LAN-GER/reBot-B601-RS-for-mujoco_sim](https://github.com/LAN-GER/reBot-B601-RS-for-mujoco_sim), revision `1249cb6efdf393ba636056fc41df30dc6ba389aa`, as the original model source and records that no upstream LICENSE file existed at that revision. This notice preserves that provenance and does not grant new rights.

`rebot_rs_dynamics.urdf` is a dynamics-only extract of `urdf/RS/urdf/ReBot_Arm_RS.urdf` from the local `reBotArm_control_py` reference repository: original link inertias, joint origins and axes are retained, visual and collision geometry is omitted. The inertial parameters are CAD values and have not been identified on this robot. Provenance is the same as the RS model above; no upstream LICENSE file was found for these assets at the referenced revision.

## Star Arm 102-HD

The following scope notice came with the retained Star Arm model assets. Its relative links refer to the upstream Star-Arm-102 repository.

# License scope

This repository contains components with different license declarations. Preserve each component's existing license and notices.

| Component | Existing declaration |
| --- | --- |
| Release-compatible follower plugin | [Apache License 2.0](lerobot/single-arm/lerobot-robot-stararm102/LICENSE) |
| Release-compatible leader plugin | [Apache License 2.0](lerobot/single-arm/lerobot-teleoperator-stararm102/LICENSE) |
| Refactored HD/FL package | MIT declared in [package metadata](lerobot/single-arm/lerobot-stararm102/pyproject.toml); a corresponding license text has not yet been supplied in that directory |
| Other code, documentation, hardware files, media, and model release | A consistent repository-wide license grant has not yet been established in the published files |

Earlier READMEs linked a root MIT license file that was absent. This scope note records that gap; it does not relicense any component or grant new rights. Ask Fashion Star to confirm the applicable terms before redistributing material without a clear license. Third-party dependencies retain their own licenses.
