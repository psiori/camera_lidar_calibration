# Spec Delta

## MODIFIED Requirements

### Requirement: NID optimization v1

The v1 optimizer SHALL run vlcal NID (BFGS or Nelder-Mead) on all ready calibration datasets sharing one `T_camera_lidar`, respecting the global range mask on structured input used to produce each dataset’s cloud and each dataset’s image optimization mask (see `image-optimization-mask`) on that dataset’s camera image during the NID cost. Image masks SHALL apply for both `NID_BFGS` and `NID_NELDER_MEAD` registration types. Datasets without an image mask SHALL use all image pixels in the cost.

#### Scenario: Multi-dataset optimize

- **WHEN** the user runs alignment with two calibration datasets
- **THEN** NID optimizes a single `T_camera_lidar` using both datasets jointly

#### Scenario: Masked pixels excluded from cost

- **WHEN** the user runs alignment and a ready dataset has an image mask with excluded pixels
- **THEN** those pixels do not contribute to the NID mutual-information cost for that dataset

#### Scenario: Nelder-Mead with image mask

- **WHEN** the user selects Nelder–Mead registration and runs alignment with image masks defined
- **THEN** excluded pixels are ignored in the Nelder–Mead cost evaluation the same as in the BFGS path
