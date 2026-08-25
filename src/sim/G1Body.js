/**
 * G1Body — the full kinematic tree, mass distribution and inertia of the G1.
 *
 * GENERATED from public/robots/g1/g1_29dof.urdf by pipeline/gen_body.py. Do not edit by hand.
 *
 * Everything that solves against the robot reads its geometry from here, so a
 * simplified transcription can never drift out of step with the URDF the
 * renderer actually loads. That mattered: the hip roll joint carries a fixed
 * rpy of [0, -0.1749, 0] and the knee an equal and opposite one. They cancel in
 * the neutral pose and nowhere else, and a hand-written chain that dropped them
 * put the solved ankle 53 mm from where urdf-loader draws it.
 *
 * Inertias are included because the microgravity clips conserve angular
 * momentum, which a mass-only model cannot do.
 *
 * Total mass 35.1151 kg over 40 links.
 */
export const G1_TOTAL_MASS = 35.115142;

/** Tree order: every link appears after its parent. */
export const G1_TREE = [
  { name: "pelvis", joint: null, parent: -1, xyz: [0,0,0], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 3.813, com: [0,0,-0.07605], I: [0.010549,0,2.1e-06,0.0093089,0,0.0079184] },
  { name: "pelvis_contour_link", joint: null, parent: 0, xyz: [0,0,0], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0.001, com: [0,0,0], I: [1e-07,0,0,1e-07,0,1e-07] },
  { name: "left_hip_pitch_link", joint: "left_hip_pitch_joint", parent: 0, xyz: [0,0.064452,-0.1027], rpy: [0,0,0], axis: "y", sign: 1, limit: [-2.5307,2.8798], mass: 1.35, com: [0.002741,0.047791,-0.02606], I: [0.001811,3.68e-05,-3.44e-05,0.0014193,0.000171,0.0012812] },
  { name: "left_hip_roll_link", joint: "left_hip_roll_joint", parent: 2, xyz: [0,0.052,-0.030465], rpy: [0,-0.1749,0], axis: "x", sign: 1, limit: [-0.5236,2.9671], mass: 1.52, com: [0.029812,-0.001045,-0.087934], I: [0.0023773,-3.8e-06,-0.0003908,0.0024123,1.84e-05,0.0016595] },
  { name: "left_hip_yaw_link", joint: "left_hip_yaw_joint", parent: 3, xyz: [0.025001,0,-0.12412], rpy: [0,0,0], axis: "z", sign: 1, limit: [-2.7576,2.7576], mass: 1.702, com: [-0.057709,-0.010981,-0.15078], I: [0.0057774,-0.0005411,-0.0023948,0.0076124,-0.0007072,0.003149] },
  { name: "left_knee_link", joint: "left_knee_joint", parent: 4, xyz: [-0.078273,0.0021489,-0.17734], rpy: [0,0.1749,0], axis: "y", sign: 1, limit: [-0.087267,2.8798], mass: 1.932, com: [0.005457,0.003964,-0.12074], I: [0.011329,4.82e-05,-4.49e-05,0.011277,-0.0007146,0.0015168] },
  { name: "left_ankle_pitch_link", joint: "left_ankle_pitch_joint", parent: 5, xyz: [0,-9.4445e-05,-0.30001], rpy: [0,0,0], axis: "y", sign: 1, limit: [-0.87267,0.5236], mass: 0.074, com: [-0.007269,0,0.011137], I: [8.4e-06,0,-2.9e-06,1.89e-05,0,1.26e-05] },
  { name: "left_ankle_roll_link", joint: "left_ankle_roll_joint", parent: 6, xyz: [0,0,-0.017558], rpy: [0,0,0], axis: "x", sign: 1, limit: [-0.2618,0.2618], mass: 0.608, com: [0.026505,0,-0.016425], I: [0.0002231,2e-07,8.91e-05,0.0016161,-1e-07,0.0016667] },
  { name: "right_hip_pitch_link", joint: "right_hip_pitch_joint", parent: 0, xyz: [0,-0.064452,-0.1027], rpy: [0,0,0], axis: "y", sign: 1, limit: [-2.5307,2.8798], mass: 1.35, com: [0.002741,-0.047791,-0.02606], I: [0.001811,-3.68e-05,-3.44e-05,0.0014193,-0.000171,0.0012812] },
  { name: "right_hip_roll_link", joint: "right_hip_roll_joint", parent: 8, xyz: [0,-0.052,-0.030465], rpy: [0,-0.1749,0], axis: "x", sign: 1, limit: [-2.9671,0.5236], mass: 1.52, com: [0.029812,0.001045,-0.087934], I: [0.0023773,3.8e-06,-0.0003908,0.0024123,-1.84e-05,0.0016595] },
  { name: "right_hip_yaw_link", joint: "right_hip_yaw_joint", parent: 9, xyz: [0.025001,0,-0.12412], rpy: [0,0,0], axis: "z", sign: 1, limit: [-2.7576,2.7576], mass: 1.702, com: [-0.057709,0.010981,-0.15078], I: [0.0057774,0.0005411,-0.0023948,0.0076124,0.0007072,0.003149] },
  { name: "right_knee_link", joint: "right_knee_joint", parent: 10, xyz: [-0.078273,-0.0021489,-0.17734], rpy: [0,0.1749,0], axis: "y", sign: 1, limit: [-0.087267,2.8798], mass: 1.932, com: [0.005457,-0.003964,-0.12074], I: [0.011329,-4.82e-05,4.49e-05,0.011277,0.0007146,0.0015168] },
  { name: "right_ankle_pitch_link", joint: "right_ankle_pitch_joint", parent: 11, xyz: [0,9.4445e-05,-0.30001], rpy: [0,0,0], axis: "y", sign: 1, limit: [-0.87267,0.5236], mass: 0.074, com: [-0.007269,0,0.011137], I: [8.4e-06,0,-2.9e-06,1.89e-05,0,1.26e-05] },
  { name: "right_ankle_roll_link", joint: "right_ankle_roll_joint", parent: 12, xyz: [0,0,-0.017558], rpy: [0,0,0], axis: "x", sign: 1, limit: [-0.2618,0.2618], mass: 0.608, com: [0.026505,0,-0.016425], I: [0.0002231,-2e-07,8.91e-05,0.0016161,1e-07,0.0016667] },
  { name: "waist_yaw_link", joint: "waist_yaw_joint", parent: 0, xyz: [0,0,0], rpy: [0,0,0], axis: "z", sign: 1, limit: [-2.618,2.618], mass: 0.244, com: [0.003964,0,0.018769], I: [9.9587e-05,-1.833e-06,-1.2617e-05,0.00012411,-1.18e-07,0.00015586] },
  { name: "waist_roll_link", joint: "waist_roll_joint", parent: 14, xyz: [-0.0039635,0,0.035], rpy: [0,0,0], axis: "x", sign: 1, limit: [-0.52,0.52], mass: 0.047, com: [0,-0.000236,0.010111], I: [7.515e-06,0,0,6.398e-06,9.9e-08,3.988e-06] },
  { name: "torso_link", joint: "waist_pitch_joint", parent: 15, xyz: [0,0,0.019], rpy: [0,0,0], axis: "y", sign: 1, limit: [-0.52,0.52], mass: 8.562, com: [0.002601,0.000257,0.153719], I: [0.065675,-8.597e-05,-0.00173725,0.0535352,8.6899e-05,0.0308081] },
  { name: "logo_link", joint: null, parent: 16, xyz: [0.0039635,0,-0.054], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0.001, com: [0,0,0], I: [1e-07,0,0,1e-07,0,1e-07] },
  { name: "head_link", joint: null, parent: 16, xyz: [0.0039635,0,-0.054], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 1.036, com: [0.005267,0.000299,0.449869], I: [0.00408505,-2.543e-06,-6.9455e-05,0.00418521,-3.726e-06,0.00180791] },
  { name: "waist_support_link", joint: null, parent: 16, xyz: [0.0039635,0,-0.054], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0.001, com: [0,0,0], I: [1e-07,0,0,1e-07,0,1e-07] },
  { name: "imu_in_torso", joint: null, parent: 16, xyz: [-0.03959,-0.00224,0.13792], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0, com: [0,0,0], I: [0,0,0,0,0,0] },
  { name: "imu_in_pelvis", joint: null, parent: 0, xyz: [0.04525,0,-0.08339], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0, com: [0,0,0], I: [0,0,0,0,0,0] },
  { name: "d435_link", joint: null, parent: 16, xyz: [0.0576235,0.01753,0.41987], rpy: [0,0.830777,0], axis: null, sign: 1, limit: null, mass: 0, com: [0,0,0], I: [0,0,0,0,0,0] },
  { name: "mid360_link", joint: null, parent: 16, xyz: [0.0002835,3e-05,0.40618], rpy: [0,0.0401426,0], axis: null, sign: 1, limit: null, mass: 0, com: [0,0,0], I: [0,0,0,0,0,0] },
  { name: "left_shoulder_pitch_link", joint: "left_shoulder_pitch_joint", parent: 16, xyz: [0.0039563,0.10022,0.23778], rpy: [0.27931,5.4949e-05,-0.00019159], axis: "y", sign: 1, limit: [-3.0892,2.6704], mass: 0.718, com: [0,0.035892,-0.011628], I: [0.0004291,-9.2e-06,6.4e-06,0.000453,2.26e-05,0.000423] },
  { name: "left_shoulder_roll_link", joint: "left_shoulder_roll_joint", parent: 24, xyz: [0,0.038,-0.013831], rpy: [-0.27925,0,0], axis: "x", sign: 1, limit: [-1.5882,2.2515], mass: 0.643, com: [-0.000227,0.00727,-0.063243], I: [0.0006177,-1e-06,8.7e-06,0.0006912,-5.3e-06,0.0003894] },
  { name: "left_shoulder_yaw_link", joint: "left_shoulder_yaw_joint", parent: 25, xyz: [0,0.00624,-0.1032], rpy: [0,0,0], axis: "z", sign: 1, limit: [-2.618,2.618], mass: 0.734, com: [0.010773,-0.002949,-0.072009], I: [0.0009988,7.9e-06,0.0001412,0.0010605,-2.86e-05,0.0004354] },
  { name: "left_elbow_link", joint: "left_elbow_joint", parent: 26, xyz: [0.015783,0,-0.080518], rpy: [0,0,0], axis: "y", sign: 1, limit: [-1.0472,2.0944], mass: 0.6, com: [0.064956,0.004454,-0.010062], I: [0.0002891,6.53e-05,1.72e-05,0.0004152,-5.6e-06,0.0004197] },
  { name: "left_wrist_roll_link", joint: "left_wrist_roll_joint", parent: 27, xyz: [0.1,0.00188791,-0.01], rpy: [0,0,0], axis: "x", sign: 1, limit: [-1.97222,1.97222], mass: 0.085445, com: [0.0171394,0.000537591,4.8864e-07], I: [4.82154e-05,-4.24511e-06,5.10599e-09,3.7229e-05,-1.23525e-09,5.48211e-05] },
  { name: "left_wrist_pitch_link", joint: "left_wrist_pitch_joint", parent: 28, xyz: [0.038,0,0], rpy: [0,0,0], axis: "y", sign: 1, limit: [-1.61443,1.61443], mass: 0.48405, com: [0.0229999,-0.00111685,-0.00111658], I: [0.000165796,-1.23121e-05,1.2317e-05,0.000429541,8.14177e-07,0.000429537] },
  { name: "left_wrist_yaw_link", joint: "left_wrist_yaw_joint", parent: 29, xyz: [0.046,0,0], rpy: [0,0,0], axis: "z", sign: 1, limit: [-1.61443,1.61443], mass: 0.0845765, com: [0.0220038,0.000494851,0.000538611], I: [4.92913e-05,-4.57355e-07,4.45868e-06,5.97334e-05,4.32172e-07,3.92808e-05] },
  { name: "left_rubber_hand", joint: null, parent: 30, xyz: [0.0415,0.003,0], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0.17, com: [0.0536131,-0.00295905,0.00215413], I: [0.000100995,3.61859e-05,-7.43015e-07,0.000281359,3.3019e-06,0.000218948] },
  { name: "right_shoulder_pitch_link", joint: "right_shoulder_pitch_joint", parent: 16, xyz: [0.0039563,-0.10021,0.23778], rpy: [-0.27931,5.4949e-05,0.00019159], axis: "y", sign: 1, limit: [-3.0892,2.6704], mass: 0.718, com: [0,-0.035892,-0.011628], I: [0.0004291,9.2e-06,6.4e-06,0.000453,-2.26e-05,0.000423] },
  { name: "right_shoulder_roll_link", joint: "right_shoulder_roll_joint", parent: 32, xyz: [0,-0.038,-0.013831], rpy: [0.27925,0,0], axis: "x", sign: 1, limit: [-2.2515,1.5882], mass: 0.643, com: [-0.000227,-0.00727,-0.063243], I: [0.0006177,1e-06,8.7e-06,0.0006912,5.3e-06,0.0003894] },
  { name: "right_shoulder_yaw_link", joint: "right_shoulder_yaw_joint", parent: 33, xyz: [0,-0.00624,-0.1032], rpy: [0,0,0], axis: "z", sign: 1, limit: [-2.618,2.618], mass: 0.734, com: [0.010773,0.002949,-0.072009], I: [0.0009988,-7.9e-06,0.0001412,0.0010605,2.86e-05,0.0004354] },
  { name: "right_elbow_link", joint: "right_elbow_joint", parent: 34, xyz: [0.015783,0,-0.080518], rpy: [0,0,0], axis: "y", sign: 1, limit: [-1.0472,2.0944], mass: 0.6, com: [0.064956,-0.004454,-0.010062], I: [0.0002891,-6.53e-05,1.72e-05,0.0004152,5.6e-06,0.0004197] },
  { name: "right_wrist_roll_link", joint: "right_wrist_roll_joint", parent: 35, xyz: [0.1,-0.00188791,-0.01], rpy: [0,0,0], axis: "x", sign: 1, limit: [-1.97222,1.97222], mass: 0.085445, com: [0.0171394,-0.000537591,4.8864e-07], I: [4.82154e-05,4.24511e-06,5.10599e-09,3.7229e-05,1.23525e-09,5.48211e-05] },
  { name: "right_wrist_pitch_link", joint: "right_wrist_pitch_joint", parent: 36, xyz: [0.038,0,0], rpy: [0,0,0], axis: "y", sign: 1, limit: [-1.61443,1.61443], mass: 0.48405, com: [0.0229999,0.00111685,-0.00111658], I: [0.000165796,1.23121e-05,1.2317e-05,0.000429541,-8.14177e-07,0.000429537] },
  { name: "right_wrist_yaw_link", joint: "right_wrist_yaw_joint", parent: 37, xyz: [0.046,0,0], rpy: [0,0,0], axis: "z", sign: 1, limit: [-1.61443,1.61443], mass: 0.0845765, com: [0.0220038,-0.000494851,0.000538611], I: [4.92913e-05,4.57355e-07,4.45868e-06,5.97334e-05,-4.32172e-07,3.92808e-05] },
  { name: "right_rubber_hand", joint: null, parent: 38, xyz: [0.0415,-0.003,0], rpy: [0,0,0], axis: null, sign: 1, limit: null, mass: 0.17, com: [0.0536131,0.00295905,0.00215413], I: [0.000100995,-3.61859e-05,-7.43015e-07,0.000281359,-3.3019e-06,0.000218948] },
];

/** [ixx, ixy, ixz, iyy, iyz, izz] -> row-major 3x3. */
export const inertiaMatrix = (I) => [I[0], I[1], I[2], I[1], I[3], I[4], I[2], I[4], I[5]];
