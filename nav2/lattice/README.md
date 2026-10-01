# Nav2 Lattice Primitives

`g1_diff_5cm_1m.json` is the Smac Lattice control set used by the
`smac_lattice_full_se2` startup profile.

This is the upstream differential-drive example matching the project's
0.05 m costmap and initial 1.0 m minimum turning radius. Reverse expansion is
disabled separately in `nav2_params.yaml`; the file retains its in-place
rotation primitives for the full-SE(2) profile.

Upstream Navigation2 reference:

- Tag: `1.1.20`
- Source: `https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/lattice_primitives/sample_primitives/5cm_resolution/1m_turning_radius/diff/output.json`
- SHA256: `984ecd63a24eb705ad24a102f7b8021c2f5fa46dcced03122449308728625590`

Navigation2 distributes this sample under the Apache License 2.0; see the
upstream repository's `LICENSE` file.
