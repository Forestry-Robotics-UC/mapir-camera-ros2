# Failure Modes / Troubleshooting

## `/dev/video0` permission denied

Symptoms:

- `v4l2-ctl` fails with permission denied
- node logs show camera open/read failures

Checks:

```bash
ls -l /dev/video0
id
```

Fix:

- ensure your user is in the `video` group (or use a udev rule appropriate for your platform).

## Device busy

Symptoms:

- open succeeds once and then fails with device busy

Checks:

```bash
fuser -v /dev/video0
```

Fix:

- stop the process holding the device, then restart the node.

## Not hitting 30 Hz

Common causes:

- CPU decode/conversion overhead (especially at 1920×1440)
- USB bandwidth limits / host controller contention

Mitigations:

- try `pixel_format:=H264` vs `MJPG`
- reduce resolution (e.g., 1280×720) to confirm the pipeline can sustain 30 Hz
- for indices, use `downsample_factor` and/or `publish_every_n`

## Indices missing / not publishing

Causes:

- band mapping does not provide required bands (e.g., OCN has no Green/RedEdge)

Fix:

- set `filter_set` correctly and/or set explicit `*_channel` overrides.

## RViz image not updating (QoS mismatch)

Symptoms:

- RViz shows `/mapir/image_raw` but no frames update
- camera node logs "incompatible QoS" warnings

Fix:

- In RViz, set the Image display QoS Reliability to `Best Effort`, or
- launch the camera with `qos_best_effort:=false` to publish RELIABLE.
