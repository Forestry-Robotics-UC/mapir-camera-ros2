"""Parameter loading helpers for the MAPIR Survey3 ROS 2 node.

This module centralizes the camera-node parameter schema so the runtime node,
launch files, and documentation all refer to one table of defaults.  The goal
is to keep parameter declaration, typed loading, and ROS-facing compatibility
assignment in one place instead of duplicating those concerns in the node
constructor.
"""

from __future__ import annotations

from dataclasses import dataclass

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class CameraNodeParameters:
    """Strongly typed snapshot of the camera node configuration.

    The dataclass mirrors the public ROS parameter surface after normalization
    and basic validation so downstream helpers can depend on stable Python
    types instead of repeatedly querying and casting parameters from the node.
    """

    debug: bool
    debug_period_s: float
    video_device: str
    req_width: int
    req_height: int
    req_fps: float
    frame_id: str
    camera_name: str
    camera_info_url: str
    pixel_format: str
    use_gstreamer: bool
    gstreamer_pipeline: str
    qos_depth: int
    qos_best_effort: bool
    uvc_controls_enabled: bool
    uvc_controls_device: str
    auto_exposure_mode: int
    exposure_time_absolute: int
    gain: int
    exposure_dynamic_framerate: int
    white_balance_automatic: int
    white_balance_temperature: int
    power_line_frequency: int
    metadata_enabled: bool
    metadata_device: str
    metadata_topic: str
    metadata_log_path: str
    metadata_log_flush_every_n: int
    vignette_enabled: bool
    vignette_flatfield_b_path: str
    vignette_flatfield_g_path: str
    vignette_flatfield_r_path: str
    vignette_dark_current: tuple[int, int, int]


PARAMETER_DEFAULTS = (
    ("debug", False),
    ("debug_period_s", 1.0),
    ("video_device", "/dev/video0"),
    ("image_width", 1280),
    ("image_height", 720),
    ("framerate", 30.0),
    ("frame_id", "mapir3_optical_frame"),
    ("camera_name", "mapir3_ocn"),
    ("camera_info_url", ""),
    ("pixel_format", "MJPG"),
    ("use_gstreamer", False),
    ("gstreamer_pipeline", ""),
    ("qos_depth", 5),
    ("qos_best_effort", True),
    ("uvc_controls_enabled", False),
    ("uvc_controls_device", ""),
    ("auto_exposure_mode", -1),
    ("exposure_time_absolute", -1),
    ("gain", -1),
    ("exposure_dynamic_framerate", -1),
    ("white_balance_automatic", -1),
    ("white_balance_temperature", -1),
    ("power_line_frequency", -1),
    ("metadata_enabled", False),
    ("metadata_device", "/dev/video1"),
    ("metadata_topic", "metadata"),
    ("metadata_log_path", ""),
    ("metadata_log_flush_every_n", 30),
    ("vignette_enabled", False),
    ("vignette_flatfield_b_path", ""),
    ("vignette_flatfield_g_path", ""),
    ("vignette_flatfield_r_path", ""),
    ("vignette_dark_current_b", 0),
    ("vignette_dark_current_g", 0),
    ("vignette_dark_current_r", 0),
)


def declare_camera_parameters(node) -> None:
    """Declare every supported ROS parameter with its node-level default."""
    for name, default in PARAMETER_DEFAULTS:
        node.declare_parameter(name, default)


def load_camera_parameters(node) -> CameraNodeParameters:
    """Read declared ROS parameters and normalize them into a typed snapshot."""
    return CameraNodeParameters(
        debug=bool(node.get_parameter("debug").value),
        debug_period_s=float(node.get_parameter("debug_period_s").value),
        video_device=str(node.get_parameter("video_device").value),
        req_width=int(node.get_parameter("image_width").value),
        req_height=int(node.get_parameter("image_height").value),
        req_fps=float(node.get_parameter("framerate").value),
        frame_id=str(node.get_parameter("frame_id").value),
        camera_name=str(node.get_parameter("camera_name").value),
        camera_info_url=str(node.get_parameter("camera_info_url").value),
        pixel_format=str(node.get_parameter("pixel_format").value).upper(),
        use_gstreamer=bool(node.get_parameter("use_gstreamer").value),
        gstreamer_pipeline=str(node.get_parameter("gstreamer_pipeline").value),
        qos_depth=max(1, int(node.get_parameter("qos_depth").value)),
        qos_best_effort=bool(node.get_parameter("qos_best_effort").value),
        uvc_controls_enabled=bool(node.get_parameter("uvc_controls_enabled").value),
        uvc_controls_device=str(node.get_parameter("uvc_controls_device").value),
        auto_exposure_mode=int(node.get_parameter("auto_exposure_mode").value),
        exposure_time_absolute=int(node.get_parameter("exposure_time_absolute").value),
        gain=int(node.get_parameter("gain").value),
        exposure_dynamic_framerate=int(
            node.get_parameter("exposure_dynamic_framerate").value
        ),
        white_balance_automatic=int(node.get_parameter("white_balance_automatic").value),
        white_balance_temperature=int(
            node.get_parameter("white_balance_temperature").value
        ),
        power_line_frequency=int(node.get_parameter("power_line_frequency").value),
        metadata_enabled=bool(node.get_parameter("metadata_enabled").value),
        metadata_device=str(node.get_parameter("metadata_device").value),
        metadata_topic=str(node.get_parameter("metadata_topic").value),
        metadata_log_path=str(node.get_parameter("metadata_log_path").value),
        metadata_log_flush_every_n=max(
            1, int(node.get_parameter("metadata_log_flush_every_n").value)
        ),
        vignette_enabled=bool(node.get_parameter("vignette_enabled").value),
        vignette_flatfield_b_path=str(
            node.get_parameter("vignette_flatfield_b_path").value
        ),
        vignette_flatfield_g_path=str(
            node.get_parameter("vignette_flatfield_g_path").value
        ),
        vignette_flatfield_r_path=str(
            node.get_parameter("vignette_flatfield_r_path").value
        ),
        vignette_dark_current=(
            int(node.get_parameter("vignette_dark_current_b").value),
            int(node.get_parameter("vignette_dark_current_g").value),
            int(node.get_parameter("vignette_dark_current_r").value),
        ),
    )


def apply_camera_parameters(node, params: CameraNodeParameters) -> None:
    """Mirror normalized parameters onto legacy node attributes.

    The refactor keeps the existing runtime attribute names so capture,
    metadata, and publish helpers can remain compatible with the original node
    implementation while parameter access moves behind :class:`CameraNodeParameters`.
    """
    node.debug = params.debug
    node.debug_period_s = params.debug_period_s
    node.video_device = params.video_device
    node.req_width = params.req_width
    node.req_height = params.req_height
    node.req_fps = params.req_fps
    node.frame_id = params.frame_id
    node.camera_name = params.camera_name
    node.camera_info_url = params.camera_info_url
    node.pixel_format = params.pixel_format
    node.use_gstreamer = params.use_gstreamer
    node.gstreamer_pipeline = params.gstreamer_pipeline
    node.qos_depth = params.qos_depth
    node.qos_best_effort = params.qos_best_effort
    node.uvc_controls_enabled = params.uvc_controls_enabled
    node.uvc_controls_device = params.uvc_controls_device
    node.auto_exposure_mode = params.auto_exposure_mode
    node.exposure_time_absolute = params.exposure_time_absolute
    node.gain = params.gain
    node.exposure_dynamic_framerate = params.exposure_dynamic_framerate
    node.white_balance_automatic = params.white_balance_automatic
    node.white_balance_temperature = params.white_balance_temperature
    node.power_line_frequency = params.power_line_frequency
    node.metadata_enabled = params.metadata_enabled
    node.metadata_device = params.metadata_device
    node.metadata_topic = params.metadata_topic
    node.metadata_log_path = params.metadata_log_path
    node.metadata_log_flush_every_n = params.metadata_log_flush_every_n
    node.vignette_enabled = params.vignette_enabled
    node.vignette_flatfield_b_path = params.vignette_flatfield_b_path
    node.vignette_flatfield_g_path = params.vignette_flatfield_g_path
    node.vignette_flatfield_r_path = params.vignette_flatfield_r_path
    node.vignette_dark_current = params.vignette_dark_current
    node.width = params.req_width
    node.height = params.req_height


def build_publisher_qos(params: CameraNodeParameters) -> QoSProfile:
    """Build the shared publisher QoS profile for image-like topics."""
    reliability = (
        ReliabilityPolicy.BEST_EFFORT
        if params.qos_best_effort
        else ReliabilityPolicy.RELIABLE
    )
    return QoSProfile(
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=params.qos_depth,
    )


def log_parameter_banner(node, params: CameraNodeParameters) -> None:
    """Emit a debug banner showing the effective runtime camera configuration."""
    node.get_logger().info(
        "Parameter priority: launch/CLI overrides > params file > node defaults"
    )
    node.get_logger().info(
        "Params: "
        f"video_device={node.video_device}, size={node.req_width}x{node.req_height}, "
        f"fps={node.req_fps}, fmt={node.pixel_format}, frame_id={node.frame_id}, "
        f"camera_name={node.camera_name}, camera_info_url={node.camera_info_url!r}, "
        f"use_gstreamer={node.use_gstreamer}, "
        f"qos_best_effort={node.qos_best_effort}, qos_depth={node.qos_depth}, "
        f"uvc_controls_enabled={node.uvc_controls_enabled}, "
        f"uvc_controls_device={node.uvc_controls_device!r}, "
        f"auto_exposure_mode={node.auto_exposure_mode}, "
        f"exposure_time_absolute={node.exposure_time_absolute}, "
        f"gain={node.gain}, "
        f"exposure_dynamic_framerate={node.exposure_dynamic_framerate}, "
        f"white_balance_automatic={node.white_balance_automatic}, "
        f"white_balance_temperature={node.white_balance_temperature}, "
        f"power_line_frequency={node.power_line_frequency}, "
        f"metadata_enabled={node.metadata_enabled}, metadata_device={node.metadata_device}, "
        f"metadata_topic={node.metadata_topic}, metadata_log_path={node.metadata_log_path!r}, "
        f"metadata_log_flush_every_n={node.metadata_log_flush_every_n}, "
        f"vignette_enabled={node.vignette_enabled}, "
        f"vignette_flatfield_b_path={node.vignette_flatfield_b_path!r}, "
        f"vignette_flatfield_g_path={node.vignette_flatfield_g_path!r}, "
        f"vignette_flatfield_r_path={node.vignette_flatfield_r_path!r}, "
        f"vignette_dark_current={node.vignette_dark_current}, "
        f"debug_period_s={node.debug_period_s}"
    )
