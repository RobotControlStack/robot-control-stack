import os
from typing import Optional

import mujoco
from rcs._core.common import Pose


class ModelComposer:
    """
    Composes MuJoCo scenes using mjSpec with flexible positioning and prefixing.
    """

    def __init__(
        self,
        model_name: str = "rcs_scene",
        add_gravcomp: bool = False,
    ):
        self.spec = mujoco.MjSpec()
        self.spec.modelname = model_name
        self.spec.compiler.autolimits = True
        self.add_gravcomp = add_gravcomp
        self._gravcomp_prefixes: set[str] = set()

    def _resolve_asset_paths(self, spec: mujoco.MjSpec, xml_path: str):
        """Resolves relative paths to absolute ones."""
        if spec.compiler.autolimits:
            self.spec.compiler.autolimits = True

        xml_dir = os.path.dirname(os.path.abspath(xml_path))
        mesh_base = os.path.join(xml_dir, spec.meshdir or "")
        tex_base = os.path.join(xml_dir, spec.texturedir or "")

        for mesh in spec.meshes:
            if mesh.file and not os.path.isabs(mesh.file):
                mesh.file = os.path.abspath(os.path.join(mesh_base, mesh.file))
        for tex in spec.textures:
            if tex.file and not os.path.isabs(tex.file):
                tex.file = os.path.abspath(os.path.join(tex_base, tex.file))

    def load_base_scene(self, xml_path: str):
        """Loads the base world XML."""
        if not os.path.exists(xml_path):
            msg = f"Base scene XML not found: {xml_path}"
            raise FileNotFoundError(msg)
        self.spec = mujoco.MjSpec.from_file(xml_path)
        self._resolve_asset_paths(self.spec, xml_path)

    def _find_site(self, name: str) -> Optional[mujoco._specs.MjsSite]:
        for s in self.spec.sites:
            if s.name == name:
                return s
        return None

    def _find_body(self, name: str) -> Optional[mujoco._specs.MjsBody]:
        try:
            return self.spec.find_body(name)
        except ValueError:
            return None

    def _find_camera(self, name: str) -> Optional[mujoco._specs.MjsCamera]:
        for camera in self.spec.cameras:
            if camera.name == name:
                return camera
        return None

    def _apply_pose(self, body: mujoco._specs.MjsBody, pose: Pose):
        body.pos = list(pose.translation())
        body.quat = list(pose.rotation_q_wxyz())

    def add_camera(
        self,
        resolution: tuple[int, int],
        fovy: float,
        name: str,
        pose: Pose | None = None,
        robot_prefix: str | None = None,
        attachment_site_name: str = "attachment_site",
    ) -> mujoco._specs.MjsCamera:
        """Adds a fixed camera to the world frame or to a robot attachment site."""
        if pose is None:
            pose = Pose()
        if robot_prefix is None:
            camera_mount = self.spec.worldbody.add_body()
            camera_mount.name = f"{name}_mount"
        else:
            site_name = robot_prefix + attachment_site_name
            attachment_site = self._find_site(site_name)

            if not attachment_site:
                msg = f"Attachment site '{site_name}' not found."
                raise ValueError(msg)

            camera_mount = mujoco.MjSpec().worldbody.add_body()
            camera_mount.name = "mount"
            camera_mount = attachment_site.attach(camera_mount, f"{name}_", "")

        self._apply_pose(camera_mount, pose)

        camera = camera_mount.add_camera()
        camera.name = name
        camera.resolution = resolution
        camera.fovy = fovy

        return camera

    def add_camera_xml(
        self,
        xml_path: str,
        name: str,
        pose: Pose | None = None,
        robot_prefix: str | None = None,
        attachment_site_name: str = "attachment_site",
    ) -> mujoco._specs.MjsCamera:
        """Attaches a camera MJCF to the world frame or to a robot attachment site."""
        if pose is None:
            pose = Pose()
        if not os.path.exists(xml_path):
            msg = f"Camera MJCF not found: {xml_path}"
            raise FileNotFoundError(msg)

        camera_spec = mujoco.MjSpec.from_file(xml_path)
        self._resolve_asset_paths(camera_spec, xml_path)

        root_body = camera_spec.worldbody.first_body()
        if root_body is None:
            msg = f"Camera MJCF '{xml_path}' must contain a root body."
            raise ValueError(msg)

        camera_names = [camera.name for camera in camera_spec.cameras]
        if len(camera_names) != 1 or camera_names[0] is None:
            msg = f"Camera MJCF '{xml_path}' must contain exactly one named camera."
            raise ValueError(msg)
        original_camera_name = camera_names[0]
        prefix = f"{name}_"

        if robot_prefix is None:
            attach_frame = self.spec.worldbody.add_frame()
            attach_frame.attach(camera_spec, prefix, "")
            attached_root_name = f"{prefix}{root_body.name}"
            attached_root = self._find_body(attached_root_name)
            if attached_root is None:
                msg = f"Could not find camera root body '{attached_root_name}' after attachment."
                raise ValueError(msg)
        else:
            site_name = robot_prefix + attachment_site_name
            attachment_site = self._find_site(site_name)

            if not attachment_site:
                msg = f"Attachment site '{site_name}' not found."
                raise ValueError(msg)

            attached_root = attachment_site.attach(root_body, prefix, "")

        self._apply_pose(attached_root, pose)

        attached_camera_name = f"{prefix}{original_camera_name}"
        camera = self._find_camera(attached_camera_name)
        if camera is None:
            msg = f"Could not find camera '{attached_camera_name}' after attachment."
            raise ValueError(msg)
        camera.name = name

        return camera

    def add_robot(self, xml_path: str, prefix: str, pose: Pose | None = None) -> mujoco._specs.MjsBody:
        """
        Attaches a robot MJCF at a specific pose.
        """
        if pose is None:
            pose = Pose()
        if not os.path.exists(xml_path):
            msg = f"Robot MJCF not found: {xml_path}"
            raise FileNotFoundError(msg)

        child_spec = mujoco.MjSpec.from_file(xml_path)
        self._resolve_asset_paths(child_spec, xml_path)

        # 1. Use a frame to attach the child spec (most comprehensive)
        frame = self.spec.worldbody.add_frame()
        frame.attach(child_spec, prefix, "")

        # 2. Identify the robot root body (it was prefixed)
        child_root_name = child_spec.worldbody.first_body().name
        prefixed_root_name = f"{prefix}{child_root_name}"

        robot_root = self._find_body(prefixed_root_name)
        if not robot_root:
            msg = f"Could not find robot root body '{prefixed_root_name}' after attachment."
            raise ValueError(msg)

        # 3. Apply the pose directly to the body
        self._apply_pose(robot_root, pose)
        if prefix:
            self._gravcomp_prefixes.add(prefix)

        return robot_root

    def add_gripper(
        self,
        xml_path: str,
        robot_prefix: str,
        gripper_prefix: str = "gripper_",
        attachment_site_name: str = "attachment_site",
        pose: Pose | None = None,
    ) -> mujoco._specs.MjsBody:
        """Attaches a gripper to a robot's attachment site with an optional local pose offset."""
        if pose is None:
            pose = Pose()
        site_name = robot_prefix + attachment_site_name
        attachment_site = self._find_site(site_name)

        if not attachment_site:
            msg = f"Attachment site '{site_name}' not found."
            raise ValueError(msg)

        gripper_spec = mujoco.MjSpec.from_file(xml_path)
        self._resolve_asset_paths(gripper_spec, xml_path)

        gripper_root = gripper_spec.worldbody.first_body()
        gripper_root = attachment_site.attach(gripper_root, gripper_prefix, "")
        self._apply_pose(gripper_root, pose)
        if gripper_prefix:
            self._gravcomp_prefixes.add(gripper_prefix)
        return gripper_root

    def add_object_robot_frame(
        self,
        xml_path: str,
        robot_prefix: str,
        object_prefix: str,
        attachment_site_name: str,
        pose: Pose | None = None,
    ) -> mujoco._specs.MjsBody:
        """Attaches an object to a robot attachment site with an optional local pose offset."""
        if pose is None:
            pose = Pose()
        site_name = robot_prefix + attachment_site_name
        attachment_site = self._find_site(site_name)

        if not attachment_site:
            msg = f"Attachment site '{site_name}' not found."
            raise ValueError(msg)

        object_spec = mujoco.MjSpec.from_file(xml_path)
        self._resolve_asset_paths(object_spec, xml_path)

        object_root = object_spec.worldbody.first_body()
        object_root = attachment_site.attach(object_root, object_prefix, "")
        self._apply_pose(object_root, pose)
        return object_root

    def add_object_world_frame(
        self, xml_path: str, object_prefix: str, pose: Pose | None = None
    ) -> mujoco._specs.MjsBody:
        """
        Attaches a single object MJCF at a specific pose.
        Assumes the XML contains only one root body in the worldbody.
        """
        if pose is None:
            pose = Pose()
        if not os.path.exists(xml_path):
            msg = f"Object MJCF not found: {xml_path}"
            raise FileNotFoundError(msg)

        # Load the child spec
        child_spec = mujoco.MjSpec.from_file(xml_path)
        self._resolve_asset_paths(child_spec, xml_path)

        # Attach using a frame
        frame = self.spec.worldbody.add_frame()
        frame.attach(child_spec, object_prefix, "")

        # Identify the root of the added object
        child_root_name = child_spec.worldbody.first_body().name
        prefixed_root_name = f"{object_prefix}{child_root_name}"
        obj_root = self._find_body(prefixed_root_name)
        if not obj_root:
            msg = f"Could not find object root body '{prefixed_root_name}' after attachment."
            raise ValueError(msg)

        # Apply the pose
        self._apply_pose(obj_root, pose)

        return obj_root

    def save_mjcf(self, output_path: str):
        """Compiles and saves the MJCF."""
        self._apply_gravcomp()
        self.spec.compile()
        xml_str = self.spec.to_xml()
        with open(output_path, "w") as f:
            f.write(xml_str)
        return xml_str

    def get_model(self):
        self._apply_gravcomp()
        return self.spec.compile()

    def _apply_gravcomp(self):
        if not self.add_gravcomp or not self._gravcomp_prefixes:
            return

        for body in self.spec.bodies:
            if body.name and any(body.name.startswith(prefix) for prefix in self._gravcomp_prefixes):
                body.gravcomp = 1

        for joint in self.spec.joints:
            if joint.name and any(joint.name.startswith(prefix) for prefix in self._gravcomp_prefixes):
                joint.actgravcomp = True
