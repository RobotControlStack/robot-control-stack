from abc import ABC, abstractmethod
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict


class BaseDataClass(BaseModel):
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
    )


class GenericCameraConfig(BaseDataClass): ...


class CameraFrame(BaseDataClass):
    color: np.ndarray
    ir: np.ndarray | None
    depth: np.ndarray | None
    temperature: float | None


class IMUFrame(BaseDataClass):
    acc_sample: np.ndarray[Literal[3], np.dtype[np.float32]]
    acc_sample_usec: float | None
    gyro_sample: np.ndarray[Literal[3], np.dtype[np.float32]]
    gyro_sample_usec: float | None
    temperature: float | None


class Frame(BaseDataClass):
    camera: CameraFrame
    imu: IMUFrame | None


class Camera(ABC):

    @property
    @abstractmethod
    def config(self) -> GenericCameraConfig: ...

    # @config.setter
    # @abstractmethod
    # def config(self, cfg: GenericCameraConfig) -> None: ...

    @abstractmethod
    def get_current_frame(self) -> Frame: ...
